"""
  UWB Fusion Accuracy Test GUI
  
  Real-time error graph and EKF configuration
"""

import wx
import wx.lib.plot as wxplot
import math
import time

from MAVProxy.modules.lib.multiproc import Process, Queue


class UWBTestGUI:
    """GUI manager that runs in separate process"""

    def __init__(self, module):
        self.module = module
        self.cmd_queue = Queue()
        self.data_queue = Queue()
        self.process = Process(target=self._run_gui, args=(self.cmd_queue, self.data_queue))
        self.process.start()
        self._alive = True

    def is_alive(self):
        return self._alive and self.process.is_alive()

    def close(self):
        self._alive = False
        try:
            self.cmd_queue.put(('close', None))
        except Exception:
            pass

    def update_data(self, data):
        """Send data update to GUI"""
        if not self.is_alive():
            return
        try:
            self.cmd_queue.put(('update', data))
        except Exception:
            pass

        # Check for commands from GUI
        self._process_gui_commands()

    def _process_gui_commands(self):
        """Process commands from GUI"""
        while True:
            try:
                if self.data_queue.empty():
                    break
                cmd = self.data_queue.get()
                if cmd is None:
                    break
            except Exception:
                break

            cmd_type, data = cmd
            if cmd_type == 'start':
                self.module.cmd_start()
            elif cmd_type == 'stop':
                self.module.cmd_stop()
            elif cmd_type == 'analyze':
                self.module.cmd_analyze()
            elif cmd_type == 'save':
                self.module.cmd_save(data)
            elif cmd_type == 'config':
                self.module.cmd_config(data)
            elif cmd_type == 'closed':
                self._alive = False

    def _run_gui(self, cmd_queue, data_queue):
        """Run GUI in separate process"""
        app = wx.App()
        frame = UWBTestFrame(cmd_queue, data_queue)
        frame.Show()
        app.MainLoop()


class UWBTestFrame(wx.Frame):
    """Main GUI window"""

    def __init__(self, cmd_queue, data_queue):
        super(UWBTestFrame, self).__init__(None, title="UWB Fusion Accuracy Test", size=(1000, 800))
        self.cmd_queue = cmd_queue
        self.data_queue = data_queue

        self.graph_data = {
            'time': [],
            'err_n': [],
            'err_e': [],
            'err_d': [],
            'err_horiz': []
        }

        self._create_ui()
        self._create_menu()

        # Timer for processing commands
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(50)  # 20 Hz

        self.Bind(wx.EVT_CLOSE, self._on_close)

    def _create_menu(self):
        menubar = wx.MenuBar()

        file_menu = wx.Menu()
        save_json = file_menu.Append(wx.ID_ANY, "Save JSON\tCtrl+S", "Save as JSON")
        save_csv = file_menu.Append(wx.ID_ANY, "Save CSV", "Save as CSV")
        file_menu.AppendSeparator()
        exit_item = file_menu.Append(wx.ID_EXIT, "Exit\tCtrl+Q", "Exit")

        self.Bind(wx.EVT_MENU, self._on_save_json, save_json)
        self.Bind(wx.EVT_MENU, self._on_save_csv, save_csv)
        self.Bind(wx.EVT_MENU, self._on_close, exit_item)

        menubar.Append(file_menu, "&File")
        self.SetMenuBar(menubar)

    def _create_ui(self):
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # === Status Section ===
        status_box = wx.StaticBox(panel, label="Test Status")
        status_sizer = wx.StaticBoxSizer(status_box, wx.HORIZONTAL)

        # Left: Test status
        left_sizer = wx.BoxSizer(wx.VERTICAL)
        
        self.status_text = wx.StaticText(panel, label="Status: IDLE")
        self.status_text.SetFont(wx.Font(12, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        left_sizer.Add(self.status_text, 0, wx.ALL, 5)

        info_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.elapsed_text = wx.StaticText(panel, label="Elapsed: 0.0 s")
        self.samples_text = wx.StaticText(panel, label="Samples: 0")
        self.rtk_text = wx.StaticText(panel, label="RTK: ---")
        info_sizer.Add(self.elapsed_text, 0, wx.RIGHT, 20)
        info_sizer.Add(self.samples_text, 0, wx.RIGHT, 20)
        info_sizer.Add(self.rtk_text, 0)
        left_sizer.Add(info_sizer, 0, wx.ALL, 5)

        status_sizer.Add(left_sizer, 1, wx.EXPAND)

        # Right: Control buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.start_btn = wx.Button(panel, label="Start Test", size=(100, 40))
        self.stop_btn = wx.Button(panel, label="Stop Test", size=(100, 40))
        self.analyze_btn = wx.Button(panel, label="Analyze", size=(100, 40))
        
        self.start_btn.SetBackgroundColour(wx.Colour(100, 200, 100))
        self.stop_btn.SetBackgroundColour(wx.Colour(200, 100, 100))
        self.stop_btn.Enable(False)

        self.start_btn.Bind(wx.EVT_BUTTON, self._on_start)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.analyze_btn.Bind(wx.EVT_BUTTON, self._on_analyze)

        btn_sizer.Add(self.start_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(self.stop_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(self.analyze_btn, 0)
        status_sizer.Add(btn_sizer, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 10)

        main_sizer.Add(status_sizer, 0, wx.EXPAND | wx.ALL, 10)

        # === Current Error Section ===
        error_box = wx.StaticBox(panel, label="Current Error (EKF - RTK)")
        error_sizer = wx.StaticBoxSizer(error_box, wx.HORIZONTAL)

        self.err_n_text = wx.StaticText(panel, label="N: --- mm", size=(120, -1))
        self.err_e_text = wx.StaticText(panel, label="E: --- mm", size=(120, -1))
        self.err_d_text = wx.StaticText(panel, label="D: --- mm", size=(120, -1))
        self.err_horiz_text = wx.StaticText(panel, label="Horiz: --- mm", size=(130, -1))
        self.err_total_text = wx.StaticText(panel, label="Total: --- mm", size=(130, -1))

        font = wx.Font(11, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        for txt in [self.err_n_text, self.err_e_text, self.err_d_text, 
                    self.err_horiz_text, self.err_total_text]:
            txt.SetFont(font)

        error_sizer.Add(self.err_n_text, 0, wx.ALL, 10)
        error_sizer.Add(self.err_e_text, 0, wx.ALL, 10)
        error_sizer.Add(self.err_d_text, 0, wx.ALL, 10)
        error_sizer.Add(self.err_horiz_text, 0, wx.ALL, 10)
        error_sizer.Add(self.err_total_text, 0, wx.ALL, 10)

        main_sizer.Add(error_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)

        # === EKF Configuration Section ===
        config_box = wx.StaticBox(panel, label="EKF Fusion Configuration (PX4)")
        config_sizer = wx.StaticBoxSizer(config_box, wx.HORIZONTAL)

        # GPS Control
        gps_sizer = wx.BoxSizer(wx.VERTICAL)
        gps_sizer.Add(wx.StaticText(panel, label="GPS Fusion:"), 0, wx.BOTTOM, 5)
        self.gps_ctrl_spin = wx.SpinCtrl(panel, min=0, max=7, initial=7, size=(60, -1))
        self.gps_ctrl_text = wx.StaticText(panel, label="(0=off, 7=all)")
        gps_h = wx.BoxSizer(wx.HORIZONTAL)
        gps_h.Add(self.gps_ctrl_spin, 0, wx.RIGHT, 5)
        gps_h.Add(self.gps_ctrl_text, 0, wx.ALIGN_CENTER_VERTICAL)
        gps_sizer.Add(gps_h, 0)
        config_sizer.Add(gps_sizer, 0, wx.ALL, 10)

        # UWB Beacon
        uwbb_sizer = wx.BoxSizer(wx.VERTICAL)
        uwbb_sizer.Add(wx.StaticText(panel, label="UWB Beacon:"), 0, wx.BOTTOM, 5)
        self.uwbb_check = wx.CheckBox(panel, label="Enable")
        uwbb_sizer.Add(self.uwbb_check, 0)
        config_sizer.Add(uwbb_sizer, 0, wx.ALL, 10)

        # UWB Tag
        uwbt_sizer = wx.BoxSizer(wx.VERTICAL)
        uwbt_sizer.Add(wx.StaticText(panel, label="UWB Tag:"), 0, wx.BOTTOM, 5)
        self.uwbt_check = wx.CheckBox(panel, label="Enable")
        uwbt_sizer.Add(self.uwbt_check, 0)
        config_sizer.Add(uwbt_sizer, 0, wx.ALL, 10)

        # Height Reference
        hgt_sizer = wx.BoxSizer(wx.VERTICAL)
        hgt_sizer.Add(wx.StaticText(panel, label="Height Ref:"), 0, wx.BOTTOM, 5)
        self.hgt_choice = wx.Choice(panel, choices=["Baro", "GPS", "Range", "Vision"])
        hgt_sizer.Add(self.hgt_choice, 0)
        config_sizer.Add(hgt_sizer, 0, wx.ALL, 10)

        # Apply button
        self.apply_btn = wx.Button(panel, label="Apply Config")
        self.apply_btn.Bind(wx.EVT_BUTTON, self._on_apply_config)
        config_sizer.Add(self.apply_btn, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 10)

        main_sizer.Add(config_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)

        # === Graph Section ===
        graph_box = wx.StaticBox(panel, label="Error History (mm)")
        graph_sizer = wx.StaticBoxSizer(graph_box, wx.VERTICAL)

        # Create a container panel for the plot
        self.graph_panel = wx.Panel(panel)
        self.graph_panel.SetMinSize(wx.Size(950, 350))
        graph_panel_sizer = wx.BoxSizer(wx.VERTICAL)
        self.graph_panel.SetSizer(graph_panel_sizer)
        
        # PlotCanvas will be created on first update
        self.plot_canvas = None
        
        graph_sizer.Add(self.graph_panel, 1, wx.EXPAND | wx.ALL, 5)

        main_sizer.Add(graph_sizer, 1, wx.EXPAND | wx.ALL, 10)

        # === Status Bar ===
        self.CreateStatusBar()
        self.SetStatusText("Ready - Configure EKF and start test")

        panel.SetSizer(main_sizer)
        
        # Defer PlotCanvas creation until after window is fully shown
        wx.CallLater(100, self._create_plot_canvas)

    def _create_plot_canvas(self):
        """Create PlotCanvas after window is shown"""
        if self.plot_canvas is not None:
            return
        try:
            self.plot_canvas = wxplot.PlotCanvas(self.graph_panel)
            self.plot_canvas.enableGrid = True
            self.plot_canvas.enableLegend = True
            self.plot_canvas.fontSizeLegend = 8
            self.graph_panel.GetSizer().Add(self.plot_canvas, 1, wx.EXPAND)
            self.graph_panel.Layout()
        except Exception as e:
            print("Failed to create PlotCanvas: %s" % e)
            self.plot_canvas = None

    def _on_timer(self, event):
        """Process commands from main module"""
        while True:
            try:
                if self.cmd_queue.empty():
                    break
                cmd = self.cmd_queue.get()
                if cmd is None:
                    break
            except Exception:
                break

            cmd_type, data = cmd
            if cmd_type == 'close':
                self.Close()
            elif cmd_type == 'update':
                self._update_display(data)

    def _update_display(self, data):
        """Update display with new data"""
        # Update status
        if data['testing']:
            self.status_text.SetLabel("Status: TESTING")
            self.status_text.SetForegroundColour(wx.Colour(200, 0, 0))
            self.start_btn.Enable(False)
            self.stop_btn.Enable(True)
        else:
            self.status_text.SetLabel("Status: IDLE")
            self.status_text.SetForegroundColour(wx.Colour(0, 100, 0))
            self.start_btn.Enable(True)
            self.stop_btn.Enable(False)

        self.elapsed_text.SetLabel("Elapsed: %.1f s" % data['elapsed'])
        self.samples_text.SetLabel("Samples: %d" % data['sample_count'])

        fix_names = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 
                     4: "DGPS", 5: "Float", 6: "Fixed"}
        rtk_color = wx.Colour(0, 128, 0) if data['rtk_fix'] >= 5 else wx.Colour(200, 0, 0)
        self.rtk_text.SetLabel("RTK: %s (%d sats)" % (
            fix_names.get(data['rtk_fix'], '?'), data['rtk_sats']))
        self.rtk_text.SetForegroundColour(rtk_color)

        # Update error display
        if data['err_n_mm'] is not None:
            self.err_n_text.SetLabel("N: %+.1f mm" % data['err_n_mm'])
            self.err_e_text.SetLabel("E: %+.1f mm" % data['err_e_mm'])
            self.err_d_text.SetLabel("D: %+.1f mm" % data['err_d_mm'])
            self.err_horiz_text.SetLabel("Horiz: %.1f mm" % data['err_horiz_mm'])
            self.err_total_text.SetLabel("Total: %.1f mm" % data['err_total_mm'])
        else:
            for txt in [self.err_n_text, self.err_e_text, self.err_d_text,
                       self.err_horiz_text, self.err_total_text]:
                txt.SetLabel(txt.GetLabel().split(':')[0] + ": --- mm")

        # Update config display
        params = data['params']
        if params['gps_ctrl'] >= 0:
            self.gps_ctrl_spin.SetValue(int(params['gps_ctrl']))
        if params['uwbb_ctrl'] >= 0:
            self.uwbb_check.SetValue(int(params['uwbb_ctrl']) > 0)
        if params['uwbt_ctrl'] >= 0:
            self.uwbt_check.SetValue(int(params['uwbt_ctrl']) > 0)
        if params['hgt_ref'] >= 0:
            self.hgt_choice.SetSelection(int(params['hgt_ref']))

        # Update graph
        self._update_graph(data['graph_samples'])

    def _update_graph(self, samples):
        """Update error graph"""
        if self.plot_canvas is None:
            return
        if len(samples) == 0:
            return

        times = [s['time'] for s in samples]
        err_n = [s['err_n'] for s in samples]
        err_e = [s['err_e'] for s in samples]
        err_d = [s['err_d'] for s in samples]
        err_horiz = [s['err_horiz'] for s in samples]

        lines = []

        # Create plot lines
        if len(times) > 1:
            data_n = list(zip(times, err_n))
            data_e = list(zip(times, err_e))
            data_d = list(zip(times, err_d))
            data_h = list(zip(times, err_horiz))

            lines.append(wxplot.PolyLine(data_n, colour='red', width=1, legend='N'))
            lines.append(wxplot.PolyLine(data_e, colour='green', width=1, legend='E'))
            lines.append(wxplot.PolyLine(data_d, colour='blue', width=1, legend='D'))
            lines.append(wxplot.PolyLine(data_h, colour='black', width=2, legend='Horiz'))

            gc = wxplot.PlotGraphics(lines, 'Error vs Time', 'Time (s)', 'Error (mm)')
            
            # Auto-scale Y axis
            all_errors = err_n + err_e + err_d + err_horiz
            y_max = max(abs(min(all_errors)), abs(max(all_errors)), 100) * 1.1
            
            try:
                self.plot_canvas.Draw(gc, xAxis=(times[0], times[-1]), yAxis=(-y_max, y_max))
            except Exception:
                pass

    def _on_start(self, event):
        """Start test button"""
        self.data_queue.put(('start', None))

    def _on_stop(self, event):
        """Stop test button"""
        self.data_queue.put(('stop', None))

    def _on_analyze(self, event):
        """Analyze button"""
        self.data_queue.put(('analyze', None))

    def _on_apply_config(self, event):
        """Apply configuration"""
        # GPS Control
        self.data_queue.put(('config', ['gps', str(self.gps_ctrl_spin.GetValue())]))
        
        # UWB Beacon
        self.data_queue.put(('config', ['uwb_beacon', '1' if self.uwbb_check.GetValue() else '0']))
        
        # UWB Tag
        self.data_queue.put(('config', ['uwb_tag', '1' if self.uwbt_check.GetValue() else '0']))
        
        # Height Reference
        self.data_queue.put(('config', ['height', str(self.hgt_choice.GetSelection())]))
        
        self.SetStatusText("Configuration applied")

    def _on_save_json(self, event):
        """Save as JSON"""
        dlg = wx.FileDialog(self, "Save Test Data", wildcard="JSON files (*.json)|*.json",
                            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)
        if dlg.ShowModal() == wx.ID_OK:
            self.data_queue.put(('save', dlg.GetPath()))
        dlg.Destroy()

    def _on_save_csv(self, event):
        """Save as CSV"""
        dlg = wx.FileDialog(self, "Save Test Data", wildcard="CSV files (*.csv)|*.csv",
                            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)
        if dlg.ShowModal() == wx.ID_OK:
            self.data_queue.put(('save', dlg.GetPath()))
        dlg.Destroy()

    def _on_close(self, event):
        """Window closing"""
        self.timer.Stop()
        self.data_queue.put(('closed', None))
        self.Destroy()
