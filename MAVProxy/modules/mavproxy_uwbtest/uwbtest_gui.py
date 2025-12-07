"""
  UWB Fusion Accuracy Test GUI - Small Screen Version
  Optimized for 3.5 inch screens (480x280)
"""

import math
import time

try:
    import wx
except ImportError:
    wx = None

from MAVProxy.modules.lib.multiproc import Process, Queue

# Screen size for 3.5 inch display
SMALL_SCREEN_WIDTH = 480
SMALL_SCREEN_HEIGHT = 280


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
        import wx
        app = wx.App()
        frame = UWBTestFrame(cmd_queue, data_queue)
        frame.Show()
        app.MainLoop()


class UWBTestFrame(wx.Frame):
    """Main GUI window - Tabbed interface for small screens"""

    def __init__(self, cmd_queue, data_queue):
        import wx
        super(UWBTestFrame, self).__init__(
            None, 
            title="UWB Test", 
            size=(SMALL_SCREEN_WIDTH, SMALL_SCREEN_HEIGHT),
            style=wx.DEFAULT_FRAME_STYLE & ~(wx.RESIZE_BORDER | wx.MAXIMIZE_BOX)
        )
        self.cmd_queue = cmd_queue
        self.data_queue = data_queue

        # Fonts
        self.small_font = wx.Font(7, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.mono_font = wx.Font(7, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.bold_font = wx.Font(7, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        self.large_font = wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)

        self._create_ui()

        # Timer for processing commands
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(50)

        self.Bind(wx.EVT_CLOSE, self._on_close)

    def _create_ui(self):
        """Create tabbed interface"""
        import wx
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create notebook (tabs)
        self.notebook = wx.Notebook(panel)

        # Tab 1: Status & Error
        self.status_panel = wx.Panel(self.notebook)
        self._create_status_tab(self.status_panel)
        self.notebook.AddPage(self.status_panel, "Status")

        # Tab 2: Config
        self.config_panel = wx.Panel(self.notebook)
        self._create_config_tab(self.config_panel)
        self.notebook.AddPage(self.config_panel, "Config")

        # Tab 3: Stats
        self.stats_panel = wx.Panel(self.notebook)
        self._create_stats_tab(self.stats_panel)
        self.notebook.AddPage(self.stats_panel, "Stats")

        main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 1)
        panel.SetSizer(main_sizer)

    def _create_status_tab(self, panel):
        """Create status and error display tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Status row
        status_row = wx.BoxSizer(wx.HORIZONTAL)
        self.status_text = wx.StaticText(panel, label="IDLE")
        self.status_text.SetFont(self.large_font)
        self.status_text.SetForegroundColour(wx.Colour(0, 100, 0))
        status_row.Add(self.status_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.elapsed_text = wx.StaticText(panel, label="0.0s")
        self.elapsed_text.SetFont(self.mono_font)
        status_row.Add(self.elapsed_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)

        self.samples_text = wx.StaticText(panel, label="0 smp")
        self.samples_text.SetFont(self.mono_font)
        status_row.Add(self.samples_text, 0, wx.ALIGN_CENTER_VERTICAL)

        sizer.Add(status_row, 0, wx.ALL, 3)

        # RTK status
        rtk_row = wx.BoxSizer(wx.HORIZONTAL)
        rtk_label = wx.StaticText(panel, label="RTK:")
        rtk_label.SetFont(self.bold_font)
        rtk_row.Add(rtk_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.rtk_text = wx.StaticText(panel, label="---")
        self.rtk_text.SetFont(self.mono_font)
        rtk_row.Add(self.rtk_text, 0, wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(rtk_row, 0, wx.LEFT | wx.RIGHT, 3)

        # Error box
        err_box = wx.StaticBox(panel, label="Error (mm)")
        err_sizer = wx.StaticBoxSizer(err_box, wx.VERTICAL)

        # NED errors
        ned_row = wx.BoxSizer(wx.HORIZONTAL)
        self.err_n_text = wx.StaticText(panel, label="N:---", size=(55, -1))
        self.err_e_text = wx.StaticText(panel, label="E:---", size=(55, -1))
        self.err_d_text = wx.StaticText(panel, label="D:---", size=(55, -1))
        self.err_n_text.SetFont(self.large_font)
        self.err_e_text.SetFont(self.large_font)
        self.err_d_text.SetFont(self.large_font)
        ned_row.Add(self.err_n_text, 1)
        ned_row.Add(self.err_e_text, 1)
        ned_row.Add(self.err_d_text, 1)
        err_sizer.Add(ned_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 2)

        # Horiz/Total
        total_row = wx.BoxSizer(wx.HORIZONTAL)
        self.err_horiz_text = wx.StaticText(panel, label="H:---", size=(70, -1))
        self.err_total_text = wx.StaticText(panel, label="T:---", size=(70, -1))
        self.err_horiz_text.SetFont(self.large_font)
        self.err_total_text.SetFont(self.large_font)
        total_row.Add(self.err_horiz_text, 1)
        total_row.Add(self.err_total_text, 1)
        err_sizer.Add(total_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 2)

        sizer.Add(err_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.start_btn = wx.Button(panel, label="Start", size=(65, 24))
        self.stop_btn = wx.Button(panel, label="Stop", size=(65, 24))
        self.analyze_btn = wx.Button(panel, label="Analyze", size=(65, 24))
        self.save_btn = wx.Button(panel, label="Save", size=(65, 24))

        self.stop_btn.Enable(False)

        self.start_btn.Bind(wx.EVT_BUTTON, self._on_start)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.analyze_btn.Bind(wx.EVT_BUTTON, self._on_analyze)
        self.save_btn.Bind(wx.EVT_BUTTON, self._on_save)

        btn_sizer.Add(self.start_btn, 1, wx.RIGHT, 2)
        btn_sizer.Add(self.stop_btn, 1, wx.RIGHT, 2)
        btn_sizer.Add(self.analyze_btn, 1, wx.RIGHT, 2)
        btn_sizer.Add(self.save_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_config_tab(self, panel):
        """Create EKF configuration tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # GPS Control
        gps_box = wx.StaticBox(panel, label="GPS Fusion")
        gps_sizer = wx.StaticBoxSizer(gps_box, wx.HORIZONTAL)
        gps_label = wx.StaticText(panel, label="Mode:")
        gps_label.SetFont(self.small_font)
        gps_sizer.Add(gps_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.gps_ctrl_spin = wx.SpinCtrl(panel, min=0, max=7, initial=7, size=(45, -1))
        gps_sizer.Add(self.gps_ctrl_spin, 0, wx.RIGHT, 3)
        gps_hint = wx.StaticText(panel, label="0=off,7=all")
        gps_hint.SetFont(self.small_font)
        gps_sizer.Add(gps_hint, 0, wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(gps_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # UWB Controls
        uwb_box = wx.StaticBox(panel, label="UWB Fusion")
        uwb_sizer = wx.StaticBoxSizer(uwb_box, wx.HORIZONTAL)
        self.uwbb_check = wx.CheckBox(panel, label="Beacon")
        self.uwbt_check = wx.CheckBox(panel, label="Tag")
        self.uwbb_check.SetFont(self.small_font)
        self.uwbt_check.SetFont(self.small_font)
        uwb_sizer.Add(self.uwbb_check, 0, wx.RIGHT, 10)
        uwb_sizer.Add(self.uwbt_check, 0)
        sizer.Add(uwb_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Height Reference
        hgt_box = wx.StaticBox(panel, label="Height Ref")
        hgt_sizer = wx.StaticBoxSizer(hgt_box, wx.HORIZONTAL)
        self.hgt_choice = wx.Choice(panel, choices=["Baro", "GPS", "Range", "Vision"], size=(70, -1))
        hgt_sizer.Add(self.hgt_choice, 0)
        sizer.Add(hgt_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Apply button
        self.apply_btn = wx.Button(panel, label="Apply Config", size=(100, 24))
        self.apply_btn.Bind(wx.EVT_BUTTON, self._on_apply_config)
        sizer.Add(self.apply_btn, 0, wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_stats_tab(self, panel):
        """Create statistics tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Statistics display
        stats_box = wx.StaticBox(panel, label="Test Statistics (mm)")
        stats_sizer = wx.StaticBoxSizer(stats_box, wx.VERTICAL)

        # Mean errors
        mean_label = wx.StaticText(panel, label="Mean Error:")
        mean_label.SetFont(self.bold_font)
        stats_sizer.Add(mean_label, 0, wx.LEFT | wx.TOP, 3)

        mean_row = wx.BoxSizer(wx.HORIZONTAL)
        self.mean_n = wx.StaticText(panel, label="N:---", size=(55, -1))
        self.mean_e = wx.StaticText(panel, label="E:---", size=(55, -1))
        self.mean_d = wx.StaticText(panel, label="D:---", size=(55, -1))
        self.mean_h = wx.StaticText(panel, label="H:---", size=(55, -1))
        for t in [self.mean_n, self.mean_e, self.mean_d, self.mean_h]:
            t.SetFont(self.mono_font)
        mean_row.Add(self.mean_n, 1)
        mean_row.Add(self.mean_e, 1)
        mean_row.Add(self.mean_d, 1)
        mean_row.Add(self.mean_h, 1)
        stats_sizer.Add(mean_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # RMS errors
        rms_label = wx.StaticText(panel, label="RMS Error:")
        rms_label.SetFont(self.bold_font)
        stats_sizer.Add(rms_label, 0, wx.LEFT | wx.TOP, 3)

        rms_row = wx.BoxSizer(wx.HORIZONTAL)
        self.rms_n = wx.StaticText(panel, label="N:---", size=(55, -1))
        self.rms_e = wx.StaticText(panel, label="E:---", size=(55, -1))
        self.rms_d = wx.StaticText(panel, label="D:---", size=(55, -1))
        self.rms_h = wx.StaticText(panel, label="H:---", size=(55, -1))
        for t in [self.rms_n, self.rms_e, self.rms_d, self.rms_h]:
            t.SetFont(self.mono_font)
        rms_row.Add(self.rms_n, 1)
        rms_row.Add(self.rms_e, 1)
        rms_row.Add(self.rms_d, 1)
        rms_row.Add(self.rms_h, 1)
        stats_sizer.Add(rms_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # Max errors
        max_label = wx.StaticText(panel, label="Max Error:")
        max_label.SetFont(self.bold_font)
        stats_sizer.Add(max_label, 0, wx.LEFT | wx.TOP, 3)

        max_row = wx.BoxSizer(wx.HORIZONTAL)
        self.max_n = wx.StaticText(panel, label="N:---", size=(55, -1))
        self.max_e = wx.StaticText(panel, label="E:---", size=(55, -1))
        self.max_d = wx.StaticText(panel, label="D:---", size=(55, -1))
        self.max_h = wx.StaticText(panel, label="H:---", size=(55, -1))
        for t in [self.max_n, self.max_e, self.max_d, self.max_h]:
            t.SetFont(self.mono_font)
        max_row.Add(self.max_n, 1)
        max_row.Add(self.max_e, 1)
        max_row.Add(self.max_d, 1)
        max_row.Add(self.max_h, 1)
        stats_sizer.Add(max_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 3)

        sizer.Add(stats_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

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
        import wx
        
        # Update status
        if data['testing']:
            self.status_text.SetLabel("TESTING")
            self.status_text.SetForegroundColour(wx.Colour(200, 0, 0))
            self.start_btn.Enable(False)
            self.stop_btn.Enable(True)
        else:
            self.status_text.SetLabel("IDLE")
            self.status_text.SetForegroundColour(wx.Colour(0, 100, 0))
            self.start_btn.Enable(True)
            self.stop_btn.Enable(False)

        self.elapsed_text.SetLabel("%.1fs" % data['elapsed'])
        self.samples_text.SetLabel("%d smp" % data['sample_count'])

        fix_names = {0: "No", 1: "No", 2: "2D", 3: "3D", 4: "DG", 5: "Flt", 6: "Fix"}
        rtk_color = wx.Colour(0, 128, 0) if data['rtk_fix'] >= 5 else wx.Colour(200, 0, 0)
        self.rtk_text.SetLabel("%s %dsat" % (fix_names.get(data['rtk_fix'], '?'), data['rtk_sats']))
        self.rtk_text.SetForegroundColour(rtk_color)

        # Update error display
        if data['err_n_mm'] is not None:
            self.err_n_text.SetLabel("N:%+.0f" % data['err_n_mm'])
            self.err_e_text.SetLabel("E:%+.0f" % data['err_e_mm'])
            self.err_d_text.SetLabel("D:%+.0f" % data['err_d_mm'])
            self.err_horiz_text.SetLabel("H:%.0f" % data['err_horiz_mm'])
            self.err_total_text.SetLabel("T:%.0f" % data['err_total_mm'])

        # Update config display
        params = data.get('params', {})
        if params.get('gps_ctrl', -1) >= 0:
            self.gps_ctrl_spin.SetValue(int(params['gps_ctrl']))
        if params.get('uwbb_ctrl', -1) >= 0:
            self.uwbb_check.SetValue(int(params['uwbb_ctrl']) > 0)
        if params.get('uwbt_ctrl', -1) >= 0:
            self.uwbt_check.SetValue(int(params['uwbt_ctrl']) > 0)
        if params.get('hgt_ref', -1) >= 0:
            self.hgt_choice.SetSelection(int(params['hgt_ref']))

        # Update stats
        stats = data.get('stats', {})
        if stats:
            if 'mean_n' in stats:
                self.mean_n.SetLabel("N:%.0f" % (stats['mean_n'] * 1000))
                self.mean_e.SetLabel("E:%.0f" % (stats['mean_e'] * 1000))
                self.mean_d.SetLabel("D:%.0f" % (stats['mean_d'] * 1000))
                self.mean_h.SetLabel("H:%.0f" % (stats['mean_h'] * 1000))
            if 'rms_n' in stats:
                self.rms_n.SetLabel("N:%.0f" % (stats['rms_n'] * 1000))
                self.rms_e.SetLabel("E:%.0f" % (stats['rms_e'] * 1000))
                self.rms_d.SetLabel("D:%.0f" % (stats['rms_d'] * 1000))
                self.rms_h.SetLabel("H:%.0f" % (stats['rms_h'] * 1000))
            if 'max_n' in stats:
                self.max_n.SetLabel("N:%.0f" % (stats['max_n'] * 1000))
                self.max_e.SetLabel("E:%.0f" % (stats['max_e'] * 1000))
                self.max_d.SetLabel("D:%.0f" % (stats['max_d'] * 1000))
                self.max_h.SetLabel("H:%.0f" % (stats['max_h'] * 1000))

    def _on_start(self, event):
        self.data_queue.put(('start', None))

    def _on_stop(self, event):
        self.data_queue.put(('stop', None))

    def _on_analyze(self, event):
        self.data_queue.put(('analyze', None))

    def _on_save(self, event):
        import wx
        dlg = wx.FileDialog(self, "Save", wildcard="JSON (*.json)|*.json|CSV (*.csv)|*.csv",
                            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)
        if dlg.ShowModal() == wx.ID_OK:
            self.data_queue.put(('save', dlg.GetPath()))
        dlg.Destroy()

    def _on_apply_config(self, event):
        self.data_queue.put(('config', ['gps', str(self.gps_ctrl_spin.GetValue())]))
        self.data_queue.put(('config', ['uwb_beacon', '1' if self.uwbb_check.GetValue() else '0']))
        self.data_queue.put(('config', ['uwb_tag', '1' if self.uwbt_check.GetValue() else '0']))
        self.data_queue.put(('config', ['height', str(self.hgt_choice.GetSelection())]))

    def _on_close(self, event):
        self.timer.Stop()
        self.data_queue.put(('closed', None))
        self.Destroy()
