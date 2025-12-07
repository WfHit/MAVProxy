"""
  UWB Anchor Deploy GUI
  
  wxPython GUI for managing UWB anchor positions
  Optimized for 3.5 inch screens (480x320) with tabbed interface
"""

import wx
import math
import time
import os

from MAVProxy.modules.lib.multiproc import Process, Queue

# Screen size for 3.5 inch display
SMALL_SCREEN_WIDTH = 480
SMALL_SCREEN_HEIGHT = 320


class UWBAnchorGUI:
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

    def update_position(self, lat, lon, alt, n, e, d, avg_progress, backsight_data=None, rtk_data=None):
        """Send position update to GUI"""
        if not self.is_alive():
            return
        try:
            self.cmd_queue.put(('position', {
                'lat': lat, 'lon': lon, 'alt': alt,
                'n': n, 'e': e, 'd': d,
                'avg_progress': avg_progress,
                'origin_set': self.module.origin.is_set(),
                'backsight': backsight_data,
                'rtk': rtk_data
            }))
        except Exception:
            pass

        # Check for commands from GUI
        self._process_gui_commands()

    def update_anchors(self):
        """Send anchor list update to GUI"""
        if not self.is_alive():
            return
        try:
            anchors = [a.to_dict() for a in self.module.anchors]
            origin = self.module.origin.to_dict() if self.module.origin.is_set() else None
            self.cmd_queue.put(('anchors', {'anchors': anchors, 'origin': origin}))
        except Exception:
            pass

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
            if cmd_type == 'set_origin':
                self.module.cmd_set_origin()
            elif cmd_type == 'add_anchor':
                self.module.cmd_add_anchor(data)
            elif cmd_type == 'delete_anchor':
                self.module.cmd_delete_anchor(data)
            elif cmd_type == 'save':
                self.module.cmd_save(data)
            elif cmd_type == 'load':
                self.module.cmd_load(data)
            elif cmd_type == 'clear':
                self.module.cmd_clear()
            elif cmd_type == 'set_backsight':
                # data is (direction, distance, threshold_mm)
                direction, distance, threshold_mm = data
                self.module.cmd_backsight([direction, str(distance), str(threshold_mm)])
            elif cmd_type == 'clear_backsight':
                self.module.cmd_backsight(['clear'])
            elif cmd_type == 'rtk_enable':
                self.module.cmd_rtk(['enable'])
            elif cmd_type == 'rtk_disable':
                self.module.cmd_rtk(['disable'])
            elif cmd_type == 'closed':
                self._alive = False

    def _run_gui(self, cmd_queue, data_queue):
        """Run GUI in separate process"""
        app = wx.App()
        frame = UWBAnchorFrame(cmd_queue, data_queue)
        frame.Show()
        app.MainLoop()


class UWBAnchorFrame(wx.Frame):
    """Main GUI window with tabbed interface for small screens"""

    def __init__(self, cmd_queue, data_queue):
        super(UWBAnchorFrame, self).__init__(
            None, 
            title="UWB Anchor", 
            size=(SMALL_SCREEN_WIDTH, SMALL_SCREEN_HEIGHT)
        )
        self.cmd_queue = cmd_queue
        self.data_queue = data_queue

        self.anchors = []
        self.origin = None

        # Smaller font for compact display
        self.small_font = wx.Font(8, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.mono_font = wx.Font(8, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.bold_font = wx.Font(8, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        self.large_font = wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)

        self._create_ui()

        # Timer for processing commands
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(50)  # 20 Hz

        self.Bind(wx.EVT_CLOSE, self._on_close)

    def _create_ui(self):
        """Create tabbed interface"""
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create notebook (tabs)
        self.notebook = wx.Notebook(panel)

        # Tab 1: Position
        self.pos_panel = wx.Panel(self.notebook)
        self._create_position_tab(self.pos_panel)
        self.notebook.AddPage(self.pos_panel, "Position")

        # Tab 2: Backsight
        self.bs_panel = wx.Panel(self.notebook)
        self._create_backsight_tab(self.bs_panel)
        self.notebook.AddPage(self.bs_panel, "Backsight")

        # Tab 3: RTK/EKF
        self.rtk_panel = wx.Panel(self.notebook)
        self._create_rtk_tab(self.rtk_panel)
        self.notebook.AddPage(self.rtk_panel, "RTK")

        # Tab 4: Anchors
        self.anchor_panel = wx.Panel(self.notebook)
        self._create_anchor_tab(self.anchor_panel)
        self.notebook.AddPage(self.anchor_panel, "Anchors")

        main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 2)
        panel.SetSizer(main_sizer)

    def _create_position_tab(self, panel):
        """Create position display tab"""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # WGS84 Section
        wgs_box = wx.StaticBox(panel, label="WGS84")
        wgs_sizer = wx.StaticBoxSizer(wgs_box, wx.VERTICAL)

        self.lat_text = wx.StaticText(panel, label="Lat: ---.--------°")
        self.lon_text = wx.StaticText(panel, label="Lon: ---.--------°")
        self.alt_text = wx.StaticText(panel, label="Alt: ---.--- m")
        
        self.lat_text.SetFont(self.mono_font)
        self.lon_text.SetFont(self.mono_font)
        self.alt_text.SetFont(self.mono_font)

        wgs_sizer.Add(self.lat_text, 0, wx.ALL, 2)
        wgs_sizer.Add(self.lon_text, 0, wx.ALL, 2)
        wgs_sizer.Add(self.alt_text, 0, wx.ALL, 2)
        sizer.Add(wgs_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # NED Section
        ned_box = wx.StaticBox(panel, label="NED (from Origin)")
        ned_sizer = wx.StaticBoxSizer(ned_box, wx.VERTICAL)

        ned_row = wx.BoxSizer(wx.HORIZONTAL)
        self.n_text = wx.StaticText(panel, label="N: ---", size=(70, -1))
        self.e_text = wx.StaticText(panel, label="E: ---", size=(70, -1))
        self.d_text = wx.StaticText(panel, label="D: ---", size=(70, -1))
        
        self.n_text.SetFont(self.mono_font)
        self.e_text.SetFont(self.mono_font)
        self.d_text.SetFont(self.mono_font)

        ned_row.Add(self.n_text, 1, wx.RIGHT, 5)
        ned_row.Add(self.e_text, 1, wx.RIGHT, 5)
        ned_row.Add(self.d_text, 1)
        ned_sizer.Add(ned_row, 0, wx.EXPAND | wx.ALL, 2)

        # Origin status
        self.origin_text = wx.StaticText(panel, label="Origin: Not set")
        self.origin_text.SetForegroundColour(wx.RED)
        self.origin_text.SetFont(self.bold_font)
        ned_sizer.Add(self.origin_text, 0, wx.ALL, 2)

        sizer.Add(ned_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Progress bar
        self.progress_label = wx.StaticText(panel, label="Recording...")
        self.progress_bar = wx.Gauge(panel, range=100, size=(-1, 15))
        self.progress_label.Hide()
        self.progress_bar.Hide()
        sizer.Add(self.progress_label, 0, wx.LEFT | wx.RIGHT | wx.TOP, 3)
        sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.origin_btn = wx.Button(panel, label="Set Origin", size=(90, 28))
        self.add_btn = wx.Button(panel, label="Add Anchor", size=(90, 28))
        
        self.origin_btn.SetBackgroundColour(wx.Colour(200, 230, 200))
        self.add_btn.SetBackgroundColour(wx.Colour(200, 220, 240))
        
        self.origin_btn.Bind(wx.EVT_BUTTON, self._on_set_origin)
        self.add_btn.Bind(wx.EVT_BUTTON, self._on_add_anchor)

        btn_sizer.Add(self.origin_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(self.add_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_backsight_tab(self, panel):
        """Create backsight reference tab"""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Status
        status_box = wx.StaticBox(panel, label="Backsight Status")
        status_sizer = wx.StaticBoxSizer(status_box, wx.VERTICAL)

        self.bs_status_text = wx.StaticText(panel, label="Not set")
        self.bs_status_text.SetFont(self.bold_font)
        status_sizer.Add(self.bs_status_text, 0, wx.ALL, 2)

        # Target NED
        target_row = wx.BoxSizer(wx.HORIZONTAL)
        self.bs_target_n = wx.StaticText(panel, label="N: ---", size=(65, -1))
        self.bs_target_e = wx.StaticText(panel, label="E: ---", size=(65, -1))
        self.bs_target_d = wx.StaticText(panel, label="D: ---", size=(65, -1))
        self.bs_target_n.SetFont(self.mono_font)
        self.bs_target_e.SetFont(self.mono_font)
        self.bs_target_d.SetFont(self.mono_font)
        target_row.Add(self.bs_target_n, 1)
        target_row.Add(self.bs_target_e, 1)
        target_row.Add(self.bs_target_d, 1)
        status_sizer.Add(target_row, 0, wx.EXPAND | wx.ALL, 2)

        sizer.Add(status_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Error display
        err_box = wx.StaticBox(panel, label="Error (mm)")
        err_sizer = wx.StaticBoxSizer(err_box, wx.VERTICAL)

        err_row = wx.BoxSizer(wx.HORIZONTAL)
        self.bs_err_n = wx.StaticText(panel, label="N: ---", size=(55, -1))
        self.bs_err_e = wx.StaticText(panel, label="E: ---", size=(55, -1))
        self.bs_err_d = wx.StaticText(panel, label="D: ---", size=(55, -1))
        self.bs_err_total = wx.StaticText(panel, label="Tot: ---", size=(65, -1))
        
        self.bs_err_n.SetFont(self.large_font)
        self.bs_err_e.SetFont(self.large_font)
        self.bs_err_d.SetFont(self.large_font)
        self.bs_err_total.SetFont(self.large_font)

        err_row.Add(self.bs_err_n, 1)
        err_row.Add(self.bs_err_e, 1)
        err_row.Add(self.bs_err_d, 1)
        err_row.Add(self.bs_err_total, 1)
        err_sizer.Add(err_row, 0, wx.EXPAND | wx.ALL, 2)

        # Match indicator
        self.bs_match_indicator = wx.StaticText(panel, label="")
        self.bs_match_indicator.SetFont(self.large_font)
        err_sizer.Add(self.bs_match_indicator, 0, wx.ALIGN_CENTER | wx.ALL, 2)

        sizer.Add(err_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.bs_set_btn = wx.Button(panel, label="Set", size=(70, 28))
        self.bs_clear_btn = wx.Button(panel, label="Clear", size=(70, 28))
        
        self.bs_set_btn.Bind(wx.EVT_BUTTON, self._on_set_backsight)
        self.bs_clear_btn.Bind(wx.EVT_BUTTON, self._on_clear_backsight)

        btn_sizer.Add(self.bs_set_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(self.bs_clear_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_rtk_tab(self, panel):
        """Create RTK/EKF status tab"""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # GPS Status
        gps_box = wx.StaticBox(panel, label="GPS Status")
        gps_sizer = wx.StaticBoxSizer(gps_box, wx.VERTICAL)

        fix_row = wx.BoxSizer(wx.HORIZONTAL)
        fix_label = wx.StaticText(panel, label="Fix:")
        fix_label.SetFont(self.bold_font)
        self.gps_fix_text = wx.StaticText(panel, label="---")
        self.gps_fix_text.SetFont(self.large_font)
        fix_row.Add(fix_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        fix_row.Add(self.gps_fix_text, 1, wx.ALIGN_CENTER_VERTICAL)
        gps_sizer.Add(fix_row, 0, wx.EXPAND | wx.ALL, 2)

        info_row = wx.BoxSizer(wx.HORIZONTAL)
        self.gps_sats_text = wx.StaticText(panel, label="Sats: --", size=(60, -1))
        self.gps_hdop_text = wx.StaticText(panel, label="HDOP: --", size=(80, -1))
        self.gps_sats_text.SetFont(self.mono_font)
        self.gps_hdop_text.SetFont(self.mono_font)
        info_row.Add(self.gps_sats_text, 1)
        info_row.Add(self.gps_hdop_text, 1)
        gps_sizer.Add(info_row, 0, wx.EXPAND | wx.ALL, 2)

        sizer.Add(gps_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # EKF Status
        ekf_box = wx.StaticBox(panel, label="EKF Status")
        ekf_sizer = wx.StaticBoxSizer(ekf_box, wx.VERTICAL)

        self.ekf_flags_text = wx.StaticText(panel, label="Flags: 0x0000")
        self.ekf_var_text = wx.StaticText(panel, label="Var: H=-.-- V=-.--")
        self.ekf_flags_text.SetFont(self.mono_font)
        self.ekf_var_text.SetFont(self.mono_font)
        ekf_sizer.Add(self.ekf_flags_text, 0, wx.ALL, 2)
        ekf_sizer.Add(self.ekf_var_text, 0, wx.ALL, 2)

        sizer.Add(ekf_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # RTK Control
        rtk_box = wx.StaticBox(panel, label="RTK Control")
        rtk_sizer = wx.StaticBoxSizer(rtk_box, wx.VERTICAL)

        self.rtk_status_text = wx.StaticText(panel, label="RTK: Disabled")
        self.rtk_status_text.SetFont(self.large_font)
        self.rtk_status_text.SetForegroundColour(wx.Colour(180, 0, 0))
        rtk_sizer.Add(self.rtk_status_text, 0, wx.ALL, 2)

        btn_row = wx.BoxSizer(wx.HORIZONTAL)
        self.rtk_enable_btn = wx.Button(panel, label="Enable", size=(70, 28))
        self.rtk_disable_btn = wx.Button(panel, label="Disable", size=(70, 28))
        self.rtk_enable_btn.SetBackgroundColour(wx.Colour(200, 230, 200))
        self.rtk_disable_btn.SetBackgroundColour(wx.Colour(240, 200, 200))
        self.rtk_enable_btn.Bind(wx.EVT_BUTTON, self._on_rtk_enable)
        self.rtk_disable_btn.Bind(wx.EVT_BUTTON, self._on_rtk_disable)
        btn_row.Add(self.rtk_enable_btn, 1, wx.RIGHT, 5)
        btn_row.Add(self.rtk_disable_btn, 1)
        rtk_sizer.Add(btn_row, 0, wx.EXPAND | wx.ALL, 2)

        sizer.Add(rtk_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_anchor_tab(self, panel):
        """Create anchor list tab"""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Anchor list (simplified for small screen)
        self.anchor_list = wx.ListCtrl(
            panel, 
            style=wx.LC_REPORT | wx.LC_SINGLE_SEL
        )
        self.anchor_list.SetFont(self.mono_font)
        
        # Columns optimized for small screen
        self.anchor_list.InsertColumn(0, "ID", width=30)
        self.anchor_list.InsertColumn(1, "Name", width=50)
        self.anchor_list.InsertColumn(2, "N", width=65)
        self.anchor_list.InsertColumn(3, "E", width=65)
        self.anchor_list.InsertColumn(4, "D", width=65)

        sizer.Add(self.anchor_list, 1, wx.EXPAND | wx.ALL, 3)

        # Buttons row 1
        btn_row1 = wx.BoxSizer(wx.HORIZONTAL)
        self.delete_btn = wx.Button(panel, label="Delete", size=(70, 26))
        self.clear_btn = wx.Button(panel, label="Clear All", size=(70, 26))
        
        self.delete_btn.SetBackgroundColour(wx.Colour(240, 200, 200))
        self.clear_btn.SetBackgroundColour(wx.Colour(240, 200, 200))
        
        self.delete_btn.Bind(wx.EVT_BUTTON, self._on_delete_anchor)
        self.clear_btn.Bind(wx.EVT_BUTTON, self._on_clear)

        btn_row1.Add(self.delete_btn, 1, wx.RIGHT, 3)
        btn_row1.Add(self.clear_btn, 1)
        sizer.Add(btn_row1, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 3)

        # Buttons row 2
        btn_row2 = wx.BoxSizer(wx.HORIZONTAL)
        self.save_btn = wx.Button(panel, label="Save", size=(70, 26))
        self.load_btn = wx.Button(panel, label="Load", size=(70, 26))
        
        self.save_btn.Bind(wx.EVT_BUTTON, self._on_save)
        self.load_btn.Bind(wx.EVT_BUTTON, self._on_load)

        btn_row2.Add(self.save_btn, 1, wx.RIGHT, 3)
        btn_row2.Add(self.load_btn, 1)
        sizer.Add(btn_row2, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 3)

        # Anchor count label
        self.anchor_count_label = wx.StaticText(panel, label="0 anchors")
        self.anchor_count_label.SetFont(self.small_font)
        sizer.Add(self.anchor_count_label, 0, wx.LEFT | wx.BOTTOM, 3)

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
            elif cmd_type == 'position':
                self._update_position(data)
            elif cmd_type == 'anchors':
                self._update_anchors(data)

    def _update_position(self, data):
        """Update position display"""
        # Position tab updates
        if data['lat'] is not None:
            self.lat_text.SetLabel("Lat: %.8f°" % data['lat'])
            self.lon_text.SetLabel("Lon: %.8f°" % data['lon'])
            self.alt_text.SetLabel("Alt: %.2f m" % data['alt'])
        else:
            self.lat_text.SetLabel("Lat: ---")
            self.lon_text.SetLabel("Lon: ---")
            self.alt_text.SetLabel("Alt: ---")

        if data['n'] is not None:
            self.n_text.SetLabel("N:%+.2f" % data['n'])
            self.e_text.SetLabel("E:%+.2f" % data['e'])
            self.d_text.SetLabel("D:%+.2f" % data['d'])
        else:
            self.n_text.SetLabel("N: ---")
            self.e_text.SetLabel("E: ---")
            self.d_text.SetLabel("D: ---")

        if data['origin_set']:
            self.origin_text.SetLabel("Origin: Set ✓")
            self.origin_text.SetForegroundColour(wx.Colour(0, 128, 0))
        else:
            self.origin_text.SetLabel("Origin: Not set")
            self.origin_text.SetForegroundColour(wx.RED)

        # Update progress bar
        if data['avg_progress'] is not None:
            self.progress_bar.Show()
            self.progress_label.Show()
            self.progress_bar.SetValue(int(data['avg_progress'] * 100))
            self.progress_label.SetLabel("Recording %d%%" % int(data['avg_progress'] * 100))
            self.add_btn.Disable()
        else:
            self.progress_bar.Hide()
            self.progress_label.Hide()
            self.add_btn.Enable()

        # Backsight tab updates
        bs = data.get('backsight')
        if bs is not None:
            self.bs_status_text.SetLabel("%s %.2fm thr:%dmm" % (
                bs['direction'], bs['distance'], bs['threshold_mm']))
            self.bs_status_text.SetForegroundColour(wx.Colour(0, 100, 150))

            self.bs_target_n.SetLabel("N:%+.2f" % bs['bs_n'])
            self.bs_target_e.SetLabel("E:%+.2f" % bs['bs_e'])
            self.bs_target_d.SetLabel("D:%+.2f" % bs['bs_d'])

            if bs['err_n_mm'] is not None:
                self.bs_err_n.SetLabel("N:%+.0f" % bs['err_n_mm'])
                self.bs_err_e.SetLabel("E:%+.0f" % bs['err_e_mm'])
                self.bs_err_d.SetLabel("D:%+.0f" % bs['err_d_mm'])
                self.bs_err_total.SetLabel("Tot:%.0f" % bs['err_total_mm'])

                if bs['is_match']:
                    match_color = wx.Colour(0, 180, 0)
                    self.bs_err_n.SetForegroundColour(match_color)
                    self.bs_err_e.SetForegroundColour(match_color)
                    self.bs_err_d.SetForegroundColour(match_color)
                    self.bs_err_total.SetForegroundColour(match_color)
                    self.bs_match_indicator.SetLabel("✓ MATCH!")
                    self.bs_match_indicator.SetForegroundColour(match_color)
                else:
                    normal_color = wx.Colour(180, 0, 0)
                    self.bs_err_n.SetForegroundColour(normal_color)
                    self.bs_err_e.SetForegroundColour(normal_color)
                    self.bs_err_d.SetForegroundColour(normal_color)
                    self.bs_err_total.SetForegroundColour(normal_color)
                    self.bs_match_indicator.SetLabel("")
            else:
                self.bs_err_n.SetLabel("N:---")
                self.bs_err_e.SetLabel("E:---")
                self.bs_err_d.SetLabel("D:---")
                self.bs_err_total.SetLabel("Tot:---")
                self.bs_match_indicator.SetLabel("")
        else:
            self.bs_status_text.SetLabel("Not set")
            self.bs_status_text.SetForegroundColour(wx.Colour(128, 128, 128))
            self.bs_target_n.SetLabel("N: ---")
            self.bs_target_e.SetLabel("E: ---")
            self.bs_target_d.SetLabel("D: ---")
            self.bs_err_n.SetLabel("N:---")
            self.bs_err_e.SetLabel("E:---")
            self.bs_err_d.SetLabel("D:---")
            self.bs_err_total.SetLabel("Tot:---")
            self.bs_err_n.SetForegroundColour(wx.BLACK)
            self.bs_err_e.SetForegroundColour(wx.BLACK)
            self.bs_err_d.SetForegroundColour(wx.BLACK)
            self.bs_err_total.SetForegroundColour(wx.BLACK)
            self.bs_match_indicator.SetLabel("")

        # RTK tab updates
        rtk = data.get('rtk')
        if rtk is not None:
            fix_types = {
                0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D",
                4: "DGPS", 5: "Float", 6: "Fixed"
            }
            fix_name = fix_types.get(rtk['fix_type'], "?")
            self.gps_fix_text.SetLabel(fix_name)

            if rtk['fix_type'] == 6:
                self.gps_fix_text.SetForegroundColour(wx.Colour(0, 150, 0))
            elif rtk['fix_type'] == 5:
                self.gps_fix_text.SetForegroundColour(wx.Colour(200, 150, 0))
            elif rtk['fix_type'] >= 3:
                self.gps_fix_text.SetForegroundColour(wx.Colour(0, 100, 200))
            else:
                self.gps_fix_text.SetForegroundColour(wx.Colour(180, 0, 0))

            self.gps_sats_text.SetLabel("Sats:%d" % rtk['satellites'])
            self.gps_hdop_text.SetLabel("HDOP:%.1f" % rtk['hdop'])
            self.ekf_flags_text.SetLabel("Flags:0x%04X" % rtk['ekf_flags'])
            self.ekf_var_text.SetLabel("H=%.2f V=%.2f" % (
                rtk['pos_horiz_var'], rtk['pos_vert_var']))

            if rtk['rtk_enabled']:
                self.rtk_status_text.SetLabel("RTK: ON ✓")
                self.rtk_status_text.SetForegroundColour(wx.Colour(0, 150, 0))
            else:
                self.rtk_status_text.SetLabel("RTK: OFF")
                self.rtk_status_text.SetForegroundColour(wx.Colour(180, 0, 0))

        # Refresh layouts
        self.pos_panel.Layout()
        self.bs_panel.Layout()
        self.rtk_panel.Layout()

    def _update_anchors(self, data):
        """Update anchor list"""
        self.anchors = data['anchors']
        self.origin = data['origin']

        # Clear list
        self.anchor_list.DeleteAllItems()

        if len(self.anchors) == 0:
            self.anchor_count_label.SetLabel("0 anchors")
            return

        for i, a in enumerate(self.anchors):
            idx = self.anchor_list.InsertItem(i, str(a['id']))
            self.anchor_list.SetItem(idx, 1, a['name'][:6])  # Truncate name
            self.anchor_list.SetItem(idx, 2, "%+.2f" % a['n'])
            self.anchor_list.SetItem(idx, 3, "%+.2f" % a['e'])
            self.anchor_list.SetItem(idx, 4, "%+.2f" % a['d'])

            # Alternate row colors
            if i % 2 == 1:
                self.anchor_list.SetItemBackgroundColour(idx, wx.Colour(245, 245, 250))

        self.anchor_count_label.SetLabel("%d anchors" % len(self.anchors))

    def _on_set_origin(self, event):
        """Set origin button clicked"""
        self.data_queue.put(('set_origin', None))

    def _on_add_anchor(self, event):
        """Add anchor button clicked"""
        dlg = wx.TextEntryDialog(
            self,
            "Anchor name:",
            "Add Anchor",
            ""
        )
        if dlg.ShowModal() == wx.ID_OK:
            name = dlg.GetValue().strip() or None
            self.data_queue.put(('add_anchor', name))
        dlg.Destroy()

    def _on_delete_anchor(self, event):
        """Delete selected anchor"""
        idx = self.anchor_list.GetFirstSelected()
        if idx == -1:
            wx.MessageBox("Select an anchor", "No Selection", wx.OK | wx.ICON_WARNING)
            return

        anchor_id = self.anchor_list.GetItemText(idx, 0)
        dlg = wx.MessageDialog(
            self, 
            "Delete anchor %s?" % anchor_id, 
            "Confirm", 
            wx.YES_NO | wx.NO_DEFAULT
        )
        if dlg.ShowModal() == wx.ID_YES:
            self.data_queue.put(('delete_anchor', anchor_id))
        dlg.Destroy()

    def _on_clear(self, event):
        """Clear all anchors"""
        if len(self.anchors) == 0:
            return

        dlg = wx.MessageDialog(
            self,
            "Delete ALL %d anchors?" % len(self.anchors),
            "Confirm",
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING
        )
        if dlg.ShowModal() == wx.ID_YES:
            self.data_queue.put(('clear', None))
        dlg.Destroy()

    def _on_save(self, event):
        """Save anchors to file"""
        if len(self.anchors) == 0:
            wx.MessageBox("No anchors", "Info", wx.OK)
            return

        dlg = wx.FileDialog(
            self,
            "Save Anchors",
            wildcard="JSON (*.json)|*.json",
            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT
        )
        dlg.SetFilename("uwb_anchors.json")
        if dlg.ShowModal() == wx.ID_OK:
            self.data_queue.put(('save', dlg.GetPath()))
        dlg.Destroy()

    def _on_load(self, event):
        """Load anchors from file"""
        dlg = wx.FileDialog(
            self,
            "Load Anchors",
            wildcard="JSON (*.json)|*.json",
            style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST
        )
        if dlg.ShowModal() == wx.ID_OK:
            self.data_queue.put(('load', dlg.GetPath()))
        dlg.Destroy()

    def _on_set_backsight(self, event):
        """Open backsight dialog"""
        dlg = BacksightDialog(self)
        if dlg.ShowModal() == wx.ID_OK:
            direction, distance, threshold = dlg.get_values()
            self.data_queue.put(('set_backsight', (direction, distance, threshold)))
        dlg.Destroy()

    def _on_clear_backsight(self, event):
        """Clear backsight"""
        self.data_queue.put(('clear_backsight', None))

    def _on_rtk_enable(self, event):
        """Enable RTK"""
        self.data_queue.put(('rtk_enable', None))

    def _on_rtk_disable(self, event):
        """Disable RTK"""
        self.data_queue.put(('rtk_disable', None))

    def _on_close(self, event):
        """Window closing"""
        self.timer.Stop()
        self.data_queue.put(('closed', None))
        self.Destroy()


class BacksightDialog(wx.Dialog):
    """Dialog for setting backsight reference point - compact for small screen"""

    def __init__(self, parent):
        super(BacksightDialog, self).__init__(
            parent,
            title="Set Backsight",
            size=(280, 200),
            style=wx.DEFAULT_DIALOG_STYLE
        )

        self._create_ui()
        self.Centre()

    def _create_ui(self):
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        small_font = wx.Font(8, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

        # Direction
        dir_sizer = wx.BoxSizer(wx.HORIZONTAL)
        dir_label = wx.StaticText(panel, label="Direction:")
        dir_label.SetFont(small_font)
        dir_sizer.Add(dir_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.dir_choice = wx.Choice(panel, choices=['N', 'E', 'S', 'W'], size=(60, -1))
        self.dir_choice.SetSelection(0)
        dir_sizer.Add(self.dir_choice, 0)
        main_sizer.Add(dir_sizer, 0, wx.ALL, 8)

        # Distance
        dist_sizer = wx.BoxSizer(wx.HORIZONTAL)
        dist_label = wx.StaticText(panel, label="Distance (m):")
        dist_label.SetFont(small_font)
        dist_sizer.Add(dist_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.dist_ctrl = wx.TextCtrl(panel, value="10.0", size=(70, -1))
        dist_sizer.Add(self.dist_ctrl, 0)
        main_sizer.Add(dist_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Threshold
        thresh_sizer = wx.BoxSizer(wx.HORIZONTAL)
        thresh_label = wx.StaticText(panel, label="Threshold (mm):")
        thresh_label.SetFont(small_font)
        thresh_sizer.Add(thresh_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.thresh_ctrl = wx.TextCtrl(panel, value="50", size=(60, -1))
        thresh_sizer.Add(self.thresh_ctrl, 0)
        main_sizer.Add(thresh_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # Buttons
        btn_sizer = wx.StdDialogButtonSizer()
        ok_btn = wx.Button(panel, wx.ID_OK, size=(60, 26))
        ok_btn.SetDefault()
        cancel_btn = wx.Button(panel, wx.ID_CANCEL, size=(60, 26))
        btn_sizer.AddButton(ok_btn)
        btn_sizer.AddButton(cancel_btn)
        btn_sizer.Realize()
        main_sizer.Add(btn_sizer, 0, wx.ALIGN_CENTER | wx.ALL, 8)

        panel.SetSizer(main_sizer)

    def get_values(self):
        """Get dialog values"""
        directions = ['N', 'E', 'S', 'W']
        direction = directions[self.dir_choice.GetSelection()]

        try:
            distance = float(self.dist_ctrl.GetValue())
        except ValueError:
            distance = 10.0

        try:
            threshold = int(self.thresh_ctrl.GetValue())
        except ValueError:
            threshold = 50

        return direction, distance, threshold
