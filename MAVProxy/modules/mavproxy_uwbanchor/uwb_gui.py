"""
  UWB Anchor Deploy GUI
  
  wxPython GUI for managing UWB anchor positions
"""

import wx
import wx.grid
import math
import time
import os

from MAVProxy.modules.lib.multiproc import Process, Queue


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
    """Main GUI window"""

    def __init__(self, cmd_queue, data_queue):
        super(UWBAnchorFrame, self).__init__(None, title="UWB Anchor Deploy", size=(950, 950))
        self.cmd_queue = cmd_queue
        self.data_queue = data_queue

        self.anchors = []
        self.origin = None

        self._create_ui()
        self._create_menu()

        # Timer for processing commands
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(50)  # 20 Hz

        self.Bind(wx.EVT_CLOSE, self._on_close)

        # Center on screen
        self.Centre()

    def _create_menu(self):
        menubar = wx.MenuBar()

        file_menu = wx.Menu()
        save_item = file_menu.Append(wx.ID_SAVE, "&Save\tCtrl+S", "Save anchors")
        load_item = file_menu.Append(wx.ID_OPEN, "&Load\tCtrl+O", "Load anchors")
        file_menu.AppendSeparator()
        exit_item = file_menu.Append(wx.ID_EXIT, "E&xit\tCtrl+Q", "Exit")

        self.Bind(wx.EVT_MENU, self._on_save, save_item)
        self.Bind(wx.EVT_MENU, self._on_load, load_item)
        self.Bind(wx.EVT_MENU, self._on_close, exit_item)

        menubar.Append(file_menu, "&File")
        self.SetMenuBar(menubar)

    def _create_ui(self):
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # === Current Position Section ===
        pos_box = wx.StaticBox(panel, label="Current Position (Real-time)")
        pos_sizer = wx.StaticBoxSizer(pos_box, wx.VERTICAL)

        # WGS84 row
        wgs_sizer = wx.BoxSizer(wx.HORIZONTAL)
        wgs_label = wx.StaticText(panel, label="WGS84:")
        wgs_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        wgs_sizer.Add(wgs_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.lat_text = wx.StaticText(panel, label="Lat: ---.--------°", size=(180, -1))
        self.lon_text = wx.StaticText(panel, label="Lon: ---.--------°", size=(190, -1))
        self.alt_text = wx.StaticText(panel, label="Alt: ---.--- m", size=(130, -1))

        self.lat_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.lon_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.alt_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))

        wgs_sizer.Add(self.lat_text, 0, wx.RIGHT, 15)
        wgs_sizer.Add(self.lon_text, 0, wx.RIGHT, 15)
        wgs_sizer.Add(self.alt_text, 0)
        pos_sizer.Add(wgs_sizer, 0, wx.ALL, 5)

        # NED row
        ned_sizer = wx.BoxSizer(wx.HORIZONTAL)
        ned_label = wx.StaticText(panel, label="NED:    ")
        ned_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        ned_sizer.Add(ned_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.n_text = wx.StaticText(panel, label="N: -------.--- m", size=(140, -1))
        self.e_text = wx.StaticText(panel, label="E: -------.--- m", size=(140, -1))
        self.d_text = wx.StaticText(panel, label="D: -------.--- m", size=(140, -1))

        self.n_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.e_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.d_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))

        ned_sizer.Add(self.n_text, 0, wx.RIGHT, 15)
        ned_sizer.Add(self.e_text, 0, wx.RIGHT, 15)
        ned_sizer.Add(self.d_text, 0)
        pos_sizer.Add(ned_sizer, 0, wx.ALL, 5)

        # Origin info
        origin_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.origin_text = wx.StaticText(panel, label="Origin: Not set")
        self.origin_text.SetForegroundColour(wx.RED)
        self.origin_text.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        origin_sizer.Add(self.origin_text, 0, wx.ALIGN_CENTER_VERTICAL)
        pos_sizer.Add(origin_sizer, 0, wx.ALL, 5)

        # === Backsight Section ===
        backsight_box = wx.StaticBox(panel, label="Backsight Reference Point")
        backsight_sizer = wx.StaticBoxSizer(backsight_box, wx.VERTICAL)

        # Backsight info row
        bs_info_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.bs_status_text = wx.StaticText(panel, label="Backsight: Not set")
        self.bs_status_text.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        bs_info_sizer.Add(self.bs_status_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 20)

        self.bs_set_btn = wx.Button(panel, label="Set Backsight", size=(100, 25))
        self.bs_clear_btn = wx.Button(panel, label="Clear", size=(60, 25))
        self.bs_set_btn.Bind(wx.EVT_BUTTON, self._on_set_backsight)
        self.bs_clear_btn.Bind(wx.EVT_BUTTON, self._on_clear_backsight)
        bs_info_sizer.Add(self.bs_set_btn, 0, wx.RIGHT, 5)
        bs_info_sizer.Add(self.bs_clear_btn, 0)
        backsight_sizer.Add(bs_info_sizer, 0, wx.ALL, 5)

        # Backsight NED target row
        bs_target_sizer = wx.BoxSizer(wx.HORIZONTAL)
        target_label = wx.StaticText(panel, label="Target NED: ")
        target_label.SetFont(wx.Font(9, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        bs_target_sizer.Add(target_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)

        self.bs_target_n = wx.StaticText(panel, label="N: --- m", size=(100, -1))
        self.bs_target_e = wx.StaticText(panel, label="E: --- m", size=(100, -1))
        self.bs_target_d = wx.StaticText(panel, label="D: --- m", size=(100, -1))
        self.bs_target_n.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.bs_target_e.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.bs_target_d.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        bs_target_sizer.Add(self.bs_target_n, 0, wx.RIGHT, 10)
        bs_target_sizer.Add(self.bs_target_e, 0, wx.RIGHT, 10)
        bs_target_sizer.Add(self.bs_target_d, 0)
        backsight_sizer.Add(bs_target_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Error display row (large, prominent)
        error_sizer = wx.BoxSizer(wx.HORIZONTAL)
        error_label = wx.StaticText(panel, label="Error (mm): ")
        error_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        error_sizer.Add(error_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)

        self.bs_err_n = wx.StaticText(panel, label="N: ----", size=(90, -1))
        self.bs_err_e = wx.StaticText(panel, label="E: ----", size=(90, -1))
        self.bs_err_d = wx.StaticText(panel, label="D: ----", size=(90, -1))
        self.bs_err_total = wx.StaticText(panel, label="Total: ----", size=(110, -1))

        err_font = wx.Font(11, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        self.bs_err_n.SetFont(err_font)
        self.bs_err_e.SetFont(err_font)
        self.bs_err_d.SetFont(err_font)
        self.bs_err_total.SetFont(err_font)

        error_sizer.Add(self.bs_err_n, 0, wx.RIGHT, 10)
        error_sizer.Add(self.bs_err_e, 0, wx.RIGHT, 10)
        error_sizer.Add(self.bs_err_d, 0, wx.RIGHT, 15)
        error_sizer.Add(self.bs_err_total, 0)
        backsight_sizer.Add(error_sizer, 0, wx.ALL, 5)

        # Match status indicator
        match_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.bs_match_indicator = wx.StaticText(panel, label="")
        self.bs_match_indicator.SetFont(wx.Font(14, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        match_sizer.Add(self.bs_match_indicator, 0, wx.ALIGN_CENTER_VERTICAL)
        backsight_sizer.Add(match_sizer, 0, wx.ALL, 5)

        main_sizer.Add(backsight_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # === RTK / EKF Status Section ===
        rtk_box = wx.StaticBox(panel, label="RTK / EKF Status")
        rtk_sizer = wx.StaticBoxSizer(rtk_box, wx.VERTICAL)

        # GPS Fix row
        gps_row = wx.BoxSizer(wx.HORIZONTAL)
        gps_label = wx.StaticText(panel, label="GPS Fix:")
        gps_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        gps_row.Add(gps_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.gps_fix_text = wx.StaticText(panel, label="---", size=(100, -1))
        self.gps_fix_text.SetFont(wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        gps_row.Add(self.gps_fix_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 20)

        self.gps_sats_text = wx.StaticText(panel, label="Sats: --", size=(70, -1))
        self.gps_hdop_text = wx.StaticText(panel, label="HDOP: --", size=(90, -1))
        self.gps_sats_text.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        self.gps_hdop_text.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        gps_row.Add(self.gps_sats_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)
        gps_row.Add(self.gps_hdop_text, 0, wx.ALIGN_CENTER_VERTICAL)
        rtk_sizer.Add(gps_row, 0, wx.ALL, 5)

        # EKF Status row
        ekf_row = wx.BoxSizer(wx.HORIZONTAL)
        ekf_label = wx.StaticText(panel, label="EKF:")
        ekf_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        ekf_row.Add(ekf_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.ekf_flags_text = wx.StaticText(panel, label="Flags: 0x0000", size=(120, -1))
        self.ekf_flags_text.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        ekf_row.Add(self.ekf_flags_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 15)

        self.ekf_var_text = wx.StaticText(panel, label="Var: H=-.-- V=-.--", size=(160, -1))
        self.ekf_var_text.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        ekf_row.Add(self.ekf_var_text, 0, wx.ALIGN_CENTER_VERTICAL)
        rtk_sizer.Add(ekf_row, 0, wx.ALL, 5)

        # RTK Enable row
        rtk_ctrl_row = wx.BoxSizer(wx.HORIZONTAL)
        self.rtk_status_text = wx.StaticText(panel, label="RTK: Disabled")
        self.rtk_status_text.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        self.rtk_status_text.SetForegroundColour(wx.Colour(180, 0, 0))
        rtk_ctrl_row.Add(self.rtk_status_text, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 20)

        self.rtk_enable_btn = wx.Button(panel, label="Enable RTK", size=(100, 28))
        self.rtk_disable_btn = wx.Button(panel, label="Disable RTK", size=(100, 28))
        self.rtk_enable_btn.SetBackgroundColour(wx.Colour(200, 230, 200))
        self.rtk_disable_btn.SetBackgroundColour(wx.Colour(240, 200, 200))
        self.rtk_enable_btn.Bind(wx.EVT_BUTTON, self._on_rtk_enable)
        self.rtk_disable_btn.Bind(wx.EVT_BUTTON, self._on_rtk_disable)
        rtk_ctrl_row.Add(self.rtk_enable_btn, 0, wx.RIGHT, 5)
        rtk_ctrl_row.Add(self.rtk_disable_btn, 0)
        rtk_sizer.Add(rtk_ctrl_row, 0, wx.ALL, 5)

        main_sizer.Add(rtk_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # Progress bar for averaging
        progress_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.progress_label = wx.StaticText(panel, label="Recording anchor...")
        self.progress_bar = wx.Gauge(panel, range=100, size=(300, 25))
        self.progress_label.Hide()
        self.progress_bar.Hide()

        progress_sizer.Add(self.progress_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)
        progress_sizer.Add(self.progress_bar, 0, wx.ALIGN_CENTER_VERTICAL)
        pos_sizer.Add(progress_sizer, 0, wx.ALL, 5)

        main_sizer.Add(pos_sizer, 0, wx.EXPAND | wx.ALL, 10)

        # === Buttons Section ===
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.origin_btn = wx.Button(panel, label="Set Origin", size=(100, 30))
        self.add_btn = wx.Button(panel, label="Add Anchor", size=(100, 30))
        self.delete_btn = wx.Button(panel, label="Delete Selected", size=(110, 30))
        self.clear_btn = wx.Button(panel, label="Clear All", size=(90, 30))
        self.save_btn = wx.Button(panel, label="Save", size=(70, 30))
        self.load_btn = wx.Button(panel, label="Load", size=(70, 30))

        self.origin_btn.SetBackgroundColour(wx.Colour(200, 230, 200))
        self.add_btn.SetBackgroundColour(wx.Colour(200, 220, 240))
        self.delete_btn.SetBackgroundColour(wx.Colour(240, 200, 200))
        self.clear_btn.SetBackgroundColour(wx.Colour(240, 200, 200))

        self.origin_btn.Bind(wx.EVT_BUTTON, self._on_set_origin)
        self.add_btn.Bind(wx.EVT_BUTTON, self._on_add_anchor)
        self.delete_btn.Bind(wx.EVT_BUTTON, self._on_delete_anchor)
        self.clear_btn.Bind(wx.EVT_BUTTON, self._on_clear)
        self.save_btn.Bind(wx.EVT_BUTTON, self._on_save)
        self.load_btn.Bind(wx.EVT_BUTTON, self._on_load)

        btn_sizer.Add(self.origin_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(self.add_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(self.delete_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(self.clear_btn, 0, wx.RIGHT, 20)
        btn_sizer.Add(self.save_btn, 0, wx.RIGHT, 5)
        btn_sizer.Add(self.load_btn, 0)

        main_sizer.Add(btn_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # === Anchor Table ===
        table_box = wx.StaticBox(panel, label="Anchors")
        table_sizer = wx.StaticBoxSizer(table_box, wx.VERTICAL)

        self.grid = wx.grid.Grid(panel)
        self.grid.CreateGrid(0, 10)
        self.grid.SetColLabelValue(0, "ID")
        self.grid.SetColLabelValue(1, "Name")
        self.grid.SetColLabelValue(2, "Latitude (°)")
        self.grid.SetColLabelValue(3, "Longitude (°)")
        self.grid.SetColLabelValue(4, "Alt (m)")
        self.grid.SetColLabelValue(5, "N (m)")
        self.grid.SetColLabelValue(6, "E (m)")
        self.grid.SetColLabelValue(7, "D (m)")
        self.grid.SetColLabelValue(8, "Dist (m)")
        self.grid.SetColLabelValue(9, "Samples")

        self.grid.SetColSize(0, 40)
        self.grid.SetColSize(1, 70)
        self.grid.SetColSize(2, 130)
        self.grid.SetColSize(3, 140)
        self.grid.SetColSize(4, 75)
        self.grid.SetColSize(5, 85)
        self.grid.SetColSize(6, 85)
        self.grid.SetColSize(7, 85)
        self.grid.SetColSize(8, 80)
        self.grid.SetColSize(9, 60)

        self.grid.EnableEditing(False)
        self.grid.SetSelectionMode(wx.grid.Grid.SelectRows)

        # Set grid font
        grid_font = wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.grid.SetDefaultCellFont(grid_font)

        table_sizer.Add(self.grid, 1, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(table_sizer, 1, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # === Status Bar ===
        self.CreateStatusBar()
        self.SetStatusText("Ready - Use 'Set Origin' to establish NED reference point")

        panel.SetSizer(main_sizer)

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
        if data['lat'] is not None:
            self.lat_text.SetLabel("Lat: %.9f°" % data['lat'])
            self.lon_text.SetLabel("Lon: %.9f°" % data['lon'])
            self.alt_text.SetLabel("Alt: %.3f m" % data['alt'])
        else:
            self.lat_text.SetLabel("Lat: ---")
            self.lon_text.SetLabel("Lon: ---")
            self.alt_text.SetLabel("Alt: ---")

        if data['n'] is not None:
            self.n_text.SetLabel("N: %+.3f m" % data['n'])
            self.e_text.SetLabel("E: %+.3f m" % data['e'])
            self.d_text.SetLabel("D: %+.3f m" % data['d'])
        else:
            self.n_text.SetLabel("N: --- m")
            self.e_text.SetLabel("E: --- m")
            self.d_text.SetLabel("D: --- m")

        if data['origin_set']:
            self.origin_text.SetLabel("Origin: Set ✓")
            self.origin_text.SetForegroundColour(wx.Colour(0, 128, 0))
        else:
            self.origin_text.SetLabel("Origin: Not set")
            self.origin_text.SetForegroundColour(wx.RED)

        # Update backsight display
        bs = data.get('backsight')
        if bs is not None:
            self.bs_status_text.SetLabel("Backsight: %s %.3f m (threshold: %d mm)" % (
                bs['direction'], bs['distance'], bs['threshold_mm']))
            self.bs_status_text.SetForegroundColour(wx.Colour(0, 100, 150))

            # Target NED
            self.bs_target_n.SetLabel("N: %+.3f m" % bs['bs_n'])
            self.bs_target_e.SetLabel("E: %+.3f m" % bs['bs_e'])
            self.bs_target_d.SetLabel("D: %+.3f m" % bs['bs_d'])

            # Error display
            if bs['err_n_mm'] is not None:
                self.bs_err_n.SetLabel("N: %+.0f" % bs['err_n_mm'])
                self.bs_err_e.SetLabel("E: %+.0f" % bs['err_e_mm'])
                self.bs_err_d.SetLabel("D: %+.0f" % bs['err_d_mm'])
                self.bs_err_total.SetLabel("Total: %.0f" % bs['err_total_mm'])

                # Highlight when position matches
                if bs['is_match']:
                    match_color = wx.Colour(0, 180, 0)  # Green
                    self.bs_err_n.SetForegroundColour(match_color)
                    self.bs_err_e.SetForegroundColour(match_color)
                    self.bs_err_d.SetForegroundColour(match_color)
                    self.bs_err_total.SetForegroundColour(match_color)
                    self.bs_match_indicator.SetLabel("✓ POSITION MATCH!")
                    self.bs_match_indicator.SetForegroundColour(match_color)
                    self.bs_match_indicator.SetBackgroundColour(wx.Colour(200, 255, 200))
                else:
                    normal_color = wx.Colour(180, 0, 0)  # Red
                    self.bs_err_n.SetForegroundColour(normal_color)
                    self.bs_err_e.SetForegroundColour(normal_color)
                    self.bs_err_d.SetForegroundColour(normal_color)
                    self.bs_err_total.SetForegroundColour(normal_color)
                    self.bs_match_indicator.SetLabel("")
                    self.bs_match_indicator.SetBackgroundColour(self.GetBackgroundColour())
            else:
                self.bs_err_n.SetLabel("N: ---")
                self.bs_err_e.SetLabel("E: ---")
                self.bs_err_d.SetLabel("D: ---")
                self.bs_err_total.SetLabel("Total: ---")
                self.bs_match_indicator.SetLabel("")
        else:
            self.bs_status_text.SetLabel("Backsight: Not set")
            self.bs_status_text.SetForegroundColour(wx.Colour(128, 128, 128))
            self.bs_target_n.SetLabel("N: --- m")
            self.bs_target_e.SetLabel("E: --- m")
            self.bs_target_d.SetLabel("D: --- m")
            self.bs_err_n.SetLabel("N: ----")
            self.bs_err_e.SetLabel("E: ----")
            self.bs_err_d.SetLabel("D: ----")
            self.bs_err_total.SetLabel("Total: ----")
            self.bs_err_n.SetForegroundColour(wx.BLACK)
            self.bs_err_e.SetForegroundColour(wx.BLACK)
            self.bs_err_d.SetForegroundColour(wx.BLACK)
            self.bs_err_total.SetForegroundColour(wx.BLACK)
            self.bs_match_indicator.SetLabel("")

        # Update progress bar
        if data['avg_progress'] is not None:
            self.progress_bar.Show()
            self.progress_label.Show()
            self.progress_bar.SetValue(int(data['avg_progress'] * 100))
            self.progress_label.SetLabel("Recording anchor... %d%%" % int(data['avg_progress'] * 100))
            self.add_btn.Disable()
        else:
            self.progress_bar.Hide()
            self.progress_label.Hide()
            self.add_btn.Enable()

        # Update RTK/EKF status
        rtk = data.get('rtk')
        if rtk is not None:
            # GPS Fix type display
            fix_types = {
                0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
                4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
            }
            fix_name = fix_types.get(rtk['fix_type'], "Unknown")
            self.gps_fix_text.SetLabel(fix_name)

            # Color code GPS fix
            if rtk['fix_type'] == 6:  # RTK Fixed
                self.gps_fix_text.SetForegroundColour(wx.Colour(0, 150, 0))
            elif rtk['fix_type'] == 5:  # RTK Float
                self.gps_fix_text.SetForegroundColour(wx.Colour(200, 150, 0))
            elif rtk['fix_type'] >= 3:  # 3D Fix or DGPS
                self.gps_fix_text.SetForegroundColour(wx.Colour(0, 100, 200))
            else:
                self.gps_fix_text.SetForegroundColour(wx.Colour(180, 0, 0))

            self.gps_sats_text.SetLabel("Sats: %d" % rtk['satellites'])
            self.gps_hdop_text.SetLabel("HDOP: %.2f" % rtk['hdop'])

            # EKF status
            self.ekf_flags_text.SetLabel("Flags: 0x%04X" % rtk['ekf_flags'])
            self.ekf_var_text.SetLabel("Var: H=%.2f V=%.2f" % (
                rtk['pos_horiz_var'], rtk['pos_vert_var']))

            # RTK enabled status
            if rtk['rtk_enabled']:
                self.rtk_status_text.SetLabel("RTK: Enabled ✓")
                self.rtk_status_text.SetForegroundColour(wx.Colour(0, 150, 0))
            else:
                self.rtk_status_text.SetLabel("RTK: Disabled")
                self.rtk_status_text.SetForegroundColour(wx.Colour(180, 0, 0))

        self.Layout()

    def _update_anchors(self, data):
        """Update anchor table"""
        self.anchors = data['anchors']
        self.origin = data['origin']

        # Clear and resize grid
        if self.grid.GetNumberRows() > 0:
            self.grid.DeleteRows(0, self.grid.GetNumberRows())

        if len(self.anchors) == 0:
            self.SetStatusText("No anchors - Add anchors using 'Add Anchor' button")
            return

        self.grid.AppendRows(len(self.anchors))

        for i, a in enumerate(self.anchors):
            dist = math.sqrt(a['n']**2 + a['e']**2 + a['d']**2)
            self.grid.SetCellValue(i, 0, str(a['id']))
            self.grid.SetCellValue(i, 1, a['name'])
            self.grid.SetCellValue(i, 2, "%.9f" % a['lat'])
            self.grid.SetCellValue(i, 3, "%.9f" % a['lon'])
            self.grid.SetCellValue(i, 4, "%.3f" % a['alt'])
            self.grid.SetCellValue(i, 5, "%+.3f" % a['n'])
            self.grid.SetCellValue(i, 6, "%+.3f" % a['e'])
            self.grid.SetCellValue(i, 7, "%+.3f" % a['d'])
            self.grid.SetCellValue(i, 8, "%.3f" % dist)
            self.grid.SetCellValue(i, 9, str(a['samples']))

            # Alternate row colors
            if i % 2 == 1:
                for j in range(10):
                    self.grid.SetCellBackgroundColour(i, j, wx.Colour(245, 245, 250))

        self.SetStatusText("%d anchors" % len(self.anchors))

    def _on_set_origin(self, event):
        """Set origin button clicked"""
        self.data_queue.put(('set_origin', None))

    def _on_add_anchor(self, event):
        """Add anchor button clicked"""
        dlg = wx.TextEntryDialog(
            self,
            "Enter anchor name (leave blank for auto-generated name):",
            "Add Anchor",
            ""
        )
        if dlg.ShowModal() == wx.ID_OK:
            name = dlg.GetValue().strip() or None
            self.data_queue.put(('add_anchor', name))
        dlg.Destroy()

    def _on_delete_anchor(self, event):
        """Delete selected anchor"""
        rows = self.grid.GetSelectedRows()
        if len(rows) == 0:
            wx.MessageBox(
                "Please select an anchor row to delete",
                "No Selection",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Confirm deletion
        if len(rows) == 1:
            msg = "Delete anchor ID %s?" % self.grid.GetCellValue(rows[0], 0)
        else:
            msg = "Delete %d anchors?" % len(rows)

        dlg = wx.MessageDialog(self, msg, "Confirm Delete", wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION)
        if dlg.ShowModal() == wx.ID_YES:
            for row in sorted(rows, reverse=True):
                anchor_id = self.grid.GetCellValue(row, 0)
                self.data_queue.put(('delete_anchor', anchor_id))
        dlg.Destroy()

    def _on_clear(self, event):
        """Clear all anchors"""
        if len(self.anchors) == 0:
            wx.MessageBox("No anchors to clear", "Info", wx.OK | wx.ICON_INFORMATION)
            return

        dlg = wx.MessageDialog(
            self,
            "Delete ALL %d anchors?\n\nThis cannot be undone." % len(self.anchors),
            "Confirm Clear All",
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING
        )
        if dlg.ShowModal() == wx.ID_YES:
            self.data_queue.put(('clear', None))
        dlg.Destroy()

    def _on_save(self, event):
        """Save anchors to file"""
        if len(self.anchors) == 0:
            wx.MessageBox("No anchors to save", "Info", wx.OK | wx.ICON_INFORMATION)
            return

        dlg = wx.FileDialog(
            self,
            "Save Anchors",
            wildcard="JSON files (*.json)|*.json",
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
            wildcard="JSON files (*.json)|*.json",
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
    """Dialog for setting backsight reference point"""

    def __init__(self, parent):
        super(BacksightDialog, self).__init__(
            parent,
            title="Set Backsight Point",
            size=(350, 250),
            style=wx.DEFAULT_DIALOG_STYLE
        )

        self._create_ui()
        self.Centre()

    def _create_ui(self):
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Description
        desc = wx.StaticText(panel, label="Define a reference point by direction and distance from origin.")
        desc.Wrap(300)
        main_sizer.Add(desc, 0, wx.ALL, 10)

        # Direction selection
        dir_sizer = wx.BoxSizer(wx.HORIZONTAL)
        dir_label = wx.StaticText(panel, label="Direction:")
        dir_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        dir_sizer.Add(dir_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.dir_choice = wx.Choice(panel, choices=['N (North)', 'E (East)', 'S (South)', 'W (West)'])
        self.dir_choice.SetSelection(0)
        dir_sizer.Add(self.dir_choice, 0, wx.ALIGN_CENTER_VERTICAL)
        main_sizer.Add(dir_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # Distance input
        dist_sizer = wx.BoxSizer(wx.HORIZONTAL)
        dist_label = wx.StaticText(panel, label="Distance (m):")
        dist_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        dist_sizer.Add(dist_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.dist_ctrl = wx.TextCtrl(panel, value="10.0", size=(100, -1))
        dist_sizer.Add(self.dist_ctrl, 0, wx.ALIGN_CENTER_VERTICAL)
        main_sizer.Add(dist_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # Threshold input
        thresh_sizer = wx.BoxSizer(wx.HORIZONTAL)
        thresh_label = wx.StaticText(panel, label="Threshold (mm):")
        thresh_label.SetFont(wx.Font(10, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD))
        thresh_sizer.Add(thresh_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)

        self.thresh_ctrl = wx.TextCtrl(panel, value="50", size=(80, -1))
        thresh_sizer.Add(self.thresh_ctrl, 0, wx.ALIGN_CENTER_VERTICAL)

        thresh_help = wx.StaticText(panel, label="(highlight when error < threshold)")
        thresh_help.SetForegroundColour(wx.Colour(100, 100, 100))
        thresh_sizer.Add(thresh_help, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 10)
        main_sizer.Add(thresh_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)

        # Buttons
        btn_sizer = wx.StdDialogButtonSizer()
        ok_btn = wx.Button(panel, wx.ID_OK)
        ok_btn.SetDefault()
        cancel_btn = wx.Button(panel, wx.ID_CANCEL)
        btn_sizer.AddButton(ok_btn)
        btn_sizer.AddButton(cancel_btn)
        btn_sizer.Realize()
        main_sizer.Add(btn_sizer, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        panel.SetSizer(main_sizer)

    def get_values(self):
        """Get dialog values"""
        dir_map = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}
        direction = dir_map[self.dir_choice.GetSelection()]

        try:
            distance = float(self.dist_ctrl.GetValue())
        except ValueError:
            distance = 10.0

        try:
            threshold = int(self.thresh_ctrl.GetValue())
        except ValueError:
            threshold = 50

        return direction, distance, threshold
