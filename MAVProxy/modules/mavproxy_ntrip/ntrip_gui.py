"""
  NTRIP GUI - Small Screen Version
  Optimized for 3.5 inch screens (480x280)
"""

try:
    import wx
except ImportError:
    wx = None

from multiprocessing import Process, Queue

# Screen size for 3.5 inch display
SMALL_SCREEN_WIDTH = 480
SMALL_SCREEN_HEIGHT = 280


class NtripGUI:
    """GUI manager that runs in separate process"""

    def __init__(self, module):
        self.module = module
        self.cmd_queue = Queue()
        self.data_queue = Queue()
        self.process = Process(target=self._run_gui, args=(self.cmd_queue, self.data_queue))
        self.process.daemon = True
        self.process.start()
        self._alive = True

        # Start command processor thread
        import threading
        self._cmd_thread = threading.Thread(target=self._process_gui_commands)
        self._cmd_thread.daemon = True
        self._cmd_thread.start()

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

    def _process_gui_commands(self):
        """Process commands from GUI"""
        while self._alive:
            try:
                cmd = self.data_queue.get(timeout=0.1)
            except Exception:
                continue

            cmd_type, data = cmd
            if cmd_type == 'start':
                self.module.cmd_start()
            elif cmd_type == 'stop':
                self.module.cmd_ntrip(['stop'])
            elif cmd_type == 'save':
                self.module._save_settings()
            elif cmd_type == 'set':
                # data is (setting_name, value)
                self.module.ntrip_settings.command([data[0], data[1]])
            elif cmd_type == 'closed':
                self._alive = False

    def _run_gui(self, cmd_queue, data_queue):
        """Run GUI in separate process"""
        import wx
        app = wx.App()
        frame = NtripFrame(cmd_queue, data_queue)
        frame.Show()
        app.MainLoop()


class NtripFrame(wx.Frame):
    """NTRIP GUI window - Tabbed interface for small screens"""

    def __init__(self, cmd_queue, data_queue):
        import wx
        super(NtripFrame, self).__init__(
            None,
            title="NTRIP",
            size=(SMALL_SCREEN_WIDTH, SMALL_SCREEN_HEIGHT),
            style=wx.DEFAULT_FRAME_STYLE
        )
        self.SetMinSize((SMALL_SCREEN_WIDTH, SMALL_SCREEN_HEIGHT))
        self.Centre()
        self.cmd_queue = cmd_queue
        self.data_queue = data_queue

        # Fonts matching launcher GUI style
        self.small_font = wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.mono_font = wx.Font(11, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.bold_font = wx.Font(12, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        self.large_font = wx.Font(12, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)

        self._create_ui()

        # Timer for processing commands
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(100)

        self.Bind(wx.EVT_CLOSE, self._on_close)

    def _create_ui(self):
        """Create tabbed interface"""
        import wx
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create notebook (tabs)
        self.notebook = wx.Notebook(panel)

        # Tab 1: Status
        self.status_panel = wx.Panel(self.notebook)
        self._create_status_tab(self.status_panel)
        self.notebook.AddPage(self.status_panel, "Status")

        # Tab 2: Server Settings
        self.server_panel = wx.Panel(self.notebook)
        self._create_server_tab(self.server_panel)
        self.notebook.AddPage(self.server_panel, "Server")

        # Tab 3: Options
        self.options_panel = wx.Panel(self.notebook)
        self._create_options_tab(self.options_panel)
        self.notebook.AddPage(self.options_panel, "Options")

        main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 1)
        panel.SetSizer(main_sizer)

    def _create_status_tab(self, panel):
        """Create status display tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Connection status
        status_box = wx.StaticBox(panel, label="Connection")
        status_sizer = wx.StaticBoxSizer(status_box, wx.VERTICAL)

        self.conn_status = wx.StaticText(panel, label="Disconnected")
        self.conn_status.SetFont(self.large_font)
        self.conn_status.SetForegroundColour(wx.Colour(180, 0, 0))
        status_sizer.Add(self.conn_status, 0, wx.LEFT | wx.RIGHT, 3)

        # Server info
        self.server_text = wx.StaticText(panel, label="Server: ---")
        self.server_text.SetFont(self.mono_font)
        status_sizer.Add(self.server_text, 0, wx.LEFT | wx.RIGHT, 3)

        self.mount_text = wx.StaticText(panel, label="Mount: ---")
        self.mount_text.SetFont(self.mono_font)
        status_sizer.Add(self.mount_text, 0, wx.LEFT | wx.RIGHT, 3)

        sizer.Add(status_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Data status
        data_box = wx.StaticBox(panel, label="Data")
        data_sizer = wx.StaticBoxSizer(data_box, wx.VERTICAL)

        row1 = wx.BoxSizer(wx.HORIZONTAL)
        self.pkt_text = wx.StaticText(panel, label="Pkts: 0", size=(70, -1))
        self.rate_text = wx.StaticText(panel, label="Rate: 0 B/s", size=(90, -1))
        self.pkt_text.SetFont(self.mono_font)
        self.rate_text.SetFont(self.mono_font)
        row1.Add(self.pkt_text, 0, wx.RIGHT, 5)
        row1.Add(self.rate_text, 0)
        data_sizer.Add(row1, 0, wx.LEFT | wx.RIGHT, 3)

        self.last_pkt_text = wx.StaticText(panel, label="Last: ---")
        self.last_pkt_text.SetFont(self.mono_font)
        data_sizer.Add(self.last_pkt_text, 0, wx.LEFT | wx.RIGHT, 3)

        sizer.Add(data_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Position
        pos_box = wx.StaticBox(panel, label="Position")
        pos_sizer = wx.StaticBoxSizer(pos_box, wx.HORIZONTAL)
        self.lat_text = wx.StaticText(panel, label="Lat: ---", size=(100, -1))
        self.lon_text = wx.StaticText(panel, label="Lon: ---", size=(100, -1))
        self.lat_text.SetFont(self.mono_font)
        self.lon_text.SetFont(self.mono_font)
        pos_sizer.Add(self.lat_text, 0, wx.RIGHT, 5)
        pos_sizer.Add(self.lon_text, 0)
        sizer.Add(pos_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.start_btn = wx.Button(panel, label="Start", size=(70, 24))
        self.stop_btn = wx.Button(panel, label="Stop", size=(70, 24))
        self.save_btn = wx.Button(panel, label="Save", size=(70, 24))

        self.stop_btn.Enable(False)

        self.start_btn.Bind(wx.EVT_BUTTON, self._on_start)
        self.stop_btn.Bind(wx.EVT_BUTTON, self._on_stop)
        self.save_btn.Bind(wx.EVT_BUTTON, self._on_save)

        btn_sizer.Add(self.start_btn, 1, wx.RIGHT, 2)
        btn_sizer.Add(self.stop_btn, 1, wx.RIGHT, 2)
        btn_sizer.Add(self.save_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_server_tab(self, panel):
        """Create server settings tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Caster
        caster_row = wx.BoxSizer(wx.HORIZONTAL)
        caster_label = wx.StaticText(panel, label="Caster:", size=(50, -1))
        caster_label.SetFont(self.small_font)
        caster_row.Add(caster_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.caster_ctrl = wx.TextCtrl(panel, size=(150, -1))
        caster_row.Add(self.caster_ctrl, 1)
        sizer.Add(caster_row, 0, wx.EXPAND | wx.ALL, 3)

        # Port
        port_row = wx.BoxSizer(wx.HORIZONTAL)
        port_label = wx.StaticText(panel, label="Port:", size=(50, -1))
        port_label.SetFont(self.small_font)
        port_row.Add(port_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.port_ctrl = wx.SpinCtrl(panel, min=1, max=65535, initial=2101, size=(70, -1))
        port_row.Add(self.port_ctrl, 0)
        sizer.Add(port_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # Mountpoint
        mount_row = wx.BoxSizer(wx.HORIZONTAL)
        mount_label = wx.StaticText(panel, label="Mount:", size=(50, -1))
        mount_label.SetFont(self.small_font)
        mount_row.Add(mount_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.mount_ctrl = wx.TextCtrl(panel, size=(120, -1))
        mount_row.Add(self.mount_ctrl, 1)
        sizer.Add(mount_row, 0, wx.EXPAND | wx.ALL, 3)

        # Username
        user_row = wx.BoxSizer(wx.HORIZONTAL)
        user_label = wx.StaticText(panel, label="User:", size=(50, -1))
        user_label.SetFont(self.small_font)
        user_row.Add(user_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.user_ctrl = wx.TextCtrl(panel, size=(100, -1))
        user_row.Add(self.user_ctrl, 1)
        sizer.Add(user_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 3)

        # Password
        pass_row = wx.BoxSizer(wx.HORIZONTAL)
        pass_label = wx.StaticText(panel, label="Pass:", size=(50, -1))
        pass_label.SetFont(self.small_font)
        pass_row.Add(pass_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.pass_ctrl = wx.TextCtrl(panel, size=(100, -1), style=wx.TE_PASSWORD)
        pass_row.Add(self.pass_ctrl, 1)
        sizer.Add(pass_row, 0, wx.EXPAND | wx.ALL, 3)

        # Apply button
        self.apply_server_btn = wx.Button(panel, label="Apply", size=(70, 24))
        self.apply_server_btn.Bind(wx.EVT_BUTTON, self._on_apply_server)
        sizer.Add(self.apply_server_btn, 0, wx.ALL, 3)

        panel.SetSizer(sizer)

    def _create_options_tab(self, panel):
        """Create options tab"""
        import wx
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Autostart
        self.autostart_check = wx.CheckBox(panel, label="Autostart on load")
        self.autostart_check.SetFont(self.small_font)
        sizer.Add(self.autostart_check, 0, wx.ALL, 5)

        # Send to all links
        self.sendall_check = wx.CheckBox(panel, label="Send to all links")
        self.sendall_check.SetFont(self.small_font)
        sizer.Add(self.sendall_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Send multiplier
        mul_row = wx.BoxSizer(wx.HORIZONTAL)
        mul_label = wx.StaticText(panel, label="Send mul:")
        mul_label.SetFont(self.small_font)
        mul_row.Add(mul_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        self.sendmul_spin = wx.SpinCtrl(panel, min=1, max=10, initial=1, size=(50, -1))
        mul_row.Add(self.sendmul_spin, 0)
        sizer.Add(mul_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Apply button
        self.apply_opts_btn = wx.Button(panel, label="Apply Options", size=(100, 24))
        self.apply_opts_btn.Bind(wx.EVT_BUTTON, self._on_apply_options)
        sizer.Add(self.apply_opts_btn, 0, wx.ALL, 5)

        panel.SetSizer(sizer)

    def _on_timer(self, event):
        """Process commands from main module"""
        while True:
            try:
                if self.cmd_queue.empty():
                    break
                cmd = self.cmd_queue.get_nowait()
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

        # Connection status
        if data['connected']:
            self.conn_status.SetLabel("Connected")
            self.conn_status.SetForegroundColour(wx.Colour(0, 128, 0))
            self.start_btn.Enable(False)
            self.stop_btn.Enable(True)
        else:
            if data['pending']:
                self.conn_status.SetLabel("Pending...")
                self.conn_status.SetForegroundColour(wx.Colour(200, 150, 0))
            else:
                self.conn_status.SetLabel("Disconnected")
                self.conn_status.SetForegroundColour(wx.Colour(180, 0, 0))
            self.start_btn.Enable(True)
            self.stop_btn.Enable(False)

        # Server info
        if data['caster']:
            self.server_text.SetLabel("Server: %s:%d" % (data['caster'], data['port']))
        if data['mountpoint']:
            self.mount_text.SetLabel("Mount: %s" % data['mountpoint'])

        # Data stats
        self.pkt_text.SetLabel("Pkts: %d" % data['pkt_count'])
        self.rate_text.SetLabel("Rate: %.0f B/s" % data['rate'])

        if data['last_pkt'] is not None:
            import time
            ago = time.time() - data['last_pkt']
            self.last_pkt_text.SetLabel("Last: %.1fs ago" % ago)
        else:
            self.last_pkt_text.SetLabel("Last: ---")

        # Position
        if data['pos']:
            self.lat_text.SetLabel("Lat: %.6f" % data['pos'][0])
            self.lon_text.SetLabel("Lon: %.6f" % data['pos'][1])

        # Settings (update fields if not focused)
        settings = data.get('settings', {})
        if settings:
            if settings.get('caster') and not self.caster_ctrl.HasFocus():
                self.caster_ctrl.SetValue(str(settings['caster']) if settings['caster'] else '')
            if settings.get('port') and not self.port_ctrl.HasFocus():
                self.port_ctrl.SetValue(settings['port'])
            if settings.get('mountpoint') and not self.mount_ctrl.HasFocus():
                self.mount_ctrl.SetValue(str(settings['mountpoint']) if settings['mountpoint'] else '')
            if settings.get('username') and not self.user_ctrl.HasFocus():
                self.user_ctrl.SetValue(str(settings['username']) if settings['username'] else '')
            if settings.get('password') and not self.pass_ctrl.HasFocus():
                self.pass_ctrl.SetValue(str(settings['password']) if settings['password'] else '')
            self.autostart_check.SetValue(settings.get('autostart', False))
            self.sendall_check.SetValue(settings.get('sendalllinks', False))
            self.sendmul_spin.SetValue(settings.get('sendmul', 1))

    def _on_start(self, event):
        self.data_queue.put(('start', None))

    def _on_stop(self, event):
        self.data_queue.put(('stop', None))

    def _on_save(self, event):
        self.data_queue.put(('save', None))

    def _on_apply_server(self, event):
        self.data_queue.put(('set', ('caster', self.caster_ctrl.GetValue())))
        self.data_queue.put(('set', ('port', str(self.port_ctrl.GetValue()))))
        self.data_queue.put(('set', ('mountpoint', self.mount_ctrl.GetValue())))
        self.data_queue.put(('set', ('username', self.user_ctrl.GetValue())))
        self.data_queue.put(('set', ('password', self.pass_ctrl.GetValue())))

    def _on_apply_options(self, event):
        self.data_queue.put(('set', ('autostart', str(self.autostart_check.GetValue()).lower())))
        self.data_queue.put(('set', ('sendalllinks', str(self.sendall_check.GetValue()).lower())))
        self.data_queue.put(('set', ('sendmul', str(self.sendmul_spin.GetValue()))))

    def _on_close(self, event):
        self.timer.Stop()
        self.data_queue.put(('closed', None))
        self.Destroy()
