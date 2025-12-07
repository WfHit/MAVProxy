#!/usr/bin/env python
'''
Launcher GUI - Small screen module loader
Optimized for 3.5 inch screens (480x280)
'''

import multiprocessing
from multiprocessing import Process, Queue

try:
    import wx
except ImportError:
    wx = None

# Max size for 3.5 inch display
MAX_WIDTH = 480
MAX_HEIGHT = 280


class LauncherGUI:
    """GUI manager for launcher"""

    def __init__(self, module):
        self.module = module
        self._alive = True

        # Queues for communication
        self.cmd_queue = Queue()
        self.status_queue = Queue()

        # Start GUI process
        self._process = Process(target=self._run_gui, args=(self.cmd_queue, self.status_queue))
        self._process.daemon = True
        self._process.start()

        # Start command processor
        import threading
        self._cmd_thread = threading.Thread(target=self._process_commands)
        self._cmd_thread.daemon = True
        self._cmd_thread.start()

    def close(self):
        """Close the GUI"""
        self._alive = False
        self.cmd_queue.put(('close', None))
        if self._process.is_alive():
            self._process.terminate()

    def show(self):
        """Show the GUI window"""
        self.cmd_queue.put(('show', None))

    def hide(self):
        """Hide the GUI window"""
        self.cmd_queue.put(('hide', None))

    def _process_commands(self):
        """Process commands from GUI"""
        while self._alive:
            try:
                cmd = self.status_queue.get(timeout=0.1)
            except Exception:
                continue

            cmd_type, data = cmd
            if cmd_type == 'load':
                self.module.load_module(data)
            elif cmd_type == 'unload':
                self.module.unload_module(data)
            elif cmd_type == 'check_loaded':
                # Send back loaded status
                loaded = self.module.get_loaded_modules()
                self.cmd_queue.put(('loaded_modules', loaded))
            elif cmd_type == 'closed':
                pass

    def _run_gui(self, cmd_queue, status_queue):
        """Run GUI in separate process"""
        import wx
        app = wx.App()
        frame = LauncherFrame(cmd_queue, status_queue)
        frame.Show()
        app.MainLoop()


class LauncherFrame(wx.Frame):
    """Launcher GUI window"""

    def __init__(self, cmd_queue, status_queue):
        super(LauncherFrame, self).__init__(
            None,
            title="Launcher",
            style=wx.DEFAULT_FRAME_STYLE
        )
        self.cmd_queue = cmd_queue
        self.status_queue = status_queue

        # Module definitions: (name, display_name, description)
        self.modules = [
            ('uwbanchor', 'UWB Anchor', 'UWB anchor deployment'),
            ('uwbtest', 'UWB Test', 'UWB system testing'),
            ('ntrip', 'NTRIP', 'RTK corrections'),
            ('console', 'Console', 'Status console'),
            ('map', 'Map', 'Map display'),
        ]

        # Track loaded state
        self.loaded_modules = set()

        # Fonts for screen
        self.small_font = wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.bold_font = wx.Font(12, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)

        self._create_ui()
        
        # Set initial size (480x280) but allow resize/maximize
        self.SetSize(wx.Size(MAX_WIDTH, MAX_HEIGHT))
        self.SetMinSize(wx.Size(MAX_WIDTH, MAX_HEIGHT))
        self.Centre()

        # Timer for updates
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_timer, self.timer)
        self.timer.Start(500)

        self.Bind(wx.EVT_CLOSE, self._on_close)

        # Request initial status
        self.status_queue.put(('check_loaded', None))

    def _create_ui(self):
        """Create the UI"""
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Title
        title = wx.StaticText(panel, label="Module Launcher")
        title.SetFont(self.bold_font)
        main_sizer.Add(title, 0, wx.ALL | wx.ALIGN_CENTER, 8)

        # Module buttons in a multi-column grid (3 columns)
        grid_sizer = wx.FlexGridSizer(cols=3, hgap=10, vgap=10)

        self.module_btns = {}
        self.status_labels = {}

        # Button size to fill screen: (480-margins)/3 cols ≈ 145px wide
        btn_width = 145
        btn_height = 60

        for mod_name, display_name, desc in self.modules:
            # Load/Unload button
            btn = wx.ToggleButton(panel, label=display_name, size=(btn_width, btn_height))
            btn.SetFont(self.small_font)
            btn.SetToolTip(desc)
            btn.Bind(wx.EVT_TOGGLEBUTTON, lambda e, m=mod_name: self._on_toggle(m, e))
            self.module_btns[mod_name] = btn
            grid_sizer.Add(btn, 0, wx.EXPAND)

            # Hidden status label (for tracking state)
            status = wx.StaticText(panel, label="", size=(0, 0))
            status.Hide()
            self.status_labels[mod_name] = status

        main_sizer.Add(grid_sizer, 1, wx.ALL | wx.ALIGN_CENTER, 10)

        # Button row (Hide and Stop)
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        
        hide_btn = wx.Button(panel, label="Hide", size=(100, 45))
        hide_btn.SetFont(self.small_font)
        hide_btn.SetToolTip("Hide window (launcher gui to reopen)")
        hide_btn.Bind(wx.EVT_BUTTON, self._on_hide_btn)
        btn_sizer.Add(hide_btn, 0, wx.RIGHT, 20)
        
        stop_btn = wx.Button(panel, label="Stop", size=(100, 45))
        stop_btn.SetFont(self.small_font)
        stop_btn.SetToolTip("Close launcher (launcher gui to restart)")
        stop_btn.Bind(wx.EVT_BUTTON, self._on_stop_btn)
        btn_sizer.Add(stop_btn, 0)
        
        main_sizer.Add(btn_sizer, 0, wx.ALIGN_CENTER | wx.BOTTOM, 10)

        panel.SetSizer(main_sizer)

    def _on_toggle(self, module_name, event):
        """Handle module toggle"""
        btn = event.GetEventObject()
        if btn.GetValue():
            # Load module
            self.status_queue.put(('load', module_name))
            self.status_labels[module_name].SetLabel("Loading...")
        else:
            # Unload module
            self.status_queue.put(('unload', module_name))
            self.status_labels[module_name].SetLabel("Unloading...")

    def _on_timer(self, event):
        """Process updates from main module"""
        while True:
            try:
                if self.cmd_queue.empty():
                    break
                cmd = self.cmd_queue.get_nowait()
            except Exception:
                break

            cmd_type, data = cmd
            if cmd_type == 'loaded_modules':
                self.loaded_modules = set(data)
                self._update_buttons()
            elif cmd_type == 'show':
                self.Show()
                self.Raise()
            elif cmd_type == 'hide':
                self.Hide()
            elif cmd_type == 'close':
                self.Destroy()

        # Periodically check loaded status
        self.status_queue.put(('check_loaded', None))

    def _update_buttons(self):
        """Update button states based on loaded modules"""
        for mod_name, btn in self.module_btns.items():
            is_loaded = mod_name in self.loaded_modules
            btn.SetValue(is_loaded)
            if is_loaded:
                btn.SetBackgroundColour(wx.Colour(144, 238, 144))  # Light green
            else:
                btn.SetBackgroundColour(wx.NullColour)  # Default color
            btn.Refresh()

    def _on_hide_btn(self, event):
        """Hide button handler - hide window but keep running"""
        self.Hide()

    def _on_stop_btn(self, event):
        """Stop button handler - close the GUI completely"""
        self.status_queue.put(('closed', None))
        self.Destroy()

    def _on_close(self, event):
        """Window close handler"""
        self.status_queue.put(('closed', None))
        self.Destroy()
