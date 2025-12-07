#!/usr/bin/env python
'''
Launcher GUI Module - Load modules through button clicks
'''

import time
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util


class LauncherModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(LauncherModule, self).__init__(mpstate, "launcher", "Module launcher GUI", public=True)
        self.add_command('launcher', self.cmd_launcher, "launcher GUI control")
        
        self.gui = None
        self._start_gui()

    def cmd_launcher(self, args):
        '''launcher command'''
        if len(args) == 0:
            print("Usage: launcher <gui|show|hide>")
            return
        
        cmd = args[0].lower()
        if cmd == 'gui':
            self._start_gui()
        elif cmd == 'show':
            self._start_gui()
            if self.gui:
                self.gui.show()
        elif cmd == 'hide':
            if self.gui:
                self.gui.hide()
        else:
            print("Unknown command: %s" % cmd)

    def _start_gui(self):
        """Start the launcher GUI"""
        if self.gui is not None and self.gui._alive and self.gui._process.is_alive():
            return
        try:
            from MAVProxy.modules.mavproxy_launcher import launcher_gui
            self.gui = launcher_gui.LauncherGUI(self)
        except Exception as e:
            print("Launcher GUI error: %s" % e)
            import traceback
            traceback.print_exc()

    def load_module(self, module_name):
        """Load a module by name"""
        try:
            self.mpstate.functions.process_stdin("module load %s" % module_name)
            return True
        except Exception as e:
            print("Failed to load %s: %s" % (module_name, e))
            return False

    def unload_module(self, module_name):
        """Unload a module by name"""
        try:
            self.mpstate.functions.process_stdin("module unload %s" % module_name)
            return True
        except Exception as e:
            print("Failed to unload %s: %s" % (module_name, e))
            return False

    def is_module_loaded(self, module_name):
        """Check if a module is loaded"""
        for m in self.mpstate.modules:
            if hasattr(m, 'name') and m.name == module_name:
                return True
            elif isinstance(m, tuple) and len(m) > 0 and hasattr(m[0], 'name') and m[0].name == module_name:
                return True
        return False

    def get_loaded_modules(self):
        """Get list of loaded module names"""
        names = []
        for m in self.mpstate.modules:
            if hasattr(m, 'name'):
                names.append(m.name)
            elif isinstance(m, tuple) and len(m) > 0 and hasattr(m[0], 'name'):
                names.append(m[0].name)
        return names

    def unload(self):
        """Called when module is unloaded"""
        if self.gui:
            self.gui.close()


def init(mpstate):
    '''initialise module'''
    return LauncherModule(mpstate)
