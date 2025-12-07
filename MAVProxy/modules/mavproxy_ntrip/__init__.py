"""
send NTRIP data to flight controller
"""

import json
import os
import random
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import ntrip
from MAVProxy.modules.lib import mp_settings


class NtripModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(NtripModule, self).__init__(mpstate, "ntrip", "ntrip", public=False)
        self.ntrip_settings = mp_settings.MPSettings(
            [('caster', str, None),
             ('port', int, 2101),
             ('username', str, 'IBS'),
             ('password', str, 'IBS'),
             ('mountpoint', str, None),
             ('logfile', str, None),
             ('sendalllinks', bool, False),
             ('frag_drop_pct', float, 0),
             ('sendmul', int, 1),
             ('autostart', bool, False),
             ('gui', bool, True)])
        self.add_command('ntrip', self.cmd_ntrip, 'NTRIP control',
                         ["<status>",
                          "<start>",
                          "<stop>",
                          "<save>",
                          "<load>",
                          "<gui>",
                          "set (NTRIPSETTING)"])
        
        # Config file path
        self.config_file = os.path.join(os.path.expanduser("~"), ".mavproxy_ntrip.json")
        self.add_completion_function('(NTRIPSETTING)',
                                     self.ntrip_settings.completion)
        self.pos = None
        self.pkt_count = 0
        self.last_pkt = None
        self.last_restart = None
        self.last_rate = None
        self.rate_total = 0
        self.ntrip = None
        self.start_pending = False
        self.rate = 0
        self.logfile = None
        self.id_counts = {}
        self.last_by_id = {}
        
        # GUI
        self.gui = None
        self.last_gui_update = 0
        
        # Auto-load saved settings
        self._load_settings()
        
        # Auto-start if enabled
        if self.ntrip_settings.autostart:
            self.start_pending = True
            print("NTRIP: autostart enabled, waiting for position")
        
        # Start GUI if enabled
        if self.ntrip_settings.gui:
            self._start_gui()

    def _start_gui(self):
        """Start the GUI"""
        # Check if GUI already exists and is alive
        if self.gui is not None and self.gui.is_alive():
            return
        try:
            from MAVProxy.modules.mavproxy_ntrip import ntrip_gui
            self.gui = ntrip_gui.NtripGUI(self)
        except Exception as e:
            print("NTRIP GUI error: %s" % e)
            import traceback
            traceback.print_exc()

    def _update_gui(self):
        """Send status update to GUI"""
        if self.gui is None or not self.gui.is_alive():
            return
        
        now = time.time()
        if now - self.last_gui_update < 0.2:  # 5 Hz max
            return
        self.last_gui_update = now
        
        data = {
            'connected': self.ntrip is not None,
            'pending': self.start_pending,
            'pkt_count': self.pkt_count,
            'rate': self.rate,
            'last_pkt': self.last_pkt,
            'pos': self.pos,
            'caster': self.ntrip_settings.caster,
            'port': self.ntrip_settings.port,
            'mountpoint': self.ntrip_settings.mountpoint,
            'settings': {
                'caster': self.ntrip_settings.caster,
                'port': self.ntrip_settings.port,
                'mountpoint': self.ntrip_settings.mountpoint,
                'username': self.ntrip_settings.username,
                'password': self.ntrip_settings.password,
                'autostart': self.ntrip_settings.autostart,
                'sendalllinks': self.ntrip_settings.sendalllinks,
                'sendmul': self.ntrip_settings.sendmul,
            }
        }
        self.gui.update_data(data)

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if msg.get_type() in ['GPS_RAW_INT', 'GPS2_RAW']:
            if msg.fix_type >= 3:
                self.pos = (msg.lat*1.0e-7, msg.lon*1.0e-7, msg.alt*1.0e-3)

    def log_rtcm(self, data):
        '''optionally log rtcm data'''
        if self.ntrip_settings.logfile is None:
            return
        if self.logfile is None:
            self.logfile = open(self.ntrip_settings.logfile, 'wb')
        if self.logfile is not None:
            self.logfile.write(data)

    def idle_task(self):
        '''called on idle'''
        # Update GUI
        self._update_gui()
        
        if self.start_pending and self.ntrip is None and self.pos is not None:
            self.cmd_start()
        if self.ntrip is None:
            return
        data = self.ntrip.read()
        if data is None:
            now = time.time()
            if (self.last_pkt is not None and
                now - self.last_pkt > 15 and
                (self.last_restart is None or now - self.last_restart > 30)):
                print("NTRIP restart")
                self.ntrip = None
                self.start_pending = True
                self.last_restart = now
            return
        if time.time() - self.ntrip.dt_last_gga_sent > 2:
            self.ntrip.setPosition(self.pos[0], self.pos[1])
            self.ntrip.send_gga()
        self.log_rtcm(data)

        rtcm_id = self.ntrip.get_ID()
        if not rtcm_id in self.id_counts:
            self.id_counts[rtcm_id] = 0
            self.last_by_id[rtcm_id] = data[:]
        self.id_counts[rtcm_id] += 1

        blen = len(data)
        if blen > 4*180:
            # can't send this with GPS_RTCM_DATA
            return
        total_len = blen
        self.rate_total += blen * self.ntrip_settings.sendmul

        if blen > 180:
            flags = 1 # fragmented
        else:
            flags = 0
        # add in the sequence number
        flags |= (self.pkt_count & 0x1F) << 3

        fragment = 0
        while blen > 0:
            send_data = bytearray(data[:180])
            frag_len = len(send_data)
            data = data[frag_len:]
            if frag_len < 180:
                send_data.extend(bytearray([0]*(180-frag_len)))
            if self.ntrip_settings.sendalllinks:
                links = self.mpstate.mav_master
            else:
                links = [self.master]
            for link in links:
                for d in range(self.ntrip_settings.sendmul):
                    if random.random() * 100 < self.ntrip_settings.frag_drop_pct:
                        continue
                    link.mav.gps_rtcm_data_send(flags | (fragment<<1), frag_len, send_data)
            fragment += 1
            blen -= frag_len
        self.pkt_count += 1

        now = time.time()
        if now - self.last_rate > 1:
            dt = now - self.last_rate
            rate_now = self.rate_total / float(dt)
            self.rate = 0.9 * self.rate + 0.1 * rate_now
            self.last_rate = now
            self.rate_total = 0
        self.last_pkt = now

    def cmd_ntrip(self, args):
        '''ntrip command handling'''
        if len(args) <= 0:
            print("Usage: ntrip <start|stop|status|save|load|gui|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        elif args[0] == "stop":
            self.ntrip = None
            self.start_pending = False
        elif args[0] == "status":
            self.ntrip_status()
        elif args[0] == "save":
            self._save_settings()
        elif args[0] == "load":
            self._load_settings()
            print("NTRIP settings loaded")
        elif args[0] == "gui":
            self._start_gui()
        elif args[0] == "set":
            self.ntrip_settings.command(args[1:])

    def ntrip_status(self):
        '''show ntrip status'''
        now = time.time()
        if self.ntrip is None:
            print("ntrip: Not started")
            return
        elif self.last_pkt is None:
            print("ntrip: no data")
            return
        frame_size = 0
        for id in sorted(self.id_counts.keys()):
            print(" %4u: %u (len %u)" % (id, self.id_counts[id], len(self.last_by_id[id])))
            frame_size += len(self.last_by_id[id])
        print("ntrip: %u packets, %.1f bytes/sec last %.1fs ago framesize %u" % (self.pkt_count, self.rate, now - self.last_pkt, frame_size))

    def cmd_start(self):
        '''start ntrip link'''
        if self.ntrip_settings.caster is None:
            print("Require caster")
            return
        if self.ntrip_settings.mountpoint is None:
            print("Require mountpoint")
            return
        if self.pos is None:
            print("Start delayed pending position")
            self.start_pending = True
            return
        user = self.ntrip_settings.username + ":" + self.ntrip_settings.password
        self.ntrip = ntrip.NtripClient(user=user,
                                       port=self.ntrip_settings.port,
                                       caster=self.ntrip_settings.caster,
                                       mountpoint=self.ntrip_settings.mountpoint,
                                       lat=self.pos[0],
                                       lon=self.pos[1],
                                       height=self.pos[2])
        print("NTRIP started")
        self.start_pending = False
        self.last_rate = time.time()
        self.rate_total = 0

    def _save_settings(self):
        '''Save settings to config file'''
        config = {
            'caster': self.ntrip_settings.caster,
            'port': self.ntrip_settings.port,
            'username': self.ntrip_settings.username,
            'password': self.ntrip_settings.password,
            'mountpoint': self.ntrip_settings.mountpoint,
            'sendalllinks': self.ntrip_settings.sendalllinks,
            'sendmul': self.ntrip_settings.sendmul,
            'autostart': self.ntrip_settings.autostart
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            print("NTRIP settings saved to %s" % self.config_file)
        except Exception as e:
            print("Failed to save NTRIP settings: %s" % e)

    def _load_settings(self):
        '''Load settings from config file'''
        if not os.path.exists(self.config_file):
            return
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
            for key, value in config.items():
                if hasattr(self.ntrip_settings, key):
                    setattr(self.ntrip_settings, key, value)
        except Exception as e:
            print("Failed to load NTRIP settings: %s" % e)

    def unload(self):
        '''Called when module is unloaded'''
        if self.gui is not None:
            self.gui.close()


def init(mpstate):
    '''initialise module'''
    return NtripModule(mpstate)
