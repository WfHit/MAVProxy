"""
  MAVProxy UWB Fusion Accuracy Test Module
  
  Tests UWB fusion accuracy by comparing EKF position (with UWB, without GPS)
  against RTK GPS reference position.
"""

import os
import sys
import math
import time
import json
import csv
from datetime import datetime
from collections import deque

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil

# WGS84 ellipsoid parameters
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def geodetic_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    """Convert WGS84 to NED relative to origin"""
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    sin_lat = math.sin(origin_lat_rad)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    M = WGS84_A * (1 - WGS84_E2) / pow(1 - WGS84_E2 * sin_lat * sin_lat, 1.5)

    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad
    dalt = alt - origin_alt

    north = dlat * M
    east = dlon * N * math.cos(origin_lat_rad)
    down = -dalt

    return north, east, down


class TestSample:
    """Single test sample with EKF and RTK positions"""
    def __init__(self, timestamp, ekf_lat, ekf_lon, ekf_alt,
                 rtk_lat, rtk_lon, rtk_alt, rtk_fix_type):
        self.timestamp = timestamp
        self.ekf_lat = ekf_lat
        self.ekf_lon = ekf_lon
        self.ekf_alt = ekf_alt
        self.rtk_lat = rtk_lat
        self.rtk_lon = rtk_lon
        self.rtk_alt = rtk_alt
        self.rtk_fix_type = rtk_fix_type
        
        # Error will be calculated when origin is set
        self.err_n = None
        self.err_e = None
        self.err_d = None
        self.err_horiz = None
        self.err_total = None

    def calculate_error(self, origin_lat, origin_lon, origin_alt):
        """Calculate error between EKF and RTK positions in NED"""
        if self.ekf_lat is None or self.rtk_lat is None:
            return
        
        # Convert both to NED relative to origin
        ekf_n, ekf_e, ekf_d = geodetic_to_ned(
            self.ekf_lat, self.ekf_lon, self.ekf_alt,
            origin_lat, origin_lon, origin_alt
        )
        rtk_n, rtk_e, rtk_d = geodetic_to_ned(
            self.rtk_lat, self.rtk_lon, self.rtk_alt,
            origin_lat, origin_lon, origin_alt
        )
        
        # Error = EKF - RTK (positive means EKF is ahead/right/down of RTK)
        self.err_n = ekf_n - rtk_n
        self.err_e = ekf_e - rtk_e
        self.err_d = ekf_d - rtk_d
        self.err_horiz = math.sqrt(self.err_n**2 + self.err_e**2)
        self.err_total = math.sqrt(self.err_n**2 + self.err_e**2 + self.err_d**2)

    def to_dict(self):
        return {
            'timestamp': self.timestamp,
            'ekf_lat': self.ekf_lat,
            'ekf_lon': self.ekf_lon,
            'ekf_alt': self.ekf_alt,
            'rtk_lat': self.rtk_lat,
            'rtk_lon': self.rtk_lon,
            'rtk_alt': self.rtk_alt,
            'rtk_fix_type': self.rtk_fix_type,
            'err_n': self.err_n,
            'err_e': self.err_e,
            'err_d': self.err_d,
            'err_horiz': self.err_horiz,
            'err_total': self.err_total
        }


class TestStatistics:
    """Statistics calculated from test samples"""
    def __init__(self, samples):
        self.sample_count = len(samples)
        self.duration = 0
        
        # Initialize stats
        self.mean_n = 0
        self.mean_e = 0
        self.mean_d = 0
        self.mean_horiz = 0
        self.mean_total = 0
        
        self.std_n = 0
        self.std_e = 0
        self.std_d = 0
        self.std_horiz = 0
        self.std_total = 0
        
        self.max_n = 0
        self.max_e = 0
        self.max_d = 0
        self.max_horiz = 0
        self.max_total = 0
        
        self.rms_n = 0
        self.rms_e = 0
        self.rms_d = 0
        self.rms_horiz = 0
        self.rms_total = 0
        
        self.cep50 = 0  # 50th percentile horizontal error
        self.cep95 = 0  # 95th percentile horizontal error
        
        if len(samples) == 0:
            return
        
        # Filter samples with valid errors
        valid = [s for s in samples if s.err_n is not None]
        if len(valid) == 0:
            return
        
        self.sample_count = len(valid)
        self.duration = valid[-1].timestamp - valid[0].timestamp
        
        # Calculate means
        self.mean_n = sum(s.err_n for s in valid) / len(valid)
        self.mean_e = sum(s.err_e for s in valid) / len(valid)
        self.mean_d = sum(s.err_d for s in valid) / len(valid)
        self.mean_horiz = sum(s.err_horiz for s in valid) / len(valid)
        self.mean_total = sum(s.err_total for s in valid) / len(valid)
        
        # Calculate standard deviations
        if len(valid) > 1:
            self.std_n = math.sqrt(sum((s.err_n - self.mean_n)**2 for s in valid) / (len(valid) - 1))
            self.std_e = math.sqrt(sum((s.err_e - self.mean_e)**2 for s in valid) / (len(valid) - 1))
            self.std_d = math.sqrt(sum((s.err_d - self.mean_d)**2 for s in valid) / (len(valid) - 1))
            self.std_horiz = math.sqrt(sum((s.err_horiz - self.mean_horiz)**2 for s in valid) / (len(valid) - 1))
            self.std_total = math.sqrt(sum((s.err_total - self.mean_total)**2 for s in valid) / (len(valid) - 1))
        
        # Calculate max (absolute)
        self.max_n = max(abs(s.err_n) for s in valid)
        self.max_e = max(abs(s.err_e) for s in valid)
        self.max_d = max(abs(s.err_d) for s in valid)
        self.max_horiz = max(s.err_horiz for s in valid)
        self.max_total = max(s.err_total for s in valid)
        
        # Calculate RMS
        self.rms_n = math.sqrt(sum(s.err_n**2 for s in valid) / len(valid))
        self.rms_e = math.sqrt(sum(s.err_e**2 for s in valid) / len(valid))
        self.rms_d = math.sqrt(sum(s.err_d**2 for s in valid) / len(valid))
        self.rms_horiz = math.sqrt(sum(s.err_horiz**2 for s in valid) / len(valid))
        self.rms_total = math.sqrt(sum(s.err_total**2 for s in valid) / len(valid))
        
        # Calculate CEP (Circular Error Probable)
        horiz_errors = sorted([s.err_horiz for s in valid])
        idx_50 = int(len(horiz_errors) * 0.50)
        idx_95 = int(len(horiz_errors) * 0.95)
        self.cep50 = horiz_errors[min(idx_50, len(horiz_errors) - 1)]
        self.cep95 = horiz_errors[min(idx_95, len(horiz_errors) - 1)]

    def to_dict(self):
        return {
            'sample_count': self.sample_count,
            'duration': self.duration,
            'mean': {'n': self.mean_n, 'e': self.mean_e, 'd': self.mean_d, 
                     'horiz': self.mean_horiz, 'total': self.mean_total},
            'std': {'n': self.std_n, 'e': self.std_e, 'd': self.std_d,
                    'horiz': self.std_horiz, 'total': self.std_total},
            'max': {'n': self.max_n, 'e': self.max_e, 'd': self.max_d,
                    'horiz': self.max_horiz, 'total': self.max_total},
            'rms': {'n': self.rms_n, 'e': self.rms_e, 'd': self.rms_d,
                    'horiz': self.rms_horiz, 'total': self.rms_total},
            'cep50': self.cep50,
            'cep95': self.cep95
        }

    def print_report(self):
        """Print statistics report"""
        print("\n" + "=" * 60)
        print("UWB FUSION ACCURACY TEST RESULTS")
        print("=" * 60)
        print("Samples: %d  Duration: %.1f s" % (self.sample_count, self.duration))
        print("-" * 60)
        print("%-12s %10s %10s %10s %10s %10s" % ("Metric", "N(mm)", "E(mm)", "D(mm)", "Horiz(mm)", "3D(mm)"))
        print("-" * 60)
        print("%-12s %10.1f %10.1f %10.1f %10.1f %10.1f" % (
            "Mean", self.mean_n*1000, self.mean_e*1000, self.mean_d*1000, 
            self.mean_horiz*1000, self.mean_total*1000))
        print("%-12s %10.1f %10.1f %10.1f %10.1f %10.1f" % (
            "Std Dev", self.std_n*1000, self.std_e*1000, self.std_d*1000,
            self.std_horiz*1000, self.std_total*1000))
        print("%-12s %10.1f %10.1f %10.1f %10.1f %10.1f" % (
            "Max", self.max_n*1000, self.max_e*1000, self.max_d*1000,
            self.max_horiz*1000, self.max_total*1000))
        print("%-12s %10.1f %10.1f %10.1f %10.1f %10.1f" % (
            "RMS", self.rms_n*1000, self.rms_e*1000, self.rms_d*1000,
            self.rms_horiz*1000, self.rms_total*1000))
        print("-" * 60)
        print("CEP50 (50%% within): %.1f mm" % (self.cep50 * 1000))
        print("CEP95 (95%% within): %.1f mm" % (self.cep95 * 1000))
        print("=" * 60)


class UWBTestModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(UWBTestModule, self).__init__(mpstate, "uwbtest", "UWB Fusion Accuracy Test", public=True)

        # Settings
        self.test_settings = mp_settings.MPSettings([
            ('graph_duration', float, 60.0),    # seconds of data to show in graph
            ('sample_rate', float, 10.0),       # Hz for recording samples
        ])
        self.add_command('uwbtest', self.cmd_uwbtest, "UWB fusion accuracy test",
                         ['<status|start|stop|analyze|save|config|gui>'])

        # Test state
        self.testing = False
        self.test_start_time = None
        self.samples = []  # list of TestSample
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        # Current positions (from MAVLink)
        self.ekf_lat = None
        self.ekf_lon = None
        self.ekf_alt = None
        self.rtk_lat = None
        self.rtk_lon = None
        self.rtk_alt = None
        self.rtk_fix_type = 0
        self.gps_satellites = 0

        # EKF parameters (saved before test)
        self.saved_gps_ctrl = None
        self.saved_hgt_ref = None

        # Last sample time
        self.last_sample_time = 0

        # GUI
        self.gui = None
        self.last_gui_update = 0
        
        # Auto-start GUI
        self.cmd_gui()

    def cmd_uwbtest(self, args):
        """Handle uwbtest commands"""
        usage = '''usage: uwbtest <command>
Commands:
    status              - show current test status and positions
    start               - start accuracy test (disables GPS fusion)
    stop                - stop test and restore GPS fusion
    analyze             - analyze test results
    save <filename>     - save test data to file
    config              - configure EKF fusion parameters
    gui                 - open GUI window
    set <param> <val>   - change settings
'''
        if len(args) < 1:
            print(usage)
            return

        cmd = args[0].lower()

        if cmd == 'status':
            self.cmd_status()
        elif cmd == 'start':
            self.cmd_start()
        elif cmd == 'stop':
            self.cmd_stop()
        elif cmd == 'analyze':
            self.cmd_analyze()
        elif cmd == 'save':
            if len(args) < 2:
                print("Usage: uwbtest save <filename>")
                return
            self.cmd_save(args[1])
        elif cmd == 'config':
            self.cmd_config(args[1:])
        elif cmd == 'gui':
            self.cmd_gui()
        elif cmd == 'set':
            self.test_settings.command(args[1:])
        else:
            print(usage)

    def cmd_status(self):
        """Show current status"""
        print("=== UWB Fusion Test Status ===")
        print("Testing: %s" % ("ACTIVE" if self.testing else "Stopped"))
        if self.testing:
            elapsed = time.time() - self.test_start_time
            print("Duration: %.1f s" % elapsed)
            print("Samples: %d" % len(self.samples))

        # EKF position
        print("\nEKF Position (GLOBAL_POSITION_INT):")
        if self.ekf_lat is not None:
            print("  Lat: %.9f°  Lon: %.9f°  Alt: %.3f m" % (
                self.ekf_lat, self.ekf_lon, self.ekf_alt))
        else:
            print("  No data")

        # RTK reference
        fix_names = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 
                     4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
        print("\nRTK Reference (GPS_RAW_INT):")
        if self.rtk_lat is not None:
            print("  Lat: %.9f°  Lon: %.9f°  Alt: %.3f m" % (
                self.rtk_lat, self.rtk_lon, self.rtk_alt))
            print("  Fix: %s  Sats: %d" % (
                fix_names.get(self.rtk_fix_type, "Unknown"), self.gps_satellites))
        else:
            print("  No data")

        # Current error
        if self.ekf_lat is not None and self.rtk_lat is not None:
            if self.origin_lat is None:
                self.origin_lat = self.rtk_lat
                self.origin_lon = self.rtk_lon
                self.origin_alt = self.rtk_alt
            
            ekf_n, ekf_e, ekf_d = geodetic_to_ned(
                self.ekf_lat, self.ekf_lon, self.ekf_alt,
                self.origin_lat, self.origin_lon, self.origin_alt)
            rtk_n, rtk_e, rtk_d = geodetic_to_ned(
                self.rtk_lat, self.rtk_lon, self.rtk_alt,
                self.origin_lat, self.origin_lon, self.origin_alt)
            
            err_n = (ekf_n - rtk_n) * 1000
            err_e = (ekf_e - rtk_e) * 1000
            err_d = (ekf_d - rtk_d) * 1000
            err_horiz = math.sqrt(err_n**2 + err_e**2)
            err_total = math.sqrt(err_n**2 + err_e**2 + err_d**2)
            
            print("\nCurrent Error (EKF - RTK):")
            print("  N: %+.1f mm  E: %+.1f mm  D: %+.1f mm" % (err_n, err_e, err_d))
            print("  Horizontal: %.1f mm  Total: %.1f mm" % (err_horiz, err_total))

        # EKF parameters
        print("\nEKF Fusion Parameters (PX4):")
        gps_ctrl = self.mpstate.mav_param.get('EKF2_GPS_CTRL', -1)
        uwbb_ctrl = self.mpstate.mav_param.get('EKF2_UWBB_CTRL', -1)
        uwbt_ctrl = self.mpstate.mav_param.get('EKF2_UWBT_CTRL', -1)
        hgt_ref = self.mpstate.mav_param.get('EKF2_HGT_REF', -1)
        print("  EKF2_GPS_CTRL: %s (1=lonlat, 2=alt, 4=vel)" % (
            int(gps_ctrl) if gps_ctrl >= 0 else "?"))
        print("  EKF2_UWBB_CTRL: %s (UWB beacon)" % (
            int(uwbb_ctrl) if uwbb_ctrl >= 0 else "?"))
        print("  EKF2_UWBT_CTRL: %s (UWB tag)" % (
            int(uwbt_ctrl) if uwbt_ctrl >= 0 else "?"))
        print("  EKF2_HGT_REF: %s (0=baro, 1=GPS, 2=range, 3=vision)" % (
            int(hgt_ref) if hgt_ref >= 0 else "?"))

    def cmd_start(self):
        """Start accuracy test"""
        if self.testing:
            print("Test already running")
            return

        if self.rtk_fix_type < 5:
            print("WARNING: RTK fix not available (current: %d). Reference may be inaccurate." % 
                  self.rtk_fix_type)

        # Save current parameters
        self.saved_gps_ctrl = self.mpstate.mav_param.get('EKF2_GPS_CTRL', 7)
        self.saved_hgt_ref = self.mpstate.mav_param.get('EKF2_HGT_REF', 1)

        # Disable GPS fusion but keep UWB enabled
        master = self.mpstate.master()
        print("Disabling GPS fusion for EKF...")
        master.mav.param_set_send(
            master.target_system, master.target_component,
            b'EKF2_GPS_CTRL', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )

        # Set origin to current RTK position
        if self.rtk_lat is not None:
            self.origin_lat = self.rtk_lat
            self.origin_lon = self.rtk_lon
            self.origin_alt = self.rtk_alt
            print("Origin set to RTK position")

        # Clear previous samples
        self.samples = []
        self.test_start_time = time.time()
        self.testing = True

        print("Test started - recording UWB vs RTK positions")
        print("Use 'uwbtest stop' to end test")

    def cmd_stop(self):
        """Stop test and restore parameters"""
        if not self.testing:
            print("No test running")
            return

        self.testing = False
        elapsed = time.time() - self.test_start_time

        # Restore GPS fusion
        master = self.mpstate.master()
        if self.saved_gps_ctrl is not None:
            print("Restoring GPS fusion (EKF2_GPS_CTRL = %d)..." % int(self.saved_gps_ctrl))
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_GPS_CTRL', int(self.saved_gps_ctrl),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )

        print("\nTest stopped")
        print("Duration: %.1f s" % elapsed)
        print("Samples: %d" % len(self.samples))

        # Auto-analyze
        if len(self.samples) > 0:
            self.cmd_analyze()

    def cmd_analyze(self):
        """Analyze test results"""
        if len(self.samples) == 0:
            print("No test data to analyze")
            return

        stats = TestStatistics(self.samples)
        stats.print_report()

    def cmd_save(self, filename):
        """Save test data to file"""
        if len(self.samples) == 0:
            print("No test data to save")
            return

        # Calculate statistics
        stats = TestStatistics(self.samples)

        # Determine format from extension
        if filename.endswith('.csv'):
            self._save_csv(filename)
        else:
            if not filename.endswith('.json'):
                filename += '.json'
            self._save_json(filename, stats)

    def _save_json(self, filename, stats):
        """Save as JSON"""
        data = {
            'test_info': {
                'start_time': datetime.fromtimestamp(self.test_start_time).isoformat() if self.test_start_time else None,
                'origin': {
                    'lat': self.origin_lat,
                    'lon': self.origin_lon,
                    'alt': self.origin_alt
                }
            },
            'statistics': stats.to_dict(),
            'samples': [s.to_dict() for s in self.samples]
        }
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print("Saved to %s" % filename)
        except Exception as e:
            print("Error saving: %s" % e)

    def _save_csv(self, filename):
        """Save as CSV"""
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'ekf_lat', 'ekf_lon', 'ekf_alt',
                                'rtk_lat', 'rtk_lon', 'rtk_alt', 'rtk_fix',
                                'err_n_m', 'err_e_m', 'err_d_m', 'err_horiz_m', 'err_total_m'])
                for s in self.samples:
                    writer.writerow([
                        s.timestamp, s.ekf_lat, s.ekf_lon, s.ekf_alt,
                        s.rtk_lat, s.rtk_lon, s.rtk_alt, s.rtk_fix_type,
                        s.err_n, s.err_e, s.err_d, s.err_horiz, s.err_total
                    ])
            print("Saved to %s" % filename)
        except Exception as e:
            print("Error saving: %s" % e)

    def cmd_config(self, args):
        """Configure EKF fusion parameters"""
        usage = '''usage: uwbtest config <param> <value>
Parameters:
    gps <0-7>        - EKF2_GPS_CTRL (0=off, 7=all)
    uwb_beacon <0|1> - EKF2_UWBB_CTRL
    uwb_tag <0|1>    - EKF2_UWBT_CTRL
    height <0-3>     - EKF2_HGT_REF (0=baro, 1=GPS, 2=range, 3=vision)
'''
        if len(args) < 2:
            print(usage)
            return

        param = args[0].lower()
        try:
            value = int(args[1])
        except ValueError:
            print("Invalid value: %s" % args[1])
            return

        master = self.mpstate.master()
        
        if param == 'gps':
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_GPS_CTRL', value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print("EKF2_GPS_CTRL set to %d" % value)
        elif param == 'uwb_beacon':
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_UWBB_CTRL', value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print("EKF2_UWBB_CTRL set to %d" % value)
        elif param == 'uwb_tag':
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_UWBT_CTRL', value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print("EKF2_UWBT_CTRL set to %d" % value)
        elif param == 'height':
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_HGT_REF', value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print("EKF2_HGT_REF set to %d" % value)
        else:
            print(usage)

    def cmd_gui(self):
        """Open GUI window"""
        if not mp_util.has_wxpython:
            print("wxPython not available")
            return

        if self.gui is not None and self.gui.is_alive():
            print("GUI already open")
            return

        from MAVProxy.modules.mavproxy_uwbtest import uwbtest_gui
        self.gui = uwbtest_gui.UWBTestGUI(self)
        print("UWB Test GUI opened")

    def mavlink_packet(self, msg):
        """Handle incoming MAVLink packets"""
        msg_type = msg.get_type()

        if msg_type == 'GLOBAL_POSITION_INT':
            # EKF fused position
            self.ekf_lat = msg.lat * 1.0e-7
            self.ekf_lon = msg.lon * 1.0e-7
            self.ekf_alt = msg.alt * 1.0e-3

        elif msg_type == 'GPS_RAW_INT':
            # Raw RTK GPS position (reference)
            self.rtk_lat = msg.lat * 1.0e-7
            self.rtk_lon = msg.lon * 1.0e-7
            self.rtk_alt = msg.alt * 1.0e-3
            self.rtk_fix_type = msg.fix_type
            self.gps_satellites = msg.satellites_visible

    def idle_task(self):
        """Periodic tasks"""
        now = time.time()

        # Check if GUI was closed - unload module
        if self.gui is not None and not self.gui.is_alive():
            self.gui = None
            self.needs_unloading = True
            return

        # Record samples during test
        if self.testing:
            sample_interval = 1.0 / self.test_settings.sample_rate
            if now - self.last_sample_time >= sample_interval:
                self.last_sample_time = now
                self._record_sample()

        # Update GUI
        if self.gui is not None and self.gui.is_alive():
            if now - self.last_gui_update >= 0.1:  # 10 Hz
                self.last_gui_update = now
                self._update_gui()

    def _record_sample(self):
        """Record a test sample"""
        if self.ekf_lat is None or self.rtk_lat is None:
            return

        sample = TestSample(
            timestamp=time.time(),
            ekf_lat=self.ekf_lat,
            ekf_lon=self.ekf_lon,
            ekf_alt=self.ekf_alt,
            rtk_lat=self.rtk_lat,
            rtk_lon=self.rtk_lon,
            rtk_alt=self.rtk_alt,
            rtk_fix_type=self.rtk_fix_type
        )

        if self.origin_lat is not None:
            sample.calculate_error(self.origin_lat, self.origin_lon, self.origin_alt)

        self.samples.append(sample)

    def _update_gui(self):
        """Send data to GUI"""
        # Current error
        err_n, err_e, err_d, err_horiz, err_total = None, None, None, None, None
        if self.ekf_lat is not None and self.rtk_lat is not None:
            if self.origin_lat is None:
                self.origin_lat = self.rtk_lat
                self.origin_lon = self.rtk_lon
                self.origin_alt = self.rtk_alt
            
            ekf_n, ekf_e, ekf_d = geodetic_to_ned(
                self.ekf_lat, self.ekf_lon, self.ekf_alt,
                self.origin_lat, self.origin_lon, self.origin_alt)
            rtk_n, rtk_e, rtk_d = geodetic_to_ned(
                self.rtk_lat, self.rtk_lon, self.rtk_alt,
                self.origin_lat, self.origin_lon, self.origin_alt)
            
            err_n = ekf_n - rtk_n
            err_e = ekf_e - rtk_e
            err_d = ekf_d - rtk_d
            err_horiz = math.sqrt(err_n**2 + err_e**2)
            err_total = math.sqrt(err_n**2 + err_e**2 + err_d**2)

        # Get recent errors for graph
        graph_samples = []
        graph_duration = self.test_settings.graph_duration
        cutoff_time = time.time() - graph_duration
        for s in self.samples:
            if s.timestamp >= cutoff_time and s.err_n is not None:
                graph_samples.append({
                    'time': s.timestamp - self.test_start_time if self.test_start_time else 0,
                    'err_n': s.err_n * 1000,  # mm
                    'err_e': s.err_e * 1000,
                    'err_d': s.err_d * 1000,
                    'err_horiz': s.err_horiz * 1000,
                    'err_total': s.err_total * 1000
                })

        # EKF parameters
        params = {
            'gps_ctrl': self.mpstate.mav_param.get('EKF2_GPS_CTRL', -1),
            'uwbb_ctrl': self.mpstate.mav_param.get('EKF2_UWBB_CTRL', -1),
            'uwbt_ctrl': self.mpstate.mav_param.get('EKF2_UWBT_CTRL', -1),
            'hgt_ref': self.mpstate.mav_param.get('EKF2_HGT_REF', -1)
        }

        self.gui.update_data({
            'testing': self.testing,
            'elapsed': time.time() - self.test_start_time if self.test_start_time else 0,
            'sample_count': len(self.samples),
            'rtk_fix': self.rtk_fix_type,
            'rtk_sats': self.gps_satellites,
            'err_n_mm': err_n * 1000 if err_n is not None else None,
            'err_e_mm': err_e * 1000 if err_e is not None else None,
            'err_d_mm': err_d * 1000 if err_d is not None else None,
            'err_horiz_mm': err_horiz * 1000 if err_horiz is not None else None,
            'err_total_mm': err_total * 1000 if err_total is not None else None,
            'graph_samples': graph_samples,
            'params': params
        })

    def unload(self):
        """Module unload"""
        # Restore GPS if test was running
        if self.testing and self.saved_gps_ctrl is not None:
            master = self.mpstate.master()
            master.mav.param_set_send(
                master.target_system, master.target_component,
                b'EKF2_GPS_CTRL', int(self.saved_gps_ctrl),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
        if self.gui is not None:
            self.gui.close()


def init(mpstate):
    """Initialize module"""
    return UWBTestModule(mpstate)
