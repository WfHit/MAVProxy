"""
  MAVProxy UWB Anchor Deploy Module
  
  For recording and managing UWB anchor positions with high precision.
  Displays both WGS84 (9+ decimals) and NED coordinates (mm precision).
"""

import os
import sys
import math
import time
import json
from datetime import datetime

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
from pymavlink import mavutil

# WGS84 ellipsoid parameters
WGS84_A = 6378137.0  # semi-major axis (m)
WGS84_E2 = 0.00669437999014  # first eccentricity squared


class Anchor:
    """Represents a single UWB anchor point"""
    def __init__(self, anchor_id, name, lat, lon, alt, n, e, d, timestamp, samples):
        self.id = anchor_id
        self.name = name
        self.lat = lat  # degrees (9+ decimals)
        self.lon = lon  # degrees (9+ decimals)
        self.alt = alt  # meters AMSL
        self.n = n      # meters (mm precision)
        self.e = e      # meters (mm precision)
        self.d = d      # meters (mm precision)
        self.timestamp = timestamp
        self.samples = samples  # number of samples averaged

    def to_dict(self):
        return {
            'id': self.id,
            'name': self.name,
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'n': self.n,
            'e': self.e,
            'd': self.d,
            'timestamp': self.timestamp,
            'samples': self.samples
        }

    @staticmethod
    def from_dict(d):
        return Anchor(
            d['id'], d['name'], d['lat'], d['lon'], d['alt'],
            d['n'], d['e'], d['d'], d['timestamp'], d['samples']
        )


class Origin:
    """NED origin point"""
    def __init__(self, lat=None, lon=None, alt=None, timestamp=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.timestamp = timestamp

    def is_set(self):
        return self.lat is not None and self.lon is not None and self.alt is not None

    def to_dict(self):
        return {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'timestamp': self.timestamp
        }

    @staticmethod
    def from_dict(d):
        return Origin(d.get('lat'), d.get('lon'), d.get('alt'), d.get('timestamp'))


class Backsight:
    """Backsight reference point defined by direction and distance from origin"""

    # Direction to NED offset factors (N, E)
    DIRECTIONS = {
        'N': (1, 0),   # North: +N, 0E
        'E': (0, 1),   # East: 0N, +E
        'S': (-1, 0),  # South: -N, 0E
        'W': (0, -1)   # West: 0N, -E
    }

    def __init__(self, direction=None, distance=None, threshold_mm=50):
        self.direction = direction  # 'N', 'E', 'S', 'W'
        self.distance = distance    # meters
        self.threshold_mm = threshold_mm  # highlight threshold in mm

    def is_set(self):
        return self.direction is not None and self.distance is not None

    def get_ned(self):
        """Get NED position of backsight point (D is always 0)"""
        if not self.is_set():
            return None, None, None
        n_factor, e_factor = self.DIRECTIONS[self.direction]
        n = n_factor * self.distance
        e = e_factor * self.distance
        d = 0.0
        return n, e, d

    def get_error_mm(self, current_n, current_e, current_d):
        """Get error between current position and backsight in mm (N, E, D, total)"""
        if not self.is_set() or current_n is None:
            return None, None, None, None
        bs_n, bs_e, bs_d = self.get_ned()
        err_n = (current_n - bs_n) * 1000  # mm
        err_e = (current_e - bs_e) * 1000  # mm
        err_d = (current_d - bs_d) * 1000  # mm
        err_total = math.sqrt(err_n**2 + err_e**2 + err_d**2)
        return err_n, err_e, err_d, err_total

    def is_within_threshold(self, current_n, current_e, current_d):
        """Check if current position is within threshold of backsight"""
        err_n, err_e, err_d, err_total = self.get_error_mm(current_n, current_e, current_d)
        if err_total is None:
            return False
        return err_total <= self.threshold_mm

    def to_dict(self):
        return {
            'direction': self.direction,
            'distance': self.distance,
            'threshold_mm': self.threshold_mm
        }

    @staticmethod
    def from_dict(d):
        return Backsight(
            direction=d.get('direction'),
            distance=d.get('distance'),
            threshold_mm=d.get('threshold_mm', 50)
        )


class PositionAverager:
    """Averages position samples over time"""
    def __init__(self, duration_seconds=5):
        self.duration = duration_seconds
        self.samples = []  # list of (lat, lon, alt, timestamp)
        self.start_time = None
        self.active = False

    def start(self):
        self.samples = []
        self.start_time = time.time()
        self.active = True

    def add_sample(self, lat, lon, alt):
        if not self.active:
            return
        self.samples.append((lat, lon, alt, time.time()))

    def is_complete(self):
        if not self.active:
            return False
        return time.time() - self.start_time >= self.duration

    def get_progress(self):
        if not self.active:
            return 0.0
        return min(1.0, (time.time() - self.start_time) / self.duration)

    def get_average(self):
        if len(self.samples) == 0:
            return None, None, None, 0
        lat_sum = sum(s[0] for s in self.samples)
        lon_sum = sum(s[1] for s in self.samples)
        alt_sum = sum(s[2] for s in self.samples)
        n = len(self.samples)
        return lat_sum / n, lon_sum / n, alt_sum / n, n

    def stop(self):
        self.active = False


def geodetic_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    """
    Convert WGS84 geodetic coordinates to NED relative to origin.
    Uses accurate geodetic calculations for mm-level precision.
    """
    # Convert to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    # Radius of curvature in prime vertical
    sin_lat = math.sin(origin_lat_rad)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)

    # Differences
    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad
    dalt = alt - origin_alt

    # NED coordinates (linear approximation valid for small distances)
    # For higher precision over longer distances, use full ECEF conversion
    M = WGS84_A * (1 - WGS84_E2) / pow(1 - WGS84_E2 * sin_lat * sin_lat, 1.5)

    north = dlat * M
    east = dlon * N * math.cos(origin_lat_rad)
    down = -dalt

    return north, east, down


def ned_to_geodetic(n, e, d, origin_lat, origin_lon, origin_alt):
    """
    Convert NED coordinates back to WGS84 geodetic.
    """
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    sin_lat = math.sin(origin_lat_rad)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    M = WGS84_A * (1 - WGS84_E2) / pow(1 - WGS84_E2 * sin_lat * sin_lat, 1.5)

    dlat = n / M
    dlon = e / (N * math.cos(origin_lat_rad))
    dalt = -d

    lat = math.degrees(origin_lat_rad + dlat)
    lon = math.degrees(origin_lon_rad + dlon)
    alt = origin_alt + dalt

    return lat, lon, alt


class UWBAnchorModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(UWBAnchorModule, self).__init__(mpstate, "uwb_anchor", "UWB Anchor Deploy", public=True)

        # Settings
        self.uwb_settings = mp_settings.MPSettings([
            ('average_time', float, 5.0),      # seconds to average position
            ('update_rate', float, 5.0),       # Hz for GUI update
        ])
        self.add_command('uwbanchor', self.cmd_uwbanchor, "UWB anchor management",
                         ['<status|origin|add|delete|list|save|load|clear|gui|backsight|rtk>'])
        self.add_completion_function('(UWBSETTING)', self.uwb_settings.completion)

        # State
        self.origin = Origin()
        self.backsight = Backsight()
        self.anchors = []  # list of Anchor
        self.next_anchor_id = 1
        self.averager = PositionAverager()
        self.pending_anchor_name = None

        # Current position (updated from MAVLink)
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.last_pos_time = None

        # RTK and EKF status
        self.gps_fix_type = 0
        self.gps_satellites = 0
        self.gps_hdop = 0
        self.ekf_flags = 0
        self.ekf_velocity_variance = 0
        self.ekf_pos_horiz_variance = 0
        self.ekf_pos_vert_variance = 0

        # GUI
        self.gui = None
        self.last_gui_update = 0
        
        # Auto-start GUI
        self.cmd_gui()

    def cmd_uwbanchor(self, args):
        """Handle uwbanchor commands"""
        usage = '''usage: uwbanchor <command>
Commands:
    status              - show current position, RTK and EKF status
    origin              - set current position as NED origin
    add [name]          - add anchor at current position (with averaging)
    delete <id|name>    - delete an anchor
    list                - list all anchors
    save <filename>     - save anchors and origin to JSON file
    load <filename>     - load anchors and origin from JSON file
    clear               - clear all anchors
    gui                 - open GUI window
    backsight <dir> <dist> [threshold_mm] - set backsight point (dir=N/E/S/W, dist in m)
    rtk <enable|disable> - enable/disable RTK for EKF fusion
    set <setting> <val> - change settings
'''
        if len(args) < 1:
            print(usage)
            return

        cmd = args[0].lower()

        if cmd == 'status':
            self.cmd_status()
        elif cmd == 'origin':
            self.cmd_set_origin()
        elif cmd == 'add':
            name = args[1] if len(args) > 1 else None
            self.cmd_add_anchor(name)
        elif cmd == 'delete':
            if len(args) < 2:
                print("Usage: uwb delete <id|name>")
                return
            self.cmd_delete_anchor(args[1])
        elif cmd == 'list':
            self.cmd_list_anchors()
        elif cmd == 'save':
            if len(args) < 2:
                print("Usage: uwb save <filename>")
                return
            self.cmd_save(args[1])
        elif cmd == 'load':
            if len(args) < 2:
                print("Usage: uwb load <filename>")
                return
            self.cmd_load(args[1])
        elif cmd == 'clear':
            self.cmd_clear()
        elif cmd == 'gui':
            self.cmd_gui()
        elif cmd == 'backsight':
            self.cmd_backsight(args[1:])
        elif cmd == 'rtk':
            self.cmd_rtk(args[1:])
        elif cmd == 'set':
            self.uwb_settings.command(args[1:])
        else:
            print(usage)

    def cmd_status(self):
        """Show current status"""
        print("=== UWB Anchor Status ===")
        if self.current_lat is not None:
            print("Current Position (WGS84):")
            print("  Lat: %.9f°" % self.current_lat)
            print("  Lon: %.9f°" % self.current_lon)
            print("  Alt: %.3f m (AMSL)" % self.current_alt)

            if self.origin.is_set():
                n, e, d = geodetic_to_ned(
                    self.current_lat, self.current_lon, self.current_alt,
                    self.origin.lat, self.origin.lon, self.origin.alt
                )
                print("Current Position (NED):")
                print("  N: %.3f m" % n)
                print("  E: %.3f m" % e)
                print("  D: %.3f m" % d)
        else:
            print("No position available")

        if self.origin.is_set():
            print("\nOrigin (WGS84):")
            print("  Lat: %.9f°" % self.origin.lat)
            print("  Lon: %.9f°" % self.origin.lon)
            print("  Alt: %.3f m (AMSL)" % self.origin.alt)
            print("  Set: %s" % self.origin.timestamp)
        else:
            print("\nOrigin: Not set")

        # Backsight info
        if self.backsight.is_set():
            bs_n, bs_e, bs_d = self.backsight.get_ned()
            print("\nBacksight: %s %.3f m (N=%.3f m, E=%.3f m)" % (
                self.backsight.direction, self.backsight.distance, bs_n, bs_e))
            print("  Threshold: %d mm" % self.backsight.threshold_mm)
            if self.origin.is_set() and self.current_lat is not None:
                n, e, d = geodetic_to_ned(
                    self.current_lat, self.current_lon, self.current_alt,
                    self.origin.lat, self.origin.lon, self.origin.alt
                )
                err_n, err_e, err_d, err_total = self.backsight.get_error_mm(n, e, d)
                print("  Error: N=%+.0f mm, E=%+.0f mm, D=%+.0f mm, Total=%.0f mm" % (
                    err_n, err_e, err_d, err_total))
                if self.backsight.is_within_threshold(n, e, d):
                    print("  *** POSITION MATCH ***")
        else:
            print("\nBacksight: Not set")

        # RTK and EKF status
        print("\n=== RTK / EKF Status ===")
        fix_types = {
            0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
        }
        fix_name = fix_types.get(self.gps_fix_type, "Unknown (%d)" % self.gps_fix_type)
        print("GPS Fix: %s" % fix_name)
        print("  Satellites: %d" % self.gps_satellites)
        print("  HDOP: %.2f" % self.gps_hdop)

        # EKF flags interpretation
        ekf_flag_names = [
            (0x01, "ATTITUDE"),
            (0x02, "VELOCITY_HORIZ"),
            (0x04, "VELOCITY_VERT"),
            (0x08, "POS_HORIZ_REL"),
            (0x10, "POS_HORIZ_ABS"),
            (0x20, "POS_VERT_ABS"),
            (0x40, "POS_VERT_AGL"),
            (0x80, "CONST_POS_MODE"),
            (0x100, "PRED_POS_HORIZ_REL"),
            (0x200, "PRED_POS_HORIZ_ABS"),
            (0x400, "GPS_GLITCHING"),
        ]
        active_flags = [name for flag, name in ekf_flag_names if self.ekf_flags & flag]
        print("\nEKF Status: 0x%04X" % self.ekf_flags)
        print("  Active: %s" % ", ".join(active_flags) if active_flags else "  Active: None")
        print("  Velocity Var: %.4f" % self.ekf_velocity_variance)
        print("  Pos Horiz Var: %.4f" % self.ekf_pos_horiz_variance)
        print("  Pos Vert Var: %.4f" % self.ekf_pos_vert_variance)

        # RTK enabled status from parameter
        rtk_enabled = self._get_rtk_enabled()
        gps_ctrl = self.mpstate.mav_param.get('EKF2_GPS_CTRL', -1)
        hgt_ref = self.mpstate.mav_param.get('EKF2_HGT_REF', -1)
        print("\nGPS/RTK Fusion (PX4):")
        print("  EKF2_GPS_CTRL: %s (1=lonlat, 2=alt, 4=vel)" % (int(gps_ctrl) if gps_ctrl >= 0 else "Unknown"))
        print("  EKF2_HGT_REF: %s (0=baro, 1=GPS, 2=range, 3=vision)" % (int(hgt_ref) if hgt_ref >= 0 else "Unknown"))
        print("  RTK Active: %s (GPS enabled + RTK fix)" % ("Yes" if rtk_enabled else "No"))

        print("\nAnchors: %d" % len(self.anchors))

    def cmd_backsight(self, args):
        """Set or clear backsight point"""
        if len(args) == 0 or args[0].lower() == 'clear':
            self.backsight = Backsight()
            print("Backsight cleared")
            return

        if len(args) < 2:
            print("Usage: uwb backsight <N|E|S|W> <distance_m> [threshold_mm]")
            print("       uwb backsight clear")
            return

        direction = args[0].upper()
        if direction not in Backsight.DIRECTIONS:
            print("Invalid direction: %s (use N, E, S, or W)" % args[0])
            return

        try:
            distance = float(args[1])
        except ValueError:
            print("Invalid distance: %s" % args[1])
            return

        threshold_mm = 50
        if len(args) >= 3:
            try:
                threshold_mm = int(args[2])
            except ValueError:
                print("Invalid threshold: %s" % args[2])
                return

        self.backsight = Backsight(direction, distance, threshold_mm)
        bs_n, bs_e, bs_d = self.backsight.get_ned()
        print("Backsight set: %s %.3f m" % (direction, distance))
        print("  NED position: N=%.3f m, E=%.3f m, D=%.3f m" % (bs_n, bs_e, bs_d))
        print("  Match threshold: %d mm" % threshold_mm)

    def cmd_rtk(self, args):
        """Enable or disable GPS/RTK for EKF fusion"""
        if len(args) == 0:
            rtk_enabled = self._get_rtk_enabled()
            gps_ctrl = self.mpstate.mav_param.get('EKF2_GPS_CTRL', -1)
            hgt_ref = self.mpstate.mav_param.get('EKF2_HGT_REF', -1)
            print("GPS/RTK Status (PX4):")
            print("  EKF2_GPS_CTRL: %s (bitmask: 1=lonlat, 2=alt, 4=vel)" % (
                int(gps_ctrl) if gps_ctrl >= 0 else "Unknown"))
            print("  EKF2_HGT_REF: %s (0=baro, 1=GPS, 2=range, 3=vision)" % (
                int(hgt_ref) if hgt_ref >= 0 else "Unknown"))
            print("  GPS Fix Type: %d (%s)" % (
                self.gps_fix_type,
                {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}.get(
                    self.gps_fix_type, "Unknown")))
            print("  RTK Active: %s" % ("Yes" if rtk_enabled else "No"))
            print("\nUsage: uwbanchor rtk <enable|disable>")
            return

        action = args[0].lower()
        if action == 'enable':
            self._enable_rtk(True)
        elif action == 'disable':
            self._enable_rtk(False)
        else:
            print("Usage: uwbanchor rtk <enable|disable>")

    def _get_rtk_enabled(self):
        """Check if RTK/GPS is enabled for EKF fusion (PX4)"""
        # PX4 uses EKF2_GPS_CTRL bitmask:
        #   bit 0 (1): lon/lat
        #   bit 1 (2): altitude  
        #   bit 2 (4): velocity
        # RTK is enabled if GPS is being used and we have RTK fix
        try:
            gps_ctrl = self.mpstate.mav_param.get('EKF2_GPS_CTRL', 0)
            # GPS fusion enabled if any GPS bits are set
            gps_enabled = int(gps_ctrl) > 0
            # RTK is "active" if GPS enabled AND we have RTK fix type (5=float, 6=fixed)
            return gps_enabled and self.gps_fix_type >= 5
        except Exception:
            # If can't read param, infer from GPS fix type
            return self.gps_fix_type >= 5  # RTK Float (5) or RTK Fixed (6)

    def _enable_rtk(self, enable):
        """Enable or disable GPS for EKF fusion (PX4)"""
        # PX4 uses EKF2_GPS_CTRL bitmask to control GPS fusion
        # 7 = all GPS fusion enabled (lon/lat + alt + vel)
        # 0 = GPS fusion disabled
        
        master = self.mpstate.master()

        if enable:
            print("Enabling GPS/RTK for EKF fusion...")

            # Enable full GPS fusion: lon/lat(1) + alt(2) + vel(4) = 7
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                b'EKF2_GPS_CTRL',
                7,  # Enable all GPS fusion
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )

            print("GPS/RTK enabled - EKF2_GPS_CTRL set to 7")
            print("Note: RTK fix depends on GPS receiver and corrections")

        else:
            print("Disabling GPS for EKF fusion...")

            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                b'EKF2_GPS_CTRL',
                0,  # Disable GPS fusion
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )

            print("GPS disabled - EKF2_GPS_CTRL set to 0")

    def cmd_set_origin(self):
        """Set current position as NED origin"""
        if self.current_lat is None:
            print("No position available")
            return

        self.origin = Origin(
            lat=self.current_lat,
            lon=self.current_lon,
            alt=self.current_alt,
            timestamp=datetime.now().isoformat()
        )
        print("Origin set:")
        print("  Lat: %.9f°" % self.origin.lat)
        print("  Lon: %.9f°" % self.origin.lon)
        print("  Alt: %.3f m (AMSL)" % self.origin.alt)

        # Recalculate NED for all existing anchors
        self._recalculate_anchor_ned()

    def cmd_add_anchor(self, name=None):
        """Start adding an anchor with position averaging"""
        if self.current_lat is None:
            print("No position available")
            return

        if not self.origin.is_set():
            print("Origin not set. Use 'uwb origin' first.")
            return

        if self.averager.active:
            print("Already recording anchor. Please wait.")
            return

        if name is None:
            name = "A%03d" % self.next_anchor_id

        self.pending_anchor_name = name
        self.averager = PositionAverager(self.uwb_settings.average_time)
        self.averager.start()
        print("Recording anchor '%s' for %.1f seconds..." % (name, self.uwb_settings.average_time))

    def _finish_add_anchor(self):
        """Complete adding an anchor after averaging"""
        lat, lon, alt, samples = self.averager.get_average()
        self.averager.stop()

        if lat is None:
            print("No samples collected")
            return

        n, e, d = geodetic_to_ned(lat, lon, alt, self.origin.lat, self.origin.lon, self.origin.alt)

        anchor = Anchor(
            anchor_id=self.next_anchor_id,
            name=self.pending_anchor_name,
            lat=lat,
            lon=lon,
            alt=alt,
            n=n,
            e=e,
            d=d,
            timestamp=datetime.now().isoformat(),
            samples=samples
        )
        self.anchors.append(anchor)
        self.next_anchor_id += 1

        print("Anchor '%s' added (ID=%d, %d samples):" % (anchor.name, anchor.id, samples))
        print("  WGS84: %.9f°, %.9f°, %.3f m" % (lat, lon, alt))
        print("  NED:   N=%.3f m, E=%.3f m, D=%.3f m" % (n, e, d))

        self._update_gui_anchors()

    def cmd_delete_anchor(self, identifier):
        """Delete an anchor by ID or name"""
        try:
            anchor_id = int(identifier)
            original_len = len(self.anchors)
            self.anchors = [a for a in self.anchors if a.id != anchor_id]
            if len(self.anchors) < original_len:
                print("Deleted anchor ID %d" % anchor_id)
            else:
                print("Anchor ID %d not found" % anchor_id)
        except ValueError:
            original_len = len(self.anchors)
            self.anchors = [a for a in self.anchors if a.name != identifier]
            if len(self.anchors) < original_len:
                print("Deleted anchor '%s'" % identifier)
            else:
                print("Anchor '%s' not found" % identifier)

        self._update_gui_anchors()

    def cmd_list_anchors(self):
        """List all anchors"""
        if len(self.anchors) == 0:
            print("No anchors")
            return

        print("=== Anchors (%d) ===" % len(self.anchors))
        print("%-4s %-10s %-14s %-15s %-12s %-10s %-10s %-10s %-10s" % (
            "ID", "Name", "Lat", "Lon", "Alt(m)", "N(m)", "E(m)", "D(m)", "Dist(m)"))
        print("-" * 105)

        for a in self.anchors:
            dist = math.sqrt(a.n * a.n + a.e * a.e + a.d * a.d)
            print("%-4d %-10s %-14.9f %-15.9f %-12.3f %-10.3f %-10.3f %-10.3f %-10.3f" % (
                a.id, a.name, a.lat, a.lon, a.alt, a.n, a.e, a.d, dist))

    def cmd_save(self, filename):
        """Save anchors and origin to JSON file"""
        if not filename.endswith('.json'):
            filename += '.json'

        data = {
            'origin': self.origin.to_dict() if self.origin.is_set() else None,
            'anchors': [a.to_dict() for a in self.anchors],
            'saved_at': datetime.now().isoformat()
        }

        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print("Saved %d anchors to %s" % (len(self.anchors), filename))
        except Exception as e:
            print("Error saving: %s" % e)

    def cmd_load(self, filename):
        """Load anchors and origin from JSON file"""
        if not filename.endswith('.json'):
            filename += '.json'

        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            if data.get('origin'):
                self.origin = Origin.from_dict(data['origin'])
                print("Loaded origin")

            self.anchors = [Anchor.from_dict(a) for a in data.get('anchors', [])]
            if self.anchors:
                self.next_anchor_id = max(a.id for a in self.anchors) + 1

            print("Loaded %d anchors from %s" % (len(self.anchors), filename))
            self._update_gui_anchors()

        except Exception as e:
            print("Error loading: %s" % e)

    def cmd_clear(self):
        """Clear all anchors"""
        count = len(self.anchors)
        self.anchors = []
        self.next_anchor_id = 1
        print("Cleared %d anchors" % count)
        self._update_gui_anchors()

    def cmd_gui(self):
        """Open GUI window"""
        if not mp_util.has_wxpython:
            print("wxPython not available")
            return

        if self.gui is not None and self.gui.is_alive():
            print("GUI already open")
            return

        from MAVProxy.modules.mavproxy_uwbanchor import uwb_gui
        self.gui = uwb_gui.UWBAnchorGUI(self)
        print("UWB Anchor GUI opened")

    def _recalculate_anchor_ned(self):
        """Recalculate NED coordinates for all anchors after origin change"""
        if not self.origin.is_set():
            return

        for a in self.anchors:
            a.n, a.e, a.d = geodetic_to_ned(
                a.lat, a.lon, a.alt,
                self.origin.lat, self.origin.lon, self.origin.alt
            )

    def _update_gui_anchors(self):
        """Notify GUI of anchor list change"""
        if self.gui is not None and self.gui.is_alive():
            self.gui.update_anchors()

    def mavlink_packet(self, msg):
        """Handle incoming MAVLink packets"""
        msg_type = msg.get_type()

        if msg_type == 'GLOBAL_POSITION_INT':
            self.current_lat = msg.lat * 1.0e-7
            self.current_lon = msg.lon * 1.0e-7
            self.current_alt = msg.alt * 1.0e-3  # AMSL
            self.last_pos_time = time.time()

            # Add sample if averaging
            if self.averager.active:
                self.averager.add_sample(self.current_lat, self.current_lon, self.current_alt)

        elif msg_type == 'GPS_RAW_INT':
            self.gps_fix_type = msg.fix_type
            self.gps_satellites = msg.satellites_visible
            self.gps_hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.99

        elif msg_type == 'EKF_STATUS_REPORT':
            self.ekf_flags = msg.flags
            self.ekf_velocity_variance = msg.velocity_variance
            self.ekf_pos_horiz_variance = msg.pos_horiz_variance
            self.ekf_pos_vert_variance = msg.pos_vert_variance

    def idle_task(self):
        """Periodic tasks"""
        now = time.time()

        # Check if averaging is complete
        if self.averager.active and self.averager.is_complete():
            self._finish_add_anchor()

        # Check if GUI was closed - unload module
        if self.gui is not None and not self.gui.is_alive():
            self.gui = None
            self.needs_unloading = True
            return

        # Update GUI
        if self.gui is not None and self.gui.is_alive():
            update_interval = 1.0 / self.uwb_settings.update_rate
            if now - self.last_gui_update >= update_interval:
                self.last_gui_update = now

                # Calculate current NED if origin is set
                current_n, current_e, current_d = None, None, None
                if self.origin.is_set() and self.current_lat is not None:
                    current_n, current_e, current_d = geodetic_to_ned(
                        self.current_lat, self.current_lon, self.current_alt,
                        self.origin.lat, self.origin.lon, self.origin.alt
                    )

                # Calculate backsight error if set
                backsight_data = None
                if self.backsight.is_set():
                    bs_n, bs_e, bs_d = self.backsight.get_ned()
                    err_n, err_e, err_d, err_total = self.backsight.get_error_mm(
                        current_n, current_e, current_d)
                    is_match = self.backsight.is_within_threshold(
                        current_n, current_e, current_d)
                    backsight_data = {
                        'direction': self.backsight.direction,
                        'distance': self.backsight.distance,
                        'threshold_mm': self.backsight.threshold_mm,
                        'bs_n': bs_n,
                        'bs_e': bs_e,
                        'bs_d': bs_d,
                        'err_n_mm': err_n,
                        'err_e_mm': err_e,
                        'err_d_mm': err_d,
                        'err_total_mm': err_total,
                        'is_match': is_match
                    }

                # RTK and EKF status data
                rtk_data = {
                    'fix_type': self.gps_fix_type,
                    'satellites': self.gps_satellites,
                    'hdop': self.gps_hdop,
                    'ekf_flags': self.ekf_flags,
                    'velocity_var': self.ekf_velocity_variance,
                    'pos_horiz_var': self.ekf_pos_horiz_variance,
                    'pos_vert_var': self.ekf_pos_vert_variance,
                    'rtk_enabled': self._get_rtk_enabled(),
                    'gps_ctrl': self.mpstate.mav_param.get('EKF2_GPS_CTRL', -1)
                }

                self.gui.update_position(
                    self.current_lat, self.current_lon, self.current_alt,
                    current_n, current_e, current_d,
                    self.averager.get_progress() if self.averager.active else None,
                    backsight_data,
                    rtk_data
                )

    def unload(self):
        """Module unload"""
        if self.gui is not None:
            self.gui.close()


def init(mpstate):
    """Initialize module"""
    return UWBAnchorModule(mpstate)
