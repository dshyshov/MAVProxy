"""
 Video Downlink with additional data and using OpenCV
"""

import os, sys, math, time
import cv2
import pingo
from pymavlink.rotmat import Vector3, Matrix3, Plane, Line
from math import radians
from pymavlink.wgstosk import WGSPoint

#from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_video import Value

from MAVProxy.modules.lib.mp_menu import *


class VideoCV(mp_module.MPModule):
    def __init__(self, mpstate):
        super(VideoCV, self).__init__(mpstate, "Video", "Video with HUD", public=True)
        self.in_air = False
        self.start_time = 0.0
        self.total_time = 0.0
        self.speed = 0
        self.max_link_num = 0
        self.view_lat = None
        self.view_lon = None
        self.sk_lat = None
        self.sk_lon = None
        # setup some default status information

        self.gps_status = Value('GPS', 'GPS: --', fg='red')

        self.radio_status = Value('Radio', 'Radio: --', fg='gray')
        self.ins_status = Value('INS', 'INS', fg='grey')
        self.mag_status = Value('MAG', 'MAG', fg='grey')
        self.as_status = Value('AS', 'AS', fg='grey')
        self.rng_status = Value('RNG', 'RNG', fg='grey')
        self.ahrs_status = Value('AHRS', 'AHRS', fg='grey')
        self.terr_status = Value('TERR', 'TERR', fg='grey')
        self.alt = Value('Alt', 'Alt ---')
        self.uav_mode = Value('UNKNOWN', 'UNKNOWN', fg='grey')
        self.airspeed = Value('AirSpeed', 'AirSpeed --', fg='white' )
        self.gpsspeed = Value('GPSSpeed', 'GPSSpeed --', fg='white')



        self.wind = Value('Wind', 'Wind ---/---', fg='white')
        self.wp = Value('WP', 'WP --', fg='white')
        self.wpdist = Value('WPDist', 'Distance ---', fg='white')
        self.wpbearing = Value('WPBearing', 'Bearing ---', fg='white')
        self.alterror = Value('AltError', 'AltError --', fg='white')
        self.aspderror = Value('AspdError', 'AspdError --', fg='white')
        self.flighttime = Value('FlightTime', 'FlightTime --', fg='white')
        self.etr = Value('ETR', 'ETR --', fg='white')
        self.homedist = Value('HomeDist', 'Home_Distance ---', fg='white')

        self.target_position_lat = Value('Target_lat', 'LAT -----', fg='white')
        self.target_position_lon = Value('Target lon', 'LON ------', fg='white')
        self.target_positionsk_lat = Value('TargetSK_lat', 'LAT -----', fg='white')
        self.target_positionsk_lon = Value('TargetSK_lon', 'LON ------', fg='white')

    def estimated_time_remaining(self, lat, lon, wpnum, speed):
        '''estimate time remaining in mission in seconds'''
        idx = wpnum
        if wpnum >= self.module('wp').wploader.count():
            return 0
        distance = 0
        done = set()
        while idx < self.module('wp').wploader.count():
            if idx in done:
                break
            done.add(idx)
            w = self.module('wp').wploader.wp(idx)
            if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
                idx = int(w.param1)
                continue
            idx += 1
            if (w.x != 0 or w.y != 0) and w.command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                                                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]:
                distance += mp_util.gps_distance(lat, lon, w.x, w.y)
                lat = w.x
                lon = w.y
                if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    break
        return distance / speed



    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''

        type = msg.get_type()

        master = self.master
        # add some status fields
        if type == 'GPS_RAW_INT':
            if self.module('wp').wploader.count() > 0:
                wp = self.module('wp').wploader.wp(0)
                home_lat = wp.x
                home_lng = wp.y
            else:
                home_lat = master.field('HOME', 'lat') * 1.0e-7
                home_lng = master.field('HOME', 'lon') * 1.0e-7
                curr_lat = msg.lat * 1.0e-7
                curr_lon = msg.lon * 1.0e-7
                home_dist = mp_util.gps_distance(curr_lat, curr_lon, home_lat, home_lng)
                self.homedist.write('HomeDist', 'HomeDist %u' % home_dist)

        if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
            if type == "GPS_RAW":
                num_sats1 = master.field('GPS_STATUS', 'satellites_visible', 0)
            else:
                num_sats1 = msg.satellites_visible
            num_sats2 = master.field('GPS2_RAW', 'satellites_visible', -1)
            if num_sats2 == -1:
                sats_string = "%u" % num_sats1
            else:
                sats_string = "%u/%u" % (num_sats1, num_sats2)
            if ((msg.fix_type == 3 and master.mavlink10()) or
                (msg.fix_type == 2 and not master.mavlink10())):
                self.gps_status.write('GPS', 'GPS: OK (%s)' % sats_string, fg='darkgreen')
            else:
                self.gps_status.write('GPS', 'GPS: %u (%s)' % (msg.fix_type, sats_string), fg='red')





        elif type == 'VFR_HUD':
            if master.mavlink10():
                alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
            else:
                alt = master.field('GPS_RAW', 'alt', 0)
            if self.module('wp').wploader.count() > 0:
                wp = self.module('wp').wploader.wp(0)
                home_lat = wp.x
                home_lng = wp.y
            else:
                home_lat = master.field('HOME', 'lat') * 1.0e-7
                home_lng = master.field('HOME', 'lon') * 1.0e-7
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
            lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
            rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
            agl_alt = None
            if self.settings.basealt != 0:
                agl_alt = self.console.ElevationMap.GetElevation(lat, lng)
                if agl_alt is not None:
                    agl_alt = self.settings.basealt - agl_alt
            else:
                agl_alt_home = self.console.ElevationMap.GetElevation(home_lat, home_lng)
                if agl_alt_home is not None:
                    agl_alt = self.console.ElevationMap.GetElevation(lat, lng)
                if agl_alt is not None:
                    agl_alt = agl_alt_home - agl_alt
            if agl_alt is not None:
                agl_alt += rel_alt
                vehicle_agl = master.field('TERRAIN_REPORT', 'current_height', None)
                if vehicle_agl is None:
                    vehicle_agl = '---'
                else:
                    vehicle_agl = int(vehicle_agl)
#                self.VideoCV.set_status('AGL', 'AGL %u/%s' % (agl_alt, vehicle_agl))
            self.alt.write('Alt', 'Alt %u' % rel_alt)
            self.airspeed.write('AirSpeed', 'AirSpeed %u' % msg.airspeed)
            self.gpsspeed.write('GPSSpeed', 'GPSSpeed %u' % msg.groundspeed)
            t = time.localtime(msg._timestamp)
            if msg.groundspeed > 3 and not self.in_air:
                self.in_air = True
                self.start_time = time.mktime(t)
            elif msg.groundspeed > 3 and self.in_air:
                self.total_time = time.mktime(t) - self.start_time
                self.flighttime.write('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
            elif msg.groundspeed < 3 and self.in_air:
                self.in_air = False
                self.total_time = time.mktime(t) - self.start_time
                self.flighttime.write('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
        elif type in ['SYS_STATUS']:
            sensors = { 'AS'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
                        'MAG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'INS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS,
                        'TERR' : mavutil.mavlink.MAV_SYS_STATUS_TERRAIN,
                        'RNG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION}
            for s in sensors.keys():
                bits = sensors[s]
                present = ((msg.onboard_control_sensors_enabled & bits) == bits)
                healthy = ((msg.onboard_control_sensors_health & bits) == bits)
                if not present:
                    fg = 'grey'
                elif not healthy:
                    fg = 'red'
                else:
                    fg = 'darkgreen'
                # for terrain show yellow if still loading
                if s == 'TERR' and fg == 'darkgreen' and master.field('TERRAIN_REPORT', 'pending', 0) != 0:
                    fg = 'yellow'
                if s == 'AS':
                    self.as_status.write(s, s, fg=fg)
                elif s == 'MAG' :
                    self.mag_status.write(s, s, fg=fg)
                elif s == 'INS' :
                    self.mag_status.write(s, s, fg=fg)
                elif s == 'AHRS' :
                    self.ahrs_status.write(s, s, fg=fg)
                elif s == 'TERR':
                    self.terr_status.write(s, s, fg=fg)
                elif s == 'RNG':
                    self.rng_status.write(s, s, fg=fg)


        elif type == 'WIND':
            self.wind.write('Wind', 'Wind %u/%.2f' % (msg.direction, msg.speed))

        elif type in ['RADIO', 'RADIO_STATUS']:
            if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
                fg = 'red'
            else:
                fg = 'black'
            self.radio_status.write('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)
        elif type == 'HEARTBEAT':
            self.uav_mode.write('Mode', '%s' % master.flightmode, fg='blue')
        elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
            self.wp.write('WP', 'WP %u' % msg.seq)
            lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
            lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
            if lat != 0 and lng != 0:
                airspeed = master.field('VFR_HUD', 'airspeed', 30)
                if abs(airspeed - self.speed) > 5:
                    self.speed = airspeed
                else:
                    self.speed = 0.98*self.speed + 0.02*airspeed
                self.speed = max(1, self.speed)
                time_remaining = int(self.estimated_time_remaining(lat, lng, msg.seq, self.speed))
                self.etr.write('ETR', 'ETR %u:%02u' % (time_remaining/60, time_remaining%60))

        elif type == 'NAV_CONTROLLER_OUTPUT':
            self.wpdist.write('WPDist', 'Distance %u' % msg.wp_dist)
            self.wpbearing.write('WPBearing', 'Bearing %u' % msg.target_bearing)
            if msg.alt_error > 0:
                alt_error_sign = "L"
            else:
                alt_error_sign = "H"
            if msg.aspd_error > 0:
                aspd_error_sign = "L"
            else:
                aspd_error_sign = "H"
            self.alterror.write('AltError', 'AltError %d%s' % (msg.alt_error, alt_error_sign))
            self.aspderror.write('AspdError', 'AspdError %.1f%s' % (msg.aspd_error*0.01, aspd_error_sign))

        elif type == 'MOUNT_REPORT':

            needed = ['GLOBAL_POSITION_INT', 'ATTITUDE']
            for n in needed:
                if not n in self.master.messages:
                    return

            gpi = self.master.messages['GLOBAL_POSITION_INT']
            att = self.master.messages['ATTITUDE']

            rotmat_copter_gimbal = Matrix3()
            rotmat_copter_gimbal.from_euler312(radians(m.pointing_b/100), radians(m.pointing_a/100), att.yaw)
            gimbal_dcm = rotmat_copter_gimbal

            lat = gpi.lat * 1.0e-7
            lon = gpi.lon * 1.0e-7
            alt = gpi.relative_alt * 1.0e-3

            if gpi.lat == 0:
                return

            # ground plane
            ground_plane = Plane()

            # the position of the camera in the air, remembering its a right
            # hand coordinate system, so +ve z is down
            camera_point = Vector3(0, 0, -alt)

            # get view point of camera when not rotated
            view_point = Vector3(1, 0, 0)


            # rotate view_point to form current view vector
            rot_point = gimbal_dcm * view_point

            # a line from the camera to the ground
            line = Line(camera_point, rot_point)

            # find the intersection with the ground
            pt = line.plane_intersection(ground_plane, forward_only=True)
            if pt is None:
                # its pointing up into the sky

                self.view_lat = 'LAT ------'
                self.view_lon = 'LON ------'
                self.sk_lat = 'LAT ------'
                self.sk_lon = 'LON ------'

            else:
                (view_lat, view_lon) = mp_util.gps_offset(lat, lon, pt.y, pt.x)
                self.view_lat = 'LAT '+str(view_lat)
                self.view_lon = 'LON '+str(view_lon)
                wgspoint = WGSPoint()
                (sk_lat, sk_lon) = wgspoint.WGS84_SK42(self.view_lat, self.view_lon, pt.z)
                (sk_lat_deg, sk_lat_min, sk_lat_sec) = self.decdeg2dms(sk_lat)
                (sk_lon_deg, sk_lon_min, sk_lon_sec) = self.decdeg2dms(sk_lon)
                self.sk_lat = 'LAT '+str(sk_lat_deg)+'d'+str(sk_lat_min)+"'"+str(sk_lat_sec)+'"'
                self.sk_lon = 'LON '+str(sk_lon_deg)+'d'+str(sk_lon_min)+"'"+str(sk_lon_sec)+"'"

                self.target_position_lat.write('Target_lat', self.view_lat, fg='white')
                self.target_position_lon.write('Target lon', self.view_lon, fg='white')
                self.target_positionsk_lat.write('TargetSK_lat', self.sk_lat, fg='white')
                self.target_positionsk_lon.write('TargetSK_lon', self.sk_lon, fg='white')

    def decdeg2dms(self, dd):
        '''convert decimal degree to degree minutes sec'''

        mnt,sec = divmod(dd*3600, 60)
        deg,mnt = divmod(mnt, 60)
        return int(deg), int(mnt), int(sec)

































def init(mpstate):
    '''initialise module'''
    return VideoCVModule(mpstate)
