"""
 Video Downlink with additional data and using OpenCV
"""

import os, sys, math, time
import cv2

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
#            self.VideoCV.set_status('Thr', 'Thr %u' % msg.throttle)
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
#        elif type == 'ATTITUDE':
#            self.VideoCV.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
#            self.VideoCV.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))
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

#        elif type == 'HWSTATUS':
#            if msg.Vcc >= 4600 and msg.Vcc <= 5300:
#                fg = 'darkgreen'
#            else:
#                fg = 'red'
#            self.VideoCV.set_status('Vcc', 'Vcc %.2f' % (msg.Vcc * 0.001), fg=fg)
#        elif type == 'POWER_STATUS':
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_CHANGED:
#                fg = 'red'
#            else:
#                fg = 'darkgreen'
#            status = 'PWR:'
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_USB_CONNECTED:
#                status += 'U'
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_BRICK_VALID:
#                status += 'B'
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_SERVO_VALID:
#                status += 'S'
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_OVERCURRENT:
#                status += 'O1'
#            if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:
#                status += 'O2'
#            self.VideoCV.set_status('PWR', status, fg=fg)
#            self.VideoCV.set_status('Srv', 'Srv %.2f' % (msg.Vservo*0.001), fg='darkgreen')
        elif type in ['RADIO', 'RADIO_STATUS']:
            if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
                fg = 'red'
            else:
                fg = 'black'
            self.radio_status.write('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)
        elif type == 'HEARTBEAT':
            self.uav_mode.write('Mode', '%s' % master.flightmode, fg='blue')
#            if self.max_link_num != len(self.mpstate.mav_master):
#                for i in range(self.max_link_num):
#                    self.link%i_status = Value('Link%u'%(i+1), '', fg='grey')
#                self.max_link_num = len(self.mpstate.mav_master)
#            for m in self.mpstate.mav_master:
#                linkdelay = (self.mpstate.status.highest_msec - m.highest_msec)*1.0e-3
#                linkline = "Link %u " % (m.linknum+1)
#                if m.linkerror:
#                    linkline += "down"
#                    fg = 'red'
#                else:
#                    packets_rcvd_percentage = 100
#                    if (m.mav_loss != 0): #avoid divide-by-zero
#                        packets_rcvd_percentage = (1.0 - (float(m.mav_loss) / float(m.mav_count))) * 100.0
#
#                    linkline += "OK (%u pkts, %.2fs delay, %u lost) %u%%" % (m.mav_count, linkdelay, m.mav_loss, packets_rcvd_percentage)
#                    if linkdelay > 1:
#                        fg = 'orange'
#                    else:
#                        fg = 'darkgreen'
#                self.VideoCV.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
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




def init(mpstate):
    '''initialise module'''
    return VideoCVModule(mpstate)
