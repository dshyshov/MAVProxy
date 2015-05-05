#!/usr/bin/env python
'''Video downlink '''

import time, os, struct, sys
from time import sleep
import pingo
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from mplayer import Player, CmdPrefix
from pymavlink.rotmat import Vector3, Matrix3, Plane, Line
from math import radians
from subprocess import call
from pymavlink.wgstosk import WGPoint

# setup board hw

board = pingo.detect.MyBoard()
btn_cap = board.pins[6]
btn_cap.mode = pingo.IN
btn_lock_targ = board.pins[17]
btn_lock_targ.mode = pingo.IN

# Mplayer command prefix

#call(["cd", "/media/VIDEO/screenshot"])

class VideoModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(VideoModule, self).__init__(mpstate, "video", "video downlink", public=True)
        self.player = Player('-vo xv -tv driver=v4l2:input=1:device=/dev/video1 tv:// -vf expand=::200::1:,rectangle=30:30:345:273,screenshot -subfont-osd-scale 2')
        self.osd_string = 'Bla'
        self.osd_string_old = ''
        self.player.fullscreen = True
        self.player.osdlevel = 0
        self.player.cmd_prefix = CmdPrefix.PAUSING_KEEP
#        self.player.overlay_add('/home/ubuntu/crossb.pgm', 33, 400, 400, 0xFFFF)
        self.view_lat = None
        self.view_lon = None
        self.targ_locked = 'Targ_Unlocked'
	self.time_osd = time.time()





    def idle_task(self):

	if self.master.linkerror == True :
		self.osd_string = 'No_data_link_connection'
	
#	print(self.lost_gps_lock)	

        
        self.osd_show()

        if btn_cap.state == pingo.HIGH:
	   time.sleep(0.1)
	   if btn_cap.state == pingo.HIGH:
           	self.cap_img()

        if btn_lock_targ.state == pingo.HIGH:
	   time.sleep(0.1)
	   if btn_lock_targ.state == pingo.HIGH:
           	self.cmd_gimbal_roi()

    def osd_show(self):

	
        
        if self.osd_string != self.osd_string_old or (time.time() - self.time_osd) > 9:
            self.osd_string_old = self.osd_string
#            print(self.osd_string)
            self.player.osd_show_text(self.osd_string, 100000)
	    self.time_osd = time.time()

    def cap_img(self):

        self.player.screenshot(0)

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''

	


        if m.get_type() != 'MOUNT_STATUS':
            type = m.get_type()
#	    self.osd_string = 'No_Mount_Report'
            return

        needed = ['ATTITUDE', 'GLOBAL_POSITION_INT']
        for n in needed:
            if not n in self.master.messages:
                return

        gpi = self.master.messages['GLOBAL_POSITION_INT']
        att = self.master.messages['ATTITUDE']
        vehicle_dcm = Matrix3()
        vehicle_dcm.from_euler(0, 0, att.yaw)

        rotmat_copter_gimbal = Matrix3()
        rotmat_copter_gimbal.from_euler312(radians(m.pointing_a/100), radians(m.pointing_b/100), 0)
        gimbal_dcm = vehicle_dcm * rotmat_copter_gimbal

        lat = gpi.lat * 1.0e-7
        lon = gpi.lon * 1.0e-7
        alt = gpi.relative_alt * 1.0e-3
	

	if gpi.lat == 0 :
		self.osd_string = 'No_GPS_Lock!'
		return		
	
	elif alt < 5:
		self.osd_string = 'UAV_Landed'

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
            self.osd_string = 'Are_you_see_a_blue_sky_?'
            self.view_lat = None
            self.view_lon = None
            return None

        (self.view_lat, self.view_lon) = mp_util.gps_offset(lat, lon, pt.y, pt.x)
        wgspoint = WGSPoint()
        (sk_lat, sk_lon) = wgspoint.WGS84_SK42(self.view_lat, self.view_lon, pt.z)

        self.osd_string = ('LON_' + str(round(self.view_lon,5)) + '__LAT_' + str(round(self.view_lat,5))+'_SK42LAT_'+str(sk_lat)+'_SK42LON_'+str(sk_lon)+"_"+self.targ_locked)
#        print(self.osd_string)

 

    def cmd_gimbal_roi(self):
        '''control roi position'''
        lat = None
        lon = None
        try:
            lat = self.view_lat
            lon = self.view_lon
        except Exception:
            print("No GPS Lock avialable")
            return
        if lat is None:
            print("No GPS Lock avialable")
            return
        
        mode = mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT
        self.targ_locked = 'Target_locked'

        self.master.mav.mount_configure_send(self.target_system,
                                             self.target_component,
                                             mode,
                                             1, 1, 1)

        self.master.mav.mount_control_send(self.target_system,
                                           self.target_component,
                                           latlon[0]*1e7,
                                           latlon[1]*1e7,
                                           0, # altitude zero for now
                                           0)
            


def init(mpstate):

    
    return VideoModule(mpstate)


