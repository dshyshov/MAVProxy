#!/usr/bin/env python
'''
Antenna tracker control module
This module catches MAVLINK_MSG_ID_GLOBAL_POSITION_INT
and sends them to a MAVlink connected antenna tracker running
ardupilot AntennaTracker
Mike McCauley, based on earlier work by Andrew Tridgell
June 2012
'''

import sys, os, time
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import mavproxy_map
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_param import ParamState

import numpy as np
from scipy.signal import butter, lfilter, freqz


# this should be in mavutil.py
mode_mapping_antenna = {
    'MANUAL' : 0,
    'AUTO' : 10,
    'INITIALISING' : 16
    }

class TrackerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        from pymavlink import mavparm
        super(TrackerModule, self).__init__(mpstate, "tracker", "antenna tracker control module")
        self.connection = None
        self.tracker_param = mavparm.MAVParmDict()
        self.pstate = ParamState(self.tracker_param, self.logdir, self.vehicle_name, 'tracker.parm')
        self.tracker_settings = mp_settings.MPSettings(
            [ ('port', str, "/dev/ttyUSB0"),
              ('baudrate', int, 57600),
              ('debug', int, 0),
	      ('offset_dz', float ,1)
              ]
            )
        self.add_command('tracker', self.cmd_tracker,
                         "antenna tracker control module",
                         ['<start|arm|disarm|level|mode|position|calpress|mode>',
                          'set (TRACKERSETTING)',
                          'param <set|show|fetch|help> (TRACKERPARAMETER)',
                          'param (TRACKERSETTING)'])
        self.add_completion_function('(TRACKERSETTING)', self.tracker_settings.completion)
        self.add_completion_function('(TRACKERPARAMETER)', self.complete_parameter)
	self.tracker_abspr = None
	self.tracker_diffpr = None
	self.tracker_temp = None
	self.baro_trigger = True
	self.baro_offset = None
	self.baro_init_offset = None
	self.baro_acc_offset = None
	self.armed_trigger = False
	self.filter_data = 0
	self.filter_i = 0
	self.filter_cutoff = 0.01

	self.baro_offset_fil = 0

    def complete_parameter(self, text):
        '''complete a tracker parameter'''
        return self.tracker_param.keys()

    def find_connection(self):
        '''find an antenna tracker connection if possible'''
        if self.connection is not None:
            return self.connection
        for m in self.mpstate.mav_master:
#	    print(m)
            if 'HEARTBEAT' in m.messages:
                if m.messages['HEARTBEAT'].type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
		    print(m)
                    return m
        return None

    def cmd_tracker(self, args):
        '''tracker command parser'''
        usage = "usage: tracker <start|set|arm|disarm|level|param|mode|position> [options]"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "start":
            self.cmd_tracker_start()
        elif args[0] == "set":
            self.tracker_settings.command(args[1:])
        elif args[0] == 'arm':
            self.cmd_tracker_arm()
        elif args[0] == 'disarm':
            self.cmd_tracker_disarm()
        elif args[0] == 'level':
            self.cmd_tracker_level()
        elif args[0] == 'param':
            self.cmd_tracker_param(args[1:])
        elif args[0] == 'mode':
            self.cmd_tracker_mode(args[1:])
        elif args[0] == 'position':
            self.cmd_tracker_position(args[1:])
        elif args[0] == 'calpress':
            self.cmd_tracker_calpress()
        else:
            print(usage)

    def cmd_tracker_position(self, args):
        '''tracker manual positioning commands'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        positions = [0, 0, 0, 0, 0] # x, y, z, r, buttons. only position[0] (yaw) and position[1] (pitch) are currently used
        for i in range(0, 4):
            if len(args) > i:
                positions[i] = int(args[i]) # default values are 0
        connection.mav.manual_control_send(connection.target_system,
                                           positions[0], positions[1],
                                           positions[2], positions[3],
                                           positions[4])

    def cmd_tracker_calpress(self):
        '''calibrate barometer on tracker'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        connection.calibrate_pressure()

    def cmd_tracker_mode(self, args):
        '''set arbitrary mode'''
        connection = self.find_connection()
        if not connection:
            print("No antenna tracker found")
            return
        mode_mapping = connection.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        if len(args) != 1:
            print('Available modes: ', mode_mapping.keys())
            return
        mode = args[0].upper()
        if mode not in mode_mapping:
            print('Unknown mode %s: ' % mode)
            return
        connection.set_mode(mode_mapping[mode])

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet from the master vehicle. Relay it to the tracker
        if it is a GLOBAL_POSITION_INT'''
        if m.get_type() in ['GLOBAL_POSITION_INT', 'SCALED_PRESSURE']:
            connection = self.find_connection()
            if not connection:
                return
            if m.get_srcSystem() != connection.target_system:
                connection.mav.send(m)
		if self.baro_trigger :		
		    self.baro_init_offset = self.mav_param.get('GND_ALT_OFFSET' ,None )
		    self.baro_acc_offset = self.baro_init_offset
		    print('Init_offset_'+str(self.baro_init_offset))
		    if self.baro_init_offset != None :
			self.baro_trigger = False  
		    

    def idle_task(self):
        '''called in idle time'''
        if not self.connection:
            return

        # check for a mavlink message from the tracker
        m = self.connection.recv_msg()
	
        if m is None:
            return

        if self.tracker_settings.debug:
            print(m)

        self.pstate.handle_mavlink_packet(self.connection, m)
        self.pstate.fetch_check(self.connection)

	m_get_type = m.get_type()
	
	if m_get_type == 'SCALED_PRESSURE':
	    self.tracker_abspr = m.press_abs
	    self.tracker_diffpr = m.press_diff
	    self.tracker_temp = m.temperature

	    self.baro_corr()


	armed = self.master.motors_armed()

	if armed != self.armed_trigger :
	    print ('Opa')
	    if armed :

		self.baro_init_offset = 0
		self.baro_acc_offset = 0
		self.baro_offset = 0
		self.baro_offset_fil = 0
		self.cmd_tracker_calpress()
		self.mav_param.mavset(self.master, 'GND_ALT_OFFSET', 0)
		self.armed_trigger = True
	    else :
	    	self.armed_trigger = False
	    
	
        if self.module('map') is None:
            return

        if m_get_type == 'GLOBAL_POSITION_INT':
            (self.lat, self.lon, self.heading) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01)
            if self.lat != 0 or self.lon != 0:
                self.module('map').create_vehicle_icon('AntennaTracker', 'red', follow=False, vehicle_type='antenna')
                self.mpstate.map.set_position('AntennaTracker', (self.lat, self.lon), rotation=self.heading)


    def cmd_tracker_start(self):
        if self.tracker_settings.port == None:
            print("tracker port not set")
            return
        print("connecting to tracker %s at %d" % (self.tracker_settings.port,
                                                  self.tracker_settings.baudrate))
        m = mavutil.mavlink_connection(self.tracker_settings.port,
                                       autoreconnect=True,
                                       source_system=self.settings.source_system,
                                       baud=self.tracker_settings.baudrate)

        m.mav.srcComponent = self.settings.source_component
        if self.logdir:
            m.setup_logfile(os.path.join(self.logdir, 'tracker.tlog'))
        self.connection = m

    def cmd_tracker_arm(self):
        '''Enable the servos in the tracker so the antenna will move'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.arducopter_arm()

    def cmd_tracker_disarm(self):
        '''Disable the servos in the tracker so the antenna will not move'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.arducopter_disarm()

    def cmd_tracker_level(self):
        '''Calibrate the accelerometers. Disarm and move the antenna level first'''
        if not self.connection:
            print("tracker not connected")
            return
        self.connection.calibrate_level()

    def cmd_tracker_param(self, args):
        '''Parameter commands'''
        if not self.connection:
            print("tracker not connected")
            return
        self.pstate.handle_command(self.connection, self.mpstate, args)

    def baro_corr(self):
	'''Baro automatic correction'''
	if not self.connection:
            print("tracker not connected")
            return
	if self.baro_init_offset is None :
	    print('No init offset from master')
	    return
	'''Calculate offset'''
	if self.tracker_abspr != None :	
	    if self.baro_offset == None :
		self.baro_offset = self.baro_init_offset

	    dH = - round((8000/(self.tracker_abspr/10)*(1+0.00366*self.tracker_temp/100))*(self.tracker_diffpr/10), 1)
	    self.baro_offset = self.baro_init_offset + dH
	    
	    dH_fil = self.baro_filter(dH)
	    
	    if dH_fil != None:
	    	self.baro_offset_fil = self.baro_init_offset + dH_fil
	    	print ('RAW      ' + str(self.baro_offset))
	    	print ('Filtered ' + str(self.baro_offset_fil))

	    ''' if out of deadzone send to master GND_ALT_OFFSET '''
	    if (self.baro_offset_fil - self.baro_acc_offset  >= self.tracker_settings.offset_dz) or (self.baro_acc_offset - self.baro_offset_fil >= self.tracker_settings.offset_dz):
		self.baro_acc_offset = self.baro_offset_fil
#		self.mav_param.mavset(self.master, 'GND_ALT_OFFSET', self.baro_acc_offset)
		print('Send GND_ALT_OFFSET to MAV ' + str(self.baro_acc_offset) + ' m')
	else:
	    print ('No abs press')


    def baro_filter(self, data):
	self.filter_data += data
	self.filter_i += 1
	if self.filter_i == 30 :
	    baro_filtered = round(self.filter_data / self.filter_i, 0)
	    self.filter_i = 0
	    self.filter_data = 0
	    return baro_filtered
	else:
	    return None
	
	
def init(mpstate):
    '''initialise module'''
    return TrackerModule(mpstate)
