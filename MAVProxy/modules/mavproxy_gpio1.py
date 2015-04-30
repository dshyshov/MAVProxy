#!/usr/bin/env python
'''GPIO gimbal & mode control'''

import time, os, struct
from time import sleep
import pingo
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

board = pingo.detect.MyBoard()
pitch_in = board.pins['A3']
roll_in = board.pins['A2']
btngim_zero = board.pins[3]
btnzm_plus = board.pins[4]
btnzm_minus = board.pins[5]

btngim_zero.mode = pingo.IN
btnzm_plus.mode = pingo.IN
btnzm_minus.mode = pingo.IN
pitch_in.mode = pingo.ANALOG
roll_in.mode = pingo.ANALOG

zoom_servo = 11

dead_zone = 0.2
dead_zone_joy = 10.0

per = roll_in.ratio

print(per)

class GPIOModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(GPIOModule, self).__init__(mpstate, "gpio1", "gpio gimbal & mode control", public = True)
        self.pitch = 0
        self.last_pitch = 0
        self.roll = 0
        self.last_roll = 0
        self.joy_pitch = 0.0
        self.joy_roll = 0.0
        self.zoom_pwm = 1500
       

    def idle_task(self):
        self.joy_pitch = round((pitch_in.ratio(171,4095,0.0,1.0)*100)-50)
        self.joy_roll = round((roll_in.ratio(210,4075,0.0,1.0)*100)-50)


        if  not (-15  < self.joy_pitch < 15 ):
            
            self.pitch = self.pitch + self.joy_pitch / 100
            print(self.pitch)
            
            if (self.pitch > 90.0):
                self.pitch = 90.0
            if (self.pitch < -90.0):
                self.pitch = -90.0
        
        if  not (-15  < self.joy_roll < 15 ):
            
            self.roll = self.roll + self.joy_roll / 150
            print(self.roll)

            if (self.roll > 70):
                 self.roll = 70
            if (self.roll < -70):
                 self.roll = -70

        
        if  not (self.pitch - dead_zone) < self.last_pitch < (self.pitch + dead_zone) or not (self.roll - dead_zone) < self.last_roll < (self.roll + dead_zone):
            self.last_pitch = self.pitch
            self.last_roll = self.roll
            self.send_control()

        elif  btngim_zero.state == pingo.HIGH :
	    time.sleep(0.1)
	    if btngim_zero.state == pingo.HIGH :

            	self.pitch = 0
            	self.roll = 0

        if btnzm_plus.state == pingo.HIGH and self.zoom_pwm != 1900:
	    time.sleep(0.1)
	    if btnzm_plus.state == pingo.HIGH :

            	self.zoom_pwm = 1900
            	self.set_zoom()
            

        if btnzm_minus.state == pingo.HIGH and self.zoom_pwm != 1100:
	    time.sleep(0.1)
	    if btnzm_minus.state == pingo.HIGH :

            	self.zoom_pwm = 1100
            	self.set_zoom()

        if btnzm_plus.state != pingo.HIGH and btnzm_minus.state != pingo.HIGH  and self.zoom_pwm != 1500:

            self.zoom_pwm = 1500
            self.set_zoom()

           

    def send_control(self):
        '''send gimbal control packet'''
        
        mode = mavutil.mavlink.MAV_MOUNT_MODE_RETRACT
        
        self.master.mav.mount_configure_send(self.target_system,
                                             self.target_component,
                                             mode,
                                             1, 1, 1)
        
        pitch = self.last_pitch
        roll = self.last_roll
#        print(roll)
#        print(pitch)
        self.master.mav.mount_control_send(self.target_system,
                                           self.target_component,
                                           pitch*100,
                                           roll*100,
                                           0,
                                           0)


   
    
    def set_zoom(self):

        self.master.mav.command_long_send(self.target_system,
                                                   self.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                                   zoom_servo , self.zoom_pwm,
                                                   0, 0, 0, 0, 0)



def init(mpstate):
    '''initialise module'''
    return GPIOModule(mpstate)
