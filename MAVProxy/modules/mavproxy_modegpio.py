

#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil
import pingo


board = pingo.detect.MyBoard()

btn_mode_land = board.pins[7]
btn_mode_auto = board.pins[8]
btn_mode_fbwa = board.pins[9]
btn_mode_circle = board.pins[10]
btn_mode_rtl = board.pins[11]

led_mode_land = board.pins[14]
led_mode_auto = board.pins[13]
led_mode_fbwa = board.pins[12]
led_mode_circle = board.pins[15]
led_mode_rtl = board.pins[16]


btn_mode_land.mode = pingo.IN
btn_mode_auto.mode = pingo.IN
btn_mode_fbwa.mode = pingo.IN
btn_mode_circle.mode = pingo.IN
btn_mode_rtl.mode = pingo.IN

led_mode_land.mode = pingo.OUT
led_mode_auto.mode = pingo.OUT
led_mode_fbwa.mode = pingo.OUT
led_mode_circle.mode = pingo.OUT
led_mode_rtl.mode = pingo.OUT



from MAVProxy.modules.lib import mp_module

class ModeGpioModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeGpioModule, self).__init__(mpstate, "modegpio")
        self.mode = self.master.flightmode
        self.user_mode = self.master.flightmode
        self.mode_mapping = self.master.mode_mapping()
	self.curr_mode = self.master.flightmode
        print(self.mode_mapping)

        
    def idle_task(self):
	
	self.curr_mode = self.master.flightmode        
	
        self.gpio_mode()
        
        self.mode_ind()


    def gpio_mode(self):
	
	
        if btn_mode_land.state == pingo.HIGH and self.curr_mode != 'LAND':
            time.sleep(0.1)
	    if btn_mode_land.state == pingo.HIGH :
	    	self.user_mode = 'LAND'
	    	self.mode_set()
        elif btn_mode_auto.state == pingo.HIGH and self.curr_mode != 'AUTO':
	    time.sleep(0.1)
	    if btn_mode_auto.state == pingo.HIGH :
	    	self.user_mode = 'AUTO'
	    	self.mode_set()
        elif btn_mode_fbwa.state == pingo.HIGH and self.curr_mode != 'FBWA':
	    time.sleep(0.1)
	    if btn_mode_fbwa.state == pingo.HIGH :
	    	self.user_mode = 'FBWA'
	    	self.mode_set()
        elif btn_mode_circle.state == pingo.HIGH and self.curr_mode != 'MANUAL':
            time.sleep(0.1)
	    if btn_mode_circle.state == pingo.HIGH :
	    	self.user_mode = 'MANUAL'
	    	self.mode_set()
        elif btn_mode_rtl.state == pingo.HIGH and self.curr_mode != 'RTL':
            time.sleep(0.1)
	    if btn_mode_rtl.state == pingo.HIGH :
	    	self.user_mode = 'RTL'
	    	self.mode_set()

    def mode_set(self):
 
        if self.curr_mode != self.user_mode:
            self.mode = self.user_mode
            if self.mode not in self.mode_mapping:
                print('Unknown mode %s: ' % self.mode)
                return
            self.master.set_mode(self.mode_mapping[self.mode])
        else:
            return



    def mode_ind(self):

        self.curr_mode = self.master.flightmode
#        print(curr_mode)

	if self.master.linkerror == True:
            led_mode_land.lo()
            led_mode_auto.lo()
            led_mode_fbwa.lo()
            led_mode_circle.lo()
            led_mode_rtl.lo()
	    return

        elif self.curr_mode == 'LAND':
            led_mode_land.hi()
            led_mode_auto.lo()
            led_mode_fbwa.lo()
            led_mode_circle.lo()
            led_mode_rtl.lo()
        elif self.curr_mode == 'AUTO':
            led_mode_land.lo()
            led_mode_auto.hi()
            led_mode_fbwa.lo()
            led_mode_circle.lo()
            led_mode_rtl.lo()
        elif self.curr_mode == 'FBWA':
            led_mode_land.lo()
            led_mode_auto.lo()
            led_mode_fbwa.hi()
            led_mode_circle.lo()
            led_mode_rtl.lo()
        elif self.curr_mode == 'MANUAL':
            led_mode_land.lo()
            led_mode_auto.lo()
            led_mode_fbwa.lo()
            led_mode_circle.hi()
            led_mode_rtl.lo()
        elif self.curr_mode == 'RTL':
            led_mode_land.lo()
            led_mode_auto.lo()
            led_mode_fbwa.lo()
            led_mode_circle.lo()
            led_mode_rtl.hi()
        else:
            led_mode_land.lo()
            led_mode_auto.lo()
            led_mode_fbwa.lo()
            led_mode_circle.lo()
            led_mode_rtl.lo()

   
   
def init(mpstate):
    '''initialise module'''
    return ModeGpioModule(mpstate)
