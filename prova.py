# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

#URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
URI = "udp://0.0.0.0:19850"
DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0]
velocity_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]


def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while (1):
            '''if position_estimate[0] > BOX_LIMIT:
                mc.start_back()
            elif position_estimate[0] < -BOX_LIMIT:
                mc.start_forward()
            '''

            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def accazzo(scf):
    print("INsied")
    
    time.sleep(1)
    ii = 0
    while(ii<50):
        scf.cf.commander.send_position_setpoint(0,0,10,0)
        time.sleep(0.1)
        
        ii+=1
        print("Dioc: "+str(ii))
    ii = 0
    sign = 1
    roll_ref = 0

    #scf.cf.param.set_value("flightmode.stabModeRoll", "0")
    #scf.cf.param.set_value("flightmode.stabModePitch", "0")
    #scf.cf.param.set_value("flightmode.stabModeYaw", "0")
    scf.cf.commander.send_notify_setpoint_stop()
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    
    time.sleep(0.1)
    while(ii < 10000):
        roll = attitude_estimate[0]
        print(str(roll) + " " + str(roll_ref))
        if roll_ref > 10 or roll_ref < -10:
            sign = -sign
        roll_ref+=sign*0.2
        scf.cf.commander.send_setpoint(roll_ref,0,0,30000)
        #scf.cf.commander.send_hover_setpoint(0.1,0,0.1,1)
        time.sleep(0.1)


def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate
    
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

    
def log_vel_callback(timestamp, data, logconf):
    global velocity_estimate

    velocity_estimate[0] = data['stateEstimate.vx']
    velocity_estimate[1] = data['stateEstimate.vy']
    velocity_estimate[2] = data['stateEstimate.vz']

def log_att_callback(timestamp, data, logconf):
    global attitude_estimate

    attitude_estimate[0] = data['stateEstimate.roll']
    attitude_estimate[1] = data['stateEstimate.pitch']
    attitude_estimate[2] = data['stateEstimate.yaw']



    




def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        print("AAAAAAA ")
        logconf_pos = LogConfig(name='Position', period_in_ms=10)
        logconf_pos.add_variable('stateEstimate.x', 'float')
        logconf_pos.add_variable('stateEstimate.y', 'float')
        logconf_pos.add_variable('stateEstimate.z', 'float')

        print("BBBBBBBB ")
        logconf_vel = LogConfig(name='Velocity', period_in_ms=10)
        logconf_vel.add_variable('stateEstimate.vx', 'float')
        logconf_vel.add_variable('stateEstimate.vy', 'float')
        logconf_vel.add_variable('stateEstimate.vz', 'float')

        print("CCCCCCC ")
        logconf_att = LogConfig(name='Attitude', period_in_ms=10)
        logconf_att.add_variable('stateEstimate.roll', 'float')
        logconf_att.add_variable('stateEstimate.pitch', 'float')
        logconf_att.add_variable('stateEstimate.yaw', 'float')

        scf.cf.log.add_config(logconf_pos)
        scf.cf.log.add_config(logconf_vel)
        scf.cf.log.add_config(logconf_att)
        #scf.cf.commander.send_hover_setpoint(0.1,0,0,1)
        
        logconf_pos.data_received_cb.add_callback(log_pos_callback)
        logconf_vel.data_received_cb.add_callback(log_vel_callback)
        logconf_att.data_received_cb.add_callback(log_att_callback)
        """
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        """
        
        #print(scf.cf.param.get_value("stateEstimate.y",1))
        logconf_pos.start()
        logconf_vel.start()
        logconf_att.start()
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)



        #take_off_simple(scf)
        accazzo(scf)

        # move_linear_simple(scf)
        # move_box_limit(scf)
        logconf_pos.stop()
        logconf_vel.stop()
        logconf_att.stop()
