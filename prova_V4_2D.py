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
import math
import numpy as np
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
#importo i commandi
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

def euler_to_rotm():
    deg2rad = np.pi / 180.0  
    cyaw = math.cos(attitude_estimate[2])     
    syaw = math.sin(attitude_estimate[2])     
    Rotm = np.array([[cyaw, -syaw],[syaw,cyaw]])

    return Rotm

def make_R_dot(wx,wy,wz):
        deg2rad = np.pi / 180.0    
        cyaw = math.cos( attitude_estimate[2] * deg2rad)     
        syaw = math.sin( attitude_estimate[2] * deg2rad)        
        
        wz=float(wz)
        
        Rdot = np.array([[wz * syaw,wz * cyaw],[-wz*cyaw, wz*syaw]])

        return Rdot


def trajectory_function(type,t):
    if type == 'horizontal_circle':
        r = 5
        w = 0.05

        x = r * np.cos(w * t)
        y = r * np.sin(w * t)
        z = 0.5

        vx = -r * w * np.sin(w * t)
        vy =  r * w * np.cos(w * t)
        vz = 0

        ax = -r * w * w * np.cos(w * t)
        ay = -r * w * w * np.sin(w * t)
        az = 0 
    elif type == 'linear_motion':
        x = 0.5 * t
        y = 0
        
        vx = 0.5
        vy = 0
        
        
        ax = 0
        ay = 0
       
    elif type == 'spezzata':
        if(t<=10):
            x=0.5*t
            y=0

            vx = 0.5
            vy = 0

            ax = 0
            ay = 0
        elif(t>10 and t<=20):
            x=0.5*10
            y=0.5*(t-10) 

            vx = 0
            vy = 0.5

            ax = 0
            ay = 0
        elif(t>20 and t<=30):
            x=0.5*10-0.5*(t-20)
            y=0.5*10 

            vx = -0.5
            vy = 0

            ax = 0
            ay = 0
        elif(t>30 and t<=40):
            x=0
            y=0.5*10-0.5*(t-30) 

            vx = 0
            vy = -0.5

            ax = 0
            ay = 0
        else:
            x=0
            y=0

            vx = 0
            vy = 0

            ax = 0
            ay = 0

    else:
        x = 0
        y = 0
        z = 0

        vx = 0
        vy = 0
        vz = 0

        ax = 0
        ay = 0

    pdes = np.array([x, y]).reshape(2, 1)
    pdotdes = np.array([vx, vy]).reshape(2, 1)
    pddotdes = np.array([ ax, ay]).reshape(2, 1)

    return pdes, pdotdes, pddotdes   

            
def pid_try(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        Kp= 2
        Kd =5
        t_start=time.time()
        a_old=np.array([0,0]).reshape(2,1)
        t_old = 0
        v_old = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
        angular_old = np.array(attitude_estimate).flatten().reshape(3,1)
        
        #posizione ostacolo 
        """
        obstacles_positions = [ 
            np.array([8.39, 5.44 , 0.8]).reshape(3,1),
            np.array([1.5, 10, 0.3]).reshape(3,1),  #ostacolo 1 10
            np.array([1.5, 10, 0.8]).reshape(3,1),
            np.array([1.5, 8, 1]).reshape(3,1), 
            np.array([1.5, 8, 1.5]).reshape(3,1),
            np.array([2, 10.5, 1]).reshape(3,1), 
            np.array([2.0, 10.5, 1.5]).reshape(3,1)
            ]
        obstacles_velocities = [ 
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1)
            ]
        obstacles_accelerations = [ 
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1),
            np.array([0,0,0]).reshape(3,1)
            ]
        """
        obstacles_positions = [np.array([10,0]).reshape(2,1)]
        obstacles_velocities = [np.array([0,0]).reshape(2,1)]
        obstacles_accelerations = [np.array([0,0]).reshape(2,1)]
        j=0
        while(1):
            t= time.time() - t_start 
            
            #print(t)
            delta_t= t - t_old
            
            #traiettoria di rifermiento 
            pdes , pddes , pdddes = trajectory_function('spezzata',t)
            
            p =np.array([position_estimate[0],position_estimate[1]]).reshape(2,1)

            z = position_estimate[2]
            v = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            
            #estimate angular velocity
            deg2rad = np.pi / 180.0
            w_x = (attitude_estimate[0]-angular_old[0]) * deg2rad / delta_t
            w_y = (attitude_estimate[1]-angular_old[1])* deg2rad / delta_t
            w_z = (attitude_estimate[2]-angular_old[2]) * deg2rad / delta_t 
            
            #calcolo matrici derivate  
            zdes = DEFAULT_HEIGHT
            R = euler_to_rotm()
            R_T = R.T
            R_dot = make_R_dot(w_x,w_y,w_z)
            R_dot_T = R_dot.T
            
            #ostacolo più vicino
            min_dist = float('inf')
            closest_idx = -1
            for i, obs_pos in enumerate(obstacles_positions):
                dist = np.linalg.norm(p - obs_pos)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            #print(closest_idx)
                    
            # Ostacolo scelto
            p_bar = obstacles_positions[closest_idx]
            v_bar = obstacles_velocities[closest_idx]
            a_bar = obstacles_accelerations[closest_idx]
            
            V = p - p_bar
            V_T = V.reshape(1,2)
            V_dot = v - v_bar
            V_dot_T = V_dot.reshape(1,2)
            
            #mu = 2
            #delta = 3
            
            #parametro può toccare
            mu = 0.9
            
            delta = 1
            
            
            #non dovrebbero toccare i droni
            #mu = 3
            #delta = 2
            
            
            
            
            a = pdddes + Kp*(pdes-p) + Kd*(pddes-v) 
            
            M = np.array([[0.92, 0],
                 [0, 0.92]])
            
            #CBF
            h = V_T @ R_T @ M @ R @ V + mu * V_T @ M @ V_dot - delta
            h_dot = 2* V_T @ R_T @ M @ R @ V_dot + V_T @ ( R_dot_T @ M @ R + R_T @ M @ R_dot) @ V + mu*V_T @ M @ (a - a_bar)
            
            Proj = V @ np.linalg.pinv(V_T @ V) @ V_T 
            I = np.array([[1,0],[0,1]]) 
            Proj_perp= I - Proj
            a_diverso = np.array([0,0]).reshape(2,1)
            vz = Kp*(zdes-z)
            if h_dot > 0 or h > delta:
                #print("no ostacolo")
                #A_MAX = 0.8
                #a[2] = np.clip(a[2], -1000, A_MAX)
                input_v = v_old +a * delta_t
                mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(vz))
                
            else:
                print("ostacolo")
                    
                a_diverso = - 1/mu * np.linalg.pinv(M) @ Proj @ (2*R_T @ M @ R @ V_dot + (R_dot_T @ M @ R + R_T @ M @ R_dot) @ V) + np.linalg.pinv(M) @ a_bar + np.linalg.pinv(M) @ Proj_perp @ a 
                
                
                input_v = v_old + a_diverso * delta_t
                mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(vz)) 
            #print((p-pdes).reshape(1,3))
            t_old = t
            v_old = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            angular_old = np.array(attitude_estimate).flatten().reshape(3,1)
            
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
        #variabile globale definita sotto URI
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print('entra')
    print(cflib.crtp.scan_interfaces())
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        print('entra')
        logconf_pos = LogConfig(name='Position', period_in_ms=10)
        logconf_pos.add_variable('stateEstimate.x', 'float')
        logconf_pos.add_variable('stateEstimate.y', 'float')
        logconf_pos.add_variable('stateEstimate.z', 'float')

        
        logconf_vel = LogConfig(name='Velocity', period_in_ms=10)
        logconf_vel.add_variable('stateEstimate.vx', 'float')
        logconf_vel.add_variable('stateEstimate.vy', 'float')
        logconf_vel.add_variable('stateEstimate.vz', 'float')

      
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
        #arma i motori
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)



        #take_off_simple(scf)
        print('arrivo')
        #move_box_limit(scf)
        pid_try(scf)

        # move_linear_simple(scf)
        # move_box_limit(scf)
        logconf_pos.stop()
        logconf_vel.stop()
        logconf_att.stop()
