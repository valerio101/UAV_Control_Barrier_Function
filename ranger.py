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
URI =  uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E5')
DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0]
velocity_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]
range_obs = [0, 0, 0,0 ]


def move_box_limit(scf):
    # questa riga MotionCommander lo fa partire da terra con alteza definita da DEFAULT_HEIGHT
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
            
            
def accelerazione(t,v):
    Kp=1000
    Kd=100
    w = 0.
    R = 5
    pdes= np.array([R * math.cos(w * t), R* math.sin(w * t), 0.05*t]).reshape(3,1)
            
            
    pddes= np.array([-R * w * math.sin(w * t), R * w * math.cos(w * t), 0.05]).reshape(3,1)
            
    pdddes=np.array([ - R * w * w * math.cos(w * t),- R * w * w * math.sin(w * t), 0]).reshape(3,1)
            
    real_p =np.array([position_estimate[0],position_estimate[1],position_estimate[2]]).reshape(3,1)
    
    real_v = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
    return pdddes + Kp*(pdes-real_p) + Kd*(pddes-real_v)
    
def rk4_step(f,t,v,dt):
    k1 =f(t,v)
    k2=f(t+dt/2,v+dt/2*k1)
    k3=f(t+dt/2,v+dt/2*k2)
    k4=f(t+dt,v+dt*k3)
    return dt/6*(k1+2*k2+2*k3+k4)	
    
def pid_try(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        #Kp=10
        #Kd=1
        Kp= 2
        Kd =5
        w = 0.4
        R = 0.4
        time.sleep(3)
        t_start=time.time()
        t=0
        a_old=np.array([0,0,0]).reshape(3,1)
        t_old =0
        v_old = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
        
        while(t<30):
            t= time.time() - t_start 
            delta_t = t - t_old
            if(delta_t == 0):
                delta_t =t
            print(t)
            real_p =np.array([position_estimate[0],position_estimate[1],position_estimate[2]]).reshape(3,1)
    
            real_v = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
            yaw = np.radians(attitude_estimate[2])
            RT = np.array([[np.cos(yaw), np.sin(yaw),0],[-np.sin(yaw) ,np.cos(yaw), 0], [0,0,1],])
            real_p = RT @ real_p
            real_v = RT @ real_v
            #pdes= np.array([R*np.cos(w*t)-R, R*np.sin(w*t), DEFAULT_HEIGHT+0.01*t]).reshape(3,1)
            pdes= np.array([0.1*t, 0, DEFAULT_HEIGHT]).reshape(3,1)
            
            
            #pddes= np.array([-w*R*np.sin(w*t),w*R*np.cos(w*t), 0.01]).reshape(3,1)
            pddes=np.array([0.1, 0, 0]).reshape(3,1)
            #pdddes=np.array([-w**2*R*np.cos(w*t),-w**2*R*np.sin(w*t), 0]).reshape(3,1)
            pdddes=np.array([0, 0, 0]).reshape(3,1)
            a=pdddes + Kp*(pdes-real_p) + Kd*(pddes-real_v)
            #v_cmd = rk4_step(accelerazione,t,real_v,delta_t)
            print((pdes-real_p).reshape(1,3))
            #dovrebbe essere input_v = v_old + a*delta_t
            input_v = v_old + a * delta_t
            mc.start_linear_motion(input_v[0],input_v[1],input_v[2],10)
            #mc.start_linear_motion( pddes[0], pddes[1], pddes[2])
            #mc.start_linear_motion( v_cmd[0], v_cmd[1], v_cmd[2])
            t_old = t
            v_old = np.array(velocity_estimate).reshape(3,1)
            time.sleep(0.1)
        """
            t= time.time() - t_start 
            delta_t = t - t_old
            if(delta_t == 0):
                delta_t =t
            print(t)
            real_p =np.array([position_estimate[0],position_estimate[1],position_estimate[2]]).reshape(3,1)
    
            real_v = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
            
            pdes= np.array([R * math.cos(w * t)-R, R* math.sin(w * t), DEFAULT_HEIGHT]).reshape(3,1)
            
            
            pddes= np.array([-R * w * math.sin(w * t), R * w * math.cos(w * t), 0]).reshape(3,1)
            
            pdddes=np.array([ - R * w * w * math.cos(w * t),- R * w * w * math.sin(w * t), 0]).reshape(3,1)
            a=pdddes + Kp*(pdes-real_p) + Kd*(pddes-real_v)
            #v_cmd = rk4_step(accelerazione,t,real_v,delta_t)
            print((pdes-real_p).reshape(1,3))
            #dovrebbe essere input_v = v_old + a*delta_t
            input_v = v_old + a * delta_t
            mc.start_linear_motion(input_v[0],input_v[1],input_v[2])
            #mc.start_linear_motion( pddes[0], pddes[1], pddes[2])
            #mc.start_linear_motion( v_cmd[0], v_cmd[1], v_cmd[2])
            t_old = t
            v_old = np.array(velocity_estimate).reshape(3,1)
            time.sleep(0.1)
        """
            
             
            
            
        
        


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

def log_range_callback(timestamp, data, logconf):
    global range_obs

    range_obs[0] = data['range.front']
    range_obs[1] = data['range.back']
    range_obs[2] = data['range.left'] 
    range_obs[3] = data['range.right']
    print('obs: ({}, {}, {}, {})'.format(range_obs[0], range_obs[1], range_obs[2],range_obs[3]))
    

def euler_to_rotm(psi,theta,phi):
        cpitch= math.cos(theta)    
        spitch = math.sin(theta)   
        croll = math.cos(psi)     
        sroll = math.sin(psi)      
        cyaw = math.cos(phi)     
        syaw = math.sin(phi)     
        Rotm = np.array([
            [cpitch*cyaw, sroll*spitch*cyaw - croll*syaw, croll*spitch*cyaw + sroll*syaw],
            [cpitch*syaw, sroll*spitch*syaw + croll*cyaw, croll*spitch*syaw - sroll*cyaw],
            [-spitch, sroll*cpitch, croll*cpitch]]
        )
        return Rotm

def make_R_dot(psi,theta,phi,wx,wy,wz):
        cpitch= math.cos(theta)    
        spitch = math.sin(theta)   
        croll = math.cos(psi)    
        sroll = math.sin(psi)     
        cyaw = math.cos(phi)     
        syaw = math.sin(phi)     

        Rdot = np.array([
            [-wy*(croll*cyaw*spitch + sroll*syaw) - wz*(croll*syaw - cyaw*spitch*sroll), -cpitch*cyaw*wz + wx*(croll*cyaw*spitch + sroll*syaw), cpitch*cyaw*wy + wx*(croll*syaw - cyaw*spitch*sroll)],
            [-wy*(croll*spitch*syaw - cyaw*sroll) + wz*(croll*cyaw + spitch*sroll*syaw), -cpitch*syaw*wz + wx*(croll*spitch*syaw - cyaw*sroll), cpitch*syaw*wy - wx*(croll*cyaw + spitch*sroll*syaw)],
            [                                             cpitch*(-croll*wy + sroll*wz),                           cpitch*croll*wx + spitch*wz,                         -cpitch*sroll*wx - spitch*wy]
        ])

        return Rdot


def trajectory_function(type,t):
    if type == 'horizontal_circle':
        r = 10
        w = 2

        x = r * np.cos(w * t)
        y = r * np.sin(w * t)
        z = 0

        vx = -r * w * np.sin(w * t)
        vy =  r * w * np.cos(w * t)
        vz = 0

        ax = -r * w * w * np.cos(w * t)
        ay = -r * w * w * np.sin(w * t)
        az = 0 
    else:
        x = 0
        y = 0
        z = 0

        vx = 0
        vy = 0
        vz = 0

        ax = 0
        ay = 0
        az = 0 

    pdes = np.array([x, y, z,]).reshape(3, 1)
    pdotdes = np.array([vx, vy, vz,]).reshape(3, 1)
    pddotdes = np.array([ ax, ay, az]).reshape(3, 1)

    return pdes, pdotdes, pddotdes

    




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
        print('entra1')

        
        logconf_vel = LogConfig(name='Velocity', period_in_ms=10)
        logconf_vel.add_variable('stateEstimate.vx', 'float')
        logconf_vel.add_variable('stateEstimate.vy', 'float')
        logconf_vel.add_variable('stateEstimate.vz', 'float')

      
        logconf_att = LogConfig(name='Attitude', period_in_ms=10)
        logconf_att.add_variable('stateEstimate.roll', 'float')
        logconf_att.add_variable('stateEstimate.pitch', 'float')
        logconf_att.add_variable('stateEstimate.yaw', 'float')

        logconf_range = LogConfig(name='Range', period_in_ms=10)
        logconf_range.add_variable('range.front', 'float')
        logconf_range.add_variable('range.back', 'float')
        logconf_range.add_variable('range.left', 'float')
        logconf_range.add_variable('range.right', 'float')
    
        scf.cf.log.add_config(logconf_pos)
        scf.cf.log.add_config(logconf_vel)
        scf.cf.log.add_config(logconf_att)
        scf.cf.log.add_config(logconf_range)
        print('entra2')

        #scf.cf.commander.send_hover_setpoint(0.1,0,0,1)
        
        logconf_pos.data_received_cb.add_callback(log_pos_callback)
        logconf_vel.data_received_cb.add_callback(log_vel_callback)
        logconf_att.data_received_cb.add_callback(log_att_callback)
        logconf_range.data_received_cb.add_callback(log_range_callback)
        print('entra2')

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
    
        
        #print(scf.cf.param.get_value("stateEstimate.y",1))
        logconf_pos.start()
        logconf_vel.start()
        logconf_att.start()
        logconf_range.start()
        #arma i motori
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)



        #take_off_simple(scf)
        print('arrivo')
        #move_box_limit(scf)
        #pid_try(scf)
        
        # move_linear_simple(scf)
        # move_box_limit(scf)
        logconf_pos.stop()
        logconf_vel.stop()
        logconf_att.stop()
        logconf_range.stop()
