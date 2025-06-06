import logging
import sys
import time
from threading import Event

import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E4')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0,0, 0]
velocity_estimate = [0, 0, 0]
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
        x0 = 0.9 
        y0 = 1.63
        while(t<30):
            t= time.time() - t_start 
            delta_t = t - t_old
            if(delta_t == 0):
                delta_t =t
        
            p =np.array([position_estimate[0],position_estimate[1]]).reshape(2,1)
            z = position_estimate[2]
            v = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            
            #pdes= np.array([R*np.cos(w*t)-R, R*np.sin(w*t), DEFAULT_HEIGHT+0.01*t]).reshape(3,1)
            pdes= np.array([x0+0.1*t, y0]).reshape(2,1)
            zdes = DEFAULT_HEIGHT
            
            #pddes= np.array([-w*R*np.sin(w*t),w*R*np.cos(w*t), 0.01]).reshape(3,1)#np.array([0.2, 0.1, 0]).reshape(3,1)
            pddes= np.array([0.1, 0]).reshape(2,1)

            #pdddes=np.array([-w**2*R*np.cos(w*t),-w**2*R*np.sin(w*t), 0]).reshape(3,1)
            pdddes = np.array([0, 0]).reshape(2,1)
            a=pdddes + Kp*(pdes-p) + Kd*(pddes-v)
            vz = Kp*(zdes-z)
            #v_cmd = rk4_step(accelerazione,t,v,delta_t)
            print((pdes-p).reshape(1,2))
            #dovrebbe essere input_v = v_old + a*delta_t
            input_v = v_old + a * delta_t

            p_bar = np.array([2, 1.63]).reshape(2,1)
            v_bar = np.array([0,0]).reshape(2,1)
            a_bar = np.array([0,0]).reshape(2,1)
            
            V = p - p_bar
            V_T = V.reshape(1,2)
            V_dot = v - v_bar
            V_dot_T = V_dot.reshape(1,2)
            mu = 1.2
            delta = 0.5
 
            a = pdddes + Kp*(pdes-p) + Kd*(pddes-v)
            
            #CBF
            h = V_T @ V + mu * V_T @ V_dot - delta
            h_dot = 2* V_T @ V_dot + mu * V_dot_T @ V_dot + mu*V_T @ (a - a_bar)
            
            #CBF con M
            #h= V_T @ M @ V + mu * V_T  @ M @ V_dot - delta
            #h_dot = 2* V_T @ M @ V_dot + mu * V_dot_T @ M @ V_dot + mu*V_T @ M @ (a - a_bar)
            
            Proj = V @ np.linalg.pinv(V_T @ V) @ V_T 
            I = np.array([[1,0],[0,1]]) 
            Proj_perp = I - Proj
            
            if h_dot > 0 or h > delta:
                print("no ostacolo")
                input_v = v_old +a * delta_t
                
            else:
                print("ostacolo")
                a_diverso = - 2/mu*V_dot + a_bar + Proj_perp @ a 
                #a_diversoM = -2 /mu * V_dot + a_bar + Proj_perp @ a
                input_v = v_old + a_diverso* delta_t 
                
            if abs(input_v[0]) > 1:
                input_v[0] = np.sign(input_v[0])*1
            if abs(input_v[1]) > 1:
                input_v[1] = np.sign(input_v[1])*1
            if abs(vz) > 1:
                vz = np.sign(vz)*1

            mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(vz)) 
            print((p-pdes).reshape(1,2))
            t_old = t
            v_old = np.array(velocity_estimate).reshape(2,1)
            
            time.sleep(0.1)

def move_box_limit(scf, start):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2
        i = 0
        while (i < 4):
            #if position_estimate[0] > BOX_LIMIT:
            #    mc.start_back()
            #elif position_estimate[0] < -BOX_LIMIT:
            #    mc.start_forward()

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
            if time.time() -start > 120:
                break

        mc.land(0.05)
        mc.stop()


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        i = 0
        time.sleep(1)
        while i < 2:
            mc.forward(0.5)
            time.sleep(.1)
            mc.turn_left(180)
            time.sleep(.1)
            mc.forward(0.5)
            time.sleep(.1)
            i+=1


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def param_deck_loco(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def position_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = data['kalman.stateX']
    position_estimate[1] = data['kalman.stateY']
    position_estimate[2]= data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(position_estimate[0], position_estimate[1], position_estimate[2]))

def velocity_callback(timestamp, data, logconf2):
    global velocity_estimate
    velocity_estimate[0] = data['kalman.statePX']
    velocity_estimate[1] = data['kalman.statePY']
    velocity_estimate[2]= data['kalman.statePZ']


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    log_conf2 = LogConfig(name='Velocity', period_in_ms=10)
    log_conf2.add_variable('kalman.statePX', 'float')
    log_conf2.add_variable('kalman.statePY', 'float')
    log_conf2.add_variable('kalman.statePZ', 'float')

    scf.cf.log.add_config(log_conf)
    scf.cf.log.add_config(log_conf2)

    log_conf.data_received_cb.add_callback(position_callback)
    log_conf2.data_received_cb.add_callback(velocity_callback)

    log_conf.start()
    log_conf2.start()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_printing(scf)
        #move_linear_simple(scf)
        pid_try(scf)
