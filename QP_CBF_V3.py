import logging
import sys
import time
from threading import Event
import math 

import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
import numpy as np
import osqp
from scipy import sparse


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#URI = "udp://0.0.0.0:19850"

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()


logging.basicConfig(level=logging.ERROR)

def trajectory_function(type,t):
    x0=1.2
    y0=1.4
    if type == 'horizontal_circle':
        r = 10
        w = 0.05

        x = r * np.cos(w * t)
        y = r * np.sin(w * t)
        z = 0.8

        vx = -r * w * np.sin(w * t)
        vy =  r * w * np.cos(w * t)
        vz = 0

        ax = -r * w * w * np.cos(w * t)
        ay = -r * w * w * np.sin(w * t)
        az = 0 
    elif type == 'linear_motion':
        x = 0.5 * t
        y = -0.6
        
        vx = 0.5
        vy = 0
        
        
        ax = 0
        ay = 0
    elif type == 'spezzata':
        vv = 0.1
        if(t<=25):
            x=x0+vv*t
            y=y0

            vx = vv
            vy = 0

            ax = 0
            ay = 0
        elif(t>25 and t<=40):
            x=x0+vv*25
            y=y0+vv*(t-25) 

            vx = 0
            vy = vv

            ax = 0
            ay = 0
        elif(t>40 and t<=65):
            x=x0+vv*25-vv*(t-40)
            y=y0+vv*15 

            vx = -vv
            vy = 0

            ax = 0
            ay = 0
        elif(t>65 and t<=80):
            x=x0
            y=y0+vv*15-vv*(t-65) 

            vx = 0
            vy = -vv

            ax = 0
            ay = 0
        else:
            x=x0
            y=y0

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

def qp_cbf_control(p,v, u_ref, delta, mu,R,R_dot, kappa):
    global solver
    """
    Solve QP-CBF (11) with OSQP:
      min ||u - u_ref||^2
      s.t. Lf_h + Lg_h u + κ(h) ≥ 0.

    OSQP formulation:

    min x^T * P * x^T + q^T x
      s.t. low_b ≤ A*x ≤ up_b

    """
    x0=1.2 
    y0=1.4
    obstacles_positions = [np.array([x0+1.25,y0+1.75]).reshape(2,1),
                            np.array([x0+2.5,y0-0.25]).reshape(2,1)]
    #compute near obstacal
    min_dist = float('inf')
    closest_idx = -1
    for i, obs_pos in enumerate(obstacles_positions):
        dist = np.linalg.norm(p - obs_pos)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    print(closest_idx)
    p_bar = obstacles_positions[closest_idx]

    # 1) Compute h, Lf_h, Lg_h

    # Barrier Certificate h(x)
    d = np.array(p - p_bar)
    M = np.array([[0.92, 0],
          [0, 0.92]])
   # h = d.T * R.T @ M @ R @ d + mu * d.T @ M @ v - delta
    h = d.T @ d + mu * d.T @ v - delta

    # Lie-derivatives for  h_dot = L_f_h + L_f_h*u
    L_f_h = 2 * d.T  @ v 
    L_g_h = mu * d.T 


    # 2) Setup Optimization Problem
    # Objective Function: ½ (u - u_ref)^T I (u - u_ref)
    #    ⇒ P = Id(NxN), q = -u_ref
    q = -u_ref

    # 3) Vincolo CBF: L_f_h + L_g_h·u + κ(h) ≥ 0
    #    -inf ≤ −L_g_h·u ≤ L_f_h + κ(h)
    A = sparse.csc_matrix(-L_g_h.reshape(1, 2))                               
    up_b = np.array([ L_f_h + kappa * h ])

    # 4) Setup and solving OSQP
    solver.update(q=q, A=A, u=up_b)
    res = solver.solve()

    if res.info.status_val != osqp.constant('OSQP_SOLVED'):
        raise ValueError(f"OSQP non ha trovato soluzione: {res.info.status}")

    print("h: " + str(h))
    return np.array(res.x).reshape(2,1) # vettore u*

# ───────────────────────────────────────────────────────────

position_estimate = [0, 0, 0]
velocity_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]

solver = osqp.OSQP()
solver.setup(P=sparse.eye(2, format='csc'), 
             q=np.array([0,0]), 
             A= sparse.csc_matrix(np.array([1,1]).reshape(1, 2)), 
             l=np.array([-np.inf]), u = np.array([np.inf]), verbose=False)

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
        t_old = 0
        v_old = 0
        angular_old = np.array(attitude_estimate).flatten().reshape(3,1)
        while(t<120):
            t= time.time() - t_start 
            delta_t = t - t_old
            if(delta_t == 0):
                delta_t =t
            
            deg2rad = np.pi / 180.0
            w_x = (attitude_estimate[0]-angular_old[0]) * deg2rad / delta_t
            w_y = (attitude_estimate[1]-angular_old[1])* deg2rad / delta_t
            w_z = (attitude_estimate[2]-angular_old[2]) * deg2rad / delta_t 
            
            p =np.array([position_estimate[0],position_estimate[1]]).reshape(2,1)
            
            z = position_estimate[2]
            v = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            
            #traiettoria di rifermiento 
            pdes , pddes , pdddes = trajectory_function('spezzata',t)

            #pdes= np.array([R*np.cos(w*t)-R, R*np.sin(w*t), DEFAULT_HEIGHT+0.01*t]).
            zdes = DEFAULT_HEIGHT
 
            vz = Kp*(zdes-z)

            mu = 0.1
            delta = 0.5
 
            u_ref = pdddes + Kp*(pdes-p) + Kd*(pddes-v)

            a = qp_cbf_control(p, v, u_ref, delta, mu, kappa=1)
            input_v = v_old + a* delta_t
            v_body = input_v
            mc.start_linear_motion(float(v_body[0]),float(v_body[1]),float(vz)) 

            t_old = t
            v_old = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            angular_old = np.array(attitude_estimate).flatten().reshape(3,1)
            time.sleep(0.1)
        
        print("Stopping")
        
    
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        

def param_deck_loco(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        #variabile globale definita sotto URI
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
