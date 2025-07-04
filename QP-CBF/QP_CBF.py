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
import numpy as np
import osqp
from scipy import sparse


#URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
URI = "udp://0.0.0.0:19850"

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()


logging.basicConfig(level=logging.ERROR)


def qp_cbf_control(p,v, u_ref, p_bar, delta, mu, kappa):
    global solver
    """
    Solve QP-CBF (11) with OSQP:
      min ||u - u_ref||^2
      s.t. Lf_h + Lg_h u + κ(h) ≥ 0.

    OSQP formulation:

    min x^T * P * x^T + q^T x
      s.t. low_b ≤ A*x ≤ up_b

    """

    # 1) Compute h, Lf_h, Lg_h

    # Barrier Certificate h(x)
    d = np.array(p - p_bar)
    h = (d.T).dot(d + mu*v) - delta

    # Lie-derivatives for  h_dot = L_f_h + L_f_h*u
    L_f_h = 2 * (d.T).dot(v)
    L_g_h = mu * d


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
        x0 = 0.9 
        y0 = 1.63
        t_old = 0
        v_old = 0
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

            vz = Kp*(zdes-z)

            p_bar = np.array([2, 1.58]).reshape(2,1)
            v_bar = np.array([0,0]).reshape(2,1)
            a_bar = np.array([0,0]).reshape(2,1)
            
            mu = 0.1
            delta = 0.25
 
            u_ref = pdddes + Kp*(pdes-p) + Kd*(pddes-v)

            a = qp_cbf_control(p, v, u_ref, p_bar, delta, mu, kappa=1)

            input_v = v_old + a* delta_t

            mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(vz)) 

            t_old = t
            v_old = np.array([velocity_estimate[0],velocity_estimate[1]]).reshape(2,1)
            
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
    



if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf_pos = LogConfig(name='Position', period_in_ms=10)
        logconf_pos.add_variable('stateEstimate.x', 'float')
        logconf_pos.add_variable('stateEstimate.y', 'float')
        logconf_pos.add_variable('stateEstimate.z', 'float')


        log_thrust = LogConfig(name='Thrust', period_in_ms=10)
        log_thrust.add_variable('stateEstimate.thrust', 'float')
        
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



        logconf_pos.data_received_cb.add_callback(log_pos_callback)

        logconf_vel.data_received_cb.add_callback(log_vel_callback)
        logconf_att.data_received_cb.add_callback(log_att_callback)
        
        

        """
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        """
        
        
        logconf_pos.start()
        logconf_vel.start()
        logconf_att.start()
        #arma i motori
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)



        
        print('Logging Starting...')
        
        pid_try(scf)

        
        logconf_pos.stop()
        logconf_vel.stop()
        logconf_att.stop()

        scf.close_link()
        print("Stopped logging.")
        
