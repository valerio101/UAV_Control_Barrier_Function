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
def trajectory_function(type,t):
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
		x = 0.9 +0.1 * t
		y = 1.63
		z =0.5
		
		vx = 1
		vy = 0
		vz = 0
		
		ax = 0
		ay = 0
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
			
def pid_try(scf):
	with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
		Kp= 2
		Kd =5
		t_start=time.time()
		a_old=np.array([0,0,0]).reshape(3,1)
		t_old = 0
		v_old = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
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
		obstacles_positions = [np.array([2,1.63,0.45]).reshape(3,1)]
		obstacles_velocities = [np.array([0,0,0]).reshape(3,1)]
		obstacles_accelerations = [np.array([0,0,0]).reshape(3,1)]
		j=0
		while(1):
			t= time.time() - t_start 
			
			print(t)
			delta_t= t - t_old
			
			#traiettoria di rifermiento 
			pdes , pddes , pdddes = trajectory_function('linear_motion',t)
			
			#posizione drone
			p =np.array([position_estimate[0],position_estimate[1],position_estimate[2]]).reshape(3,1)
			#velocità drone
			v = np.array([velocity_estimate[0],velocity_estimate[1],velocity_estimate[2]]).reshape(3,1)
			
			#estimate angular velocity
			w_x = (attitude_estimate[0]-angular_old[0]) / delta_t
			w_y = (attitude_estimate[1]-angular_old[1]) / delta_t
			w_z = (attitude_estimate[2]-angular_old[2]) / delta_t
			
			#calcolo matrici derivate  
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
			print(closest_idx)
					
			# Ostacolo scelto
			p_bar = obstacles_positions[closest_idx]
			v_bar = obstacles_velocities[closest_idx]
			a_bar = obstacles_accelerations[closest_idx]
			
			V = p - p_bar
			V_T = V.reshape(1,3)
			V_dot = v - v_bar
			V_dot_T = V_dot.reshape(1,3)
			
			#mu = 2
			#delta = 3
			
			#parametro può toccare
			mu = 0.9
			
			delta = 0.3
			
			
			#non dovrebbero toccare i droni
			#mu = 3
			#delta = 2
			
			
			
			
			a = pdddes + Kp*(pdes-p) + Kd*(pddes-v) 
			
			M = np.array([[0.92, 0, 0],
				 [0, 0.92, 0],
				 [0, 0, 0.29]])
			
			#CBF
			h = V_T @ R_T @ M @ R @ V + mu * V_T @ M @ V_dot - delta
			h_dot = 2* V_T @ R_T @ M @ R @ V_dot + V_T @ ( R_dot_T @ M @ R + R_T @ M @ R_dot) @ V + mu*V_T @ M @ (a - a_bar)
			
			Proj = V @ np.linalg.pinv(V_T @ V) @ V_T 
			I = np.array([[1,0,0],[0,1,0],[0,0,1]]) 
			Proj_perp1= I - Proj
			a_diverso = np.array([0,0,0]).reshape(3,1)
			if h_dot > 0 or h > delta:
				print("no ostacolo")
				#A_MAX = 0.8
				#a[2] = np.clip(a[2], -1000, A_MAX)
				input_v = v_old +a * delta_t
				mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(input_v[2]))
				
			else:
				
				print("ostacolo")
				if( a[2] < 0 and V[2] >0 ):
					v3 = - float(V[2])
				elif( a[2] >0 and V[2] <0):
					v3 = -float(V[2])
				else: 
					v3=float(V[2])
					
				Proj_perp =  np.array([ [0,0,Proj_perp1[0,2]/v3] , [0,0,Proj_perp1[1,2]/v3], [0,0,Proj_perp1[2,2]/v3] ])
				a_diverso = - 1/mu * np.linalg.pinv(M) @ Proj @ (2*R_T @ M @ R @ V_dot + (R_dot_T @ M @ R + R_T @ M @ R_dot) @ V) + np.linalg.pinv(M) @ a_bar + np.linalg.pinv(M) @ Proj_perp @ a 
				print(a_diverso.reshape(1,3))
				A_MAX = 0.8
				#saturo 
				a_diverso[2] = np.clip(a_diverso[2], -A_MAX, A_MAX)
				#saturo velocità
				#V_MAX=1
				#input_v = np.clip(a_diverso, -V_MAX, V_MAX)
				
				input_v = v_old + a_diverso * delta_t
				mc.start_linear_motion(float(input_v[0]),float(input_v[1]),float(input_v[2])) 
			print((p-pdes).reshape(1,3))
			t_old = t
			v_old = np.array(velocity_estimate).reshape(3,1)
			angular_old = np.array(attitude_estimate).flatten().reshape(3,1)
			
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
