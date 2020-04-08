#!/usr/bin/env python

import rospy
import numpy as np
import random
from numpy.linalg import inv
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32


class Server:
	def __init__(self):
		#==================================================================================================
		#variabel kinematik
		self.alp = np.array([[0*np.pi/180., 120.*np.pi/180., 240.*np.pi/180.]]).T #robot specification
		self.l = 0.20 #panjang posisi roda robot dari titik tengah robot
		self.r = 0.05 #besar jari - jari roda robot
		self.Jr = self.jacobianR()
		self.lamda = np.matrix([[20., 0., 0.], [0., 20., 0.], [0., 0., 20.]])
		self.lamda1 = np.matrix([[50., 0., 0.], [0., 50., 0.], [0., 0., 50.]])
		self.lamda2 = np.matrix([[35., 0., 0.], [0., 35., 0.], [0., 0., 35.]])
		self.kp = np.matrix([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
		#self.visual_lamda = np.matrix([[30., 0., 0.], [0., 42., 0.], [0., 0., 42.]])
		self.visual_lamda = np.matrix([[30., 0., 0.], [0., 35., 0.], [0., 0., 32.]])
		#self.visual_lamda1 = np.matrix([[40., 0., 0.], [0., 45., 0.], [0., 0., 45.]])		
		self.visual_lamda1 = np.matrix([[40., 0., 0.], [0., 45., 0.], [0., 0., 42.]])			
		#lambda untuk mengambil bola mati
		self.visual_lamda2 = np.matrix([[25., 0., 0.], [0., 25., 0.], [0., 0., 28.]])		
		self.visual_lamda3 = np.matrix([[40., 0., 0.], [0., 40., 0.], [0., 0., 43.]])		
		self.ts = 0.1
		self.er = np.array([[0., 0., 0.]]).T
		self.last_er = np.array([[0., 0., 0.]]).T
		self.pose = np.matrix([[0., 0., 0.]]).T
		self.pose_des = np.matrix([[0., 0., 0.]]).T
		self.ball_pos = np.matrix([[0., 0., 0.]]).T
		self.ball_des = np.matrix([[0.2, 0., 0.]]).T	#desiren saat mendekati bola permainan
		self.ball_des1 = np.matrix([[0.35, 0., 0.]]).T	#desired saat mendekati bola mati
		#=======================================================================================
		self.game_mode = 0.
		self.attack = 0.
		self.treshold_error = 0.25 									#threshold for position error
		self.treshold_error_kickoff = 0.125 						#threshold for position error
		self.treshold_error_shooting = 0.25 							#threshold for position error
		self.treshold_visual_error = 0.15							#threshold for visual servoing
		self.lost_ball_secs = 0
		self.get_ball_secs = 0
		self.goal_reached = 0.										#flag positioning
		self.lost_ball_thresh = 15
		self.limit_pwm = 700										#maximum RPM/PWM
		self.min_pwm = 160											#minimum threshold RPM/PWM
		self.drib_slow = 10
		self.drib_fast = 100
		#obstacle avoidance variables
		self.k_att = 0.5										#attractive force gain
		#self.k_rep = 10											#repulsive force gain (obstacle)
		#self.k_border = 1.75										#repulsive force gain (border)
		self.k_rep = 0											#repulsive force gain (obstacle)
		self.k_border = 0.										#repulsive force gain (border)		
		self.k_lm = 0.5											#local minimal angle gain
		self.safe_lm = 1.2											#local minimal threshold
		self.safe_d = 3.25											#safe distance threshold between robot and obstacle
		self.local_minima_d = 0.5 * self.safe_d						#local_minima_distance threshold between robot and obstacle
		self.v_attr = None									
		self.v_reps = None
		self.obstacles = np.array([[100, 100]])
		self.border = np.array([[100.0, 100.0],[100.0, 100.0]])
		self.border_limits = np.array([[-5.0, -3.5], [5.0, 3.5]])	#4 points of border coordinate ([x1, y1], [x2, y2])
		self.r_border = 0.65
		#circle trajectory variables
		self.ball_error = 0.
		self.flag_status = Int32MultiArray()
		self.flag_status.data = [0, 0]
		self.position_code = 0
		self.corner_pos = 0
		#game variables
		self.strategy = 0
		self.pass_ball = 0		
		self.robot2_ball_flag = 0
		self.robot1_ball_flag = 0
		self.lock = 0
		self.dribbler_lock = 0
		self.kick_ball = 0
		self.get_ball_secs = 0
		self.stop_ball = 0
		print ("robot2 kinematic is running")
	
	def jacobianR(self):
		tr = -90.*np.pi/180.
		R  = np.matrix([[np.cos(tr), -np.sin(tr), 0.], [np.sin(tr), np.cos(tr), 0.], [0., 0., 1]])
		Jr = np.matrix([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
		Jr[0,0]=np.cos(self.alp[0,0])
		Jr[0,1]=np.cos(self.alp[1,0])
		Jr[0,2]=np.cos(self.alp[2,0])
		Jr[1,0]=np.sin(self.alp[0,0])
		Jr[1,1]=np.sin(self.alp[1,0])
		Jr[1,2]=np.sin(self.alp[2,0])
		Jr[2,0]=1/self.l
		Jr[2,1]=1/self.l
		Jr[2,2]=1/self.l
		return self.r*(R*Jr)
		#return self.r*Jr
	
	def jacobianW(self,th,Jr):
		rotZ = np.matrix([[np.cos(th), -np.sin(th), 0.], [np.sin(th), np.cos(th), 0.], [0., 0., 1]])
		return rotZ*Jr	
	
	def attr_force(self, current_pos, goal, gain):
		att = np.linalg.norm(goal - current_pos) * gain
		theta = np.arctan2((goal[1] - current_pos[1]),(goal[0] - current_pos[0]))
		return att, theta
	
	def reps_force(self, cur_pos, obs, gain):
		reps = []
		thetas = []		
		current_pos = np.array([cur_pos[0,0], cur_pos[1,0]])
		for i in range(len(obs)):			
			Dobs = np.linalg.norm(obs[i] - current_pos.T)	
			#print(Dobs)
			if Dobs < self.safe_d :
				rep = 0.
				theta = 0.
				rep = gain * (1/Dobs - 1/self.safe_d) * 1/(Dobs**2)
				theta = np.arctan2((obs[i,1] - current_pos[1]),(obs[i,0] - current_pos[0]))
				reps.append(rep)				
				if(theta > np.pi):
					theta = (-np.pi) + (theta % np.pi)
				elif(theta < -np.pi):
					theta = (np.pi) + (theta % -np.pi)
				thetas.append(theta)
		return reps,thetas
	
	def border_force(self, cur_pos, obs, gain):
		reps = []
		thetas = []		
		current_pos = np.array([cur_pos[0,0], cur_pos[1,0]])
		for i in range(len(obs)):			
			Dobs = np.linalg.norm(obs[i] - current_pos.T)			
			if Dobs < self.r_border :
				rep = 0.
				theta = 0.
				rep = gain * (1/Dobs - 1/self.r_border) * 1/(Dobs**2)
				if(current_pos[0]>3.9):
					theta = 0.
				elif(current_pos[0]<-3.9):
					theta = np.pi			
				elif(current_pos[1]>2.5):
					theta = 0.5*np.pi
				elif(current_pos[1]<-2.5):
					theta = -0.5*np.pi				
				#theta = np.arctan2((obs[i,1] - current_pos[1]),(obs[i,0] - current_pos[0]))
				reps.append(rep)
				thetas.append(theta)
		#print ("Border Th = ", thetas)
		return reps,thetas
	
	def game_mode_callback(self, dat): #callback untuk mode permainan (start, stop, positioning) dan status menyerang(menyerang/bertahan)
		self.game_mode = dat.data[0]
		self.position_code = dat.data[1]
		self.attack = dat.data[2]
		self.corner_pos = 0
		if(self.game_mode == 2):
			if(self.attack == 0):
				self.pass_ball = 1
			else:
				self.pass_ball = 0
			self.dribbler_lock = 1
		else:
			self.dribbler_lock = 0
		print self.pass_ball
		self.main()
	
	def game_strategy_callback(self, dat):
		self.strategy = dat.data[0]
		#self.pass_ball = dat.data[1]
		
	def cmd_vel_callback(self, dat):
		vx = dat.linear.x
		vy = dat.linear.y
		vz = dat.angular.z
		self.er = np.matrix([[vx, vy, vz]]).T
		#self.main()
	
	def pose_callback(self, dat):
		self.pose[0,0] = dat.x
		self.pose[1,0] = dat.y
		self.pose[2,0] = dat.z
		if(self.pose[1,0] < self.border_limits[0,1] + self.r_border): #yr < y1 + r
			self.border[0,0] = self.pose[0,0]
			self.border[0,1] = self.border_limits[0,1]
			#print "Border y1"
		elif(self.pose[1,0] > self.border_limits[1,1] - self.r_border): #yr > y2 + r
			self.border[0,0] = self.pose[0,0]		
			self.border[0,1] = self.border_limits[1,1]
			#print "Border y2"
		else:
			self.border[0,0] = 100.
			self.border[0,1] = 100.
			#print "No border y near your robot"								
		self.main()
	
	def pose_des_callback(self, dat):
		self.pose_des[0,0] = dat.x
		self.pose_des[1,0] = dat.y
		self.pose_des[2,0] = dat.z
	
	def ball_pos_callback(self, dat):		
		self.ball_pos[0,0] = dat.x
		self.ball_pos[1,0] = dat.y
		self.ball_pos[2,0] = dat.z
		self.main()
	
	def ball_flag_callback(self, dat):
		self.robot2_ball_flag = dat.data
		if(self.robot2_ball_flag == 0):
			self.get_ball_secs = 0
	
	def robot1_ball_flag_callback(self, dat):
		self.robot1_ball_flag = dat.data	
	
	def obstacle_position_callback(self, dat):
		rot = np.matrix([[np.cos(self.pose[2,0]), -np.sin(self.pose[2,0])], [np.sin(self.pose[2,0]), np.cos(self.pose[2,0])]])		
		obstacles_coordinate = []
		robot_coordinate = np.matrix([[self.pose[0,0], self.pose[1,0]]]).T 		
		for i in range(0,len(dat.data),2):
			obstacles_coordinate.append([dat.data[i],dat.data[i+1]])
		self.obstacles = np.array([[100., 100.]]*len(obstacles_coordinate))
		obs = np.array(obstacles_coordinate)
		for i in range(len(obs)):
			x = np.array([[obs[i][0], obs[i][1]]]).T
			obss = rot.dot(x)+robot_coordinate
			self.obstacles[i,0] = obss[0,0]
			self.obstacles[i,1] = obss[1,0]
			#print obss[0,0], obss[1,0]
			#print obss
		#print(self.obstacles)
		
	def pwm_leveling(self, w):
		temp = np.array([abs(w[0,0]), abs(w[1,0]), abs(w[2,0])])
		out = np.matrix([[0., 0., 0.]]).T
		maximum = np.max(temp)
		if(maximum > self.limit_pwm):
			out[0,0] = (w[0,0]/maximum)*self.limit_pwm
			out[1,0] = (w[1,0]/maximum)*self.limit_pwm
			out[2,0] = (w[2,0]/maximum)*self.limit_pwm
		else:
			out[0,0] = w[0,0]
			out[1,0] = w[1,0]
			out[2,0] = w[2,0]
		#====================== W1 ====================
		if(out[0,0]>0.4) and (out[0,0]<self.min_pwm):
			out[0,0] = self.min_pwm
		elif (out[0,0]<-0.4) and (out[0,0]>-self.min_pwm):
			out[0,0] = -self.min_pwm
		#====================== W2 ====================
		if(out[1,0]>0.4) and (out[1,0]<self.min_pwm):
			out[1,0] = self.min_pwm
		elif (w[1,0]<-0.4) and (out[1,0]>-self.min_pwm):
			out[1,0] = -self.min_pwm
		#====================== W3 ====================
		if(out[2,0]>0.4) and (out[2,0]<self.min_pwm):
			out[2,0] = self.min_pwm
		elif (out[2,0]<-0.4) and (out[2,0]>-self.min_pwm):
			out[2,0] = -self.min_pwm
		#==============================================
		motor.x = out[0,0] #belakang
		motor.y = out[1,0] #kanan
		motor.z = out[2,0] #kiri	
		#print motor
		pwm_publisher.publish(motor)
	
	def circular_trajectory_kinematics(self):
		Jinv = np.linalg.inv(self.Jr)
		error = np.matrix([[0., 0., 0.]]).T
		if((self.pose[2,0]>np.radians(175))or (self.pose[2,0]<np.radians(-90))):
			error = error * 0
			self.corner_pos = 1
		else:
			error[0,0] = 0.
			error[1,0] = -1.5
			error[2,0] = 7.5
		w = self.visual_lamda1*Jinv*error
		self.pwm_leveling(w)				
	
	def obstacle_avoidance_kinematic(self):
		J = self.jacobianW(self.pose[2,0],self.Jr)
		Jinv = np.linalg.inv(J)
		#compute error position
		error = self.pose_des - self.pose								#compute error for invers kinematic move
		des_pos = np.array([self.pose_des[0,0], self.pose_des[1,0]])	
		curr_pos = np.array([self.pose[0,0], self.pose[1,0]])
		ers = des_pos - curr_pos										#compute error for repulsive gain value
		#print (np.linalg.norm(ers))
		if(np.linalg.norm(ers)<=0.75):									#if robot is close enough to the desired position
			rep = 0.													#the repulsive gain set into 0
		else:															
			rep = self.k_rep
		#==================================
		#compute force
		v_attr, theta_attr = self.attr_force(self.pose, self.pose_des, self.k_att) # get F_attractive
		v_reps, theta_reps = self.reps_force(self.pose, self.obstacles, rep) # get F_repulsive and F_lm
		v_bord, theta_bord = self.border_force(self.pose, self.border, self.k_border) # get F_border
		x_dot = v_attr * np.cos(theta_attr)
		y_dot = v_attr * np.sin(theta_attr)
		obs_sum = len(v_reps)
		ang = np.pi									
		for j in range(len(v_reps)): # adding x_dot and y_dot with F_repulsive and F_lm
			x_dot = x_dot - v_reps[j] * np.cos(theta_reps[j])
			y_dot = y_dot - v_reps[j] * np.sin(theta_reps[j])		
			lm_rate = v_attr/v_reps[j]
			if(lm_rate < self.safe_lm): # local minimal addition
				x_dot = x_dot - v_reps[j] * np.cos(theta_reps[j] + self.k_lm * ang) # adding x_dot with F_lm if F_lm available
				y_dot = y_dot - v_reps[j] * np.sin(theta_reps[j] + self.k_lm * ang) # adding y_dot with F_lm if F_lm available
				#print "Local Minimal Detected"
		for b in range(len(v_bord)):
			x_dot = x_dot - v_bord[b] * np.cos(theta_bord[b]) # adding x_dot with F_border
			y_dot = y_dot - v_bord[b] * np.sin(theta_bord[b]) # adding y_dot with F_border
		error[0,0] = x_dot
		error[1,0] = y_dot
		stop_er = np.matrix([[error[0,0], error[1,0]]]).T
		e_norm = np.linalg.norm(stop_er)		
		e_norm_shoot = np.linalg.norm(error)
		if(e_norm < 1.0):
			self.kp = self.lamda1
		else:
			self.kp = self.lamda
		if(self.dribbler_lock == 1) or (self.robot1_ball_flag != 0):
			if(self.position_code == 0) or (self.position_code == 11):
				tresh = self.treshold_error_kickoff
			else:
				tresh = self.treshold_error		
			if(e_norm < tresh):
				error = error * 0
				if(self.flag_status.data[1] != 1):
					self.flag_status.data[1] = 1
					goal_reached_publisher.publish(self.flag_status)
			else:
				self.flag_status.data[1] = 0
			dribbler.x = 0
			dribbler.y = 0
			print "Positioning"
		else:
			tresh = self.treshold_error_shooting		
			self.get_ball_secs = self.get_ball_secs + 1
			rot = np.matrix([[np.cos(self.pose[2,0]), -np.sin(self.pose[2,0])], [np.sin(self.pose[2,0]), np.cos(self.pose[2,0])]])				
			rot_inv = np.linalg.inv(rot)
			ers = rot_inv*(np.matrix([[error[0,0],error[1,0]]]).T)
			
			if(e_norm_shoot < tresh):
				error = error * 0
				if(self.flag_status.data[1] != 1):
					self.flag_status.data[1] = 1
					goal_reached_publisher.publish(self.flag_status)									
				self.kick_ball = 1
				
			else:
				self.flag_status.data[1] = 0			
			
			if(self.pass_ball == 1):
				error[0,0] = 0
				error[1,0] = 0
				
			#dribbler.x = 150
			#dribbler.y = 150
			if(ers[0,0]<-0.1):
				if(ers[0,0]<-1.0):
					dribbler.x = 200
					dribbler.y = 200
				else:
					dribbler.x = 150
					dribbler.y = 150
				self.get_ball_secs = 0							
			elif(ers[1,0]>0.1)or(error[2,0]>0.1):
				if(ers[1,0]>1.0)or(error[2,0]>1.0):
					dribbler.x = 180
					dribbler.y = 80
				else:
					dribbler.x = 150
					dribbler.y = 50
				self.get_ball_secs = 0
			elif(ers[1,0]<-0.1)or(error[2,0]<-0.1):
				if(ers[1,0]<-1.0)or(error[2,0]<-1.0):
					dribbler.x = 80
					dribbler.y = 180
				else:
					dribbler.x = 50
					dribbler.y = 150
				self.get_ball_secs = 0
			elif(ers[0,0]>0.1):
				if(ers[0,0]>1.0):
					dribbler.x = 40
					dribbler.y = 40
				else:
					dribbler.x = 5
					dribbler.y = 5
				self.get_ball_secs = 0		
			else:
				if(self.get_ball_secs < 5):
					dribbler.x = 180
					dribbler.y = 180
				elif(self.get_ball_secs < 10):
					dribbler.x = 120
					dribbler.y = 120
				elif(self.get_ball_secs < 15):
					dribbler.x = 80
					dribbler.y = 80
				else:
					dribbler.x = 40
					dribbler.y = 40											
			self.get_ball_secs += 1
			
		w = self.kp*Jinv*error
		dribbler_publisher.publish(dribbler)
		self.pwm_leveling(w)
	
	def visual_invers_kinematic(self):
		Jinv = np.linalg.inv(self.Jr)
		if(self.strategy == 2): #Strategy ketika bola lebih dekat dengan robot2
			#digunakan ketika robot kehilangan bola
			#robot tidak langsung berhenti, tapi tetap bergerak ke arah dimana bola terdeteksi untuk beberapa saat
			if(np.linalg.norm(self.ball_pos) == 0):
				self.lost_ball_secs += 1
			else:
				self.ball_error = self.ball_pos - self.ball_des1
				self.lost_ball_secs = 0
			if(self.lost_ball_secs > self.lost_ball_thresh):
				self.ball_error = self.ball_error * 0
				
			e_norm = np.linalg.norm(self.ball_error)
			
			if(e_norm < self.treshold_visual_error):				
				self.lock = 1
			elif(self.ball_pos[2,0] > 0.4) or (self.ball_pos[2,0] < -0.4):
				self.lock = 0			
			
			if(self.robot2_ball_flag == 1):
				self.lock = 0
				self.get_ball_secs = self.get_ball_secs + 1				
				if(self.get_ball_secs > 5):
					dribbler.x = self.drib_slow
					dribbler.y = self.drib_slow
				else:
					dribbler.x = self.drib_fast
					dribbler.y = self.drib_fast
				self.ball_error = self.ball_error * 0
				gain = 0
			elif(e_norm < 1.0):
				gain = self.visual_lamda3
			else:
				self.get_ball_secs = 0
				gain = self.visual_lamda2
				dribbler.x = 0
				dribbler.y = 0			
				
			if(self.lock==1):
				self.get_ball_secs = 0
				gain = self.visual_lamda3
				dribbler.x = 100
				dribbler.y = 100
				w = gain*Jinv*self.ball_error
				if(self.ball_error[2,0] <= 0.25 and self.ball_error[2,0] <= -0.25):
					w[0,0] = 0.   
					w[1,0] = 300
					w[2,0] = -320
				if(self.ball_error[2,0] > 0.25):
					w[0,0] = 0.   
					w[1,0] = -350
					w[2,0] = -370
				if(self.ball_error[2,0] > -0.25):
					w[0,0] = 0.   
					w[1,0] = 350
					w[2,0] = 370
				else:					
					w[0,0] = 0.   
					w[1,0] = 350
					w[2,0] = -370
					
				"""self.ball_error[0,0] = 0.5
				self.ball_error[1,0] = 0
				if(self.ball_error[2,0] > 0.2) or (self.ball_error[2,0] < -0.2):
					self.ball_error[2,0] = (abs(self.ball_error[2,0])/self.ball_error[2,0])*(self.ball_error[2,0]+1.0)"""
			else:
				w = gain*Jinv*self.ball_error				
			#print e_norm
			
		else:
			self.ball_error = self.ball_error * 0
			gain = 0
			self.get_ball_secs = 0
			w = gain*Jinv*self.ball_error
		#print self.strategy, self.get_ball_secs, e_norm		
		dribbler_publisher.publish(dribbler)
		self.pwm_leveling(w)
	
	def visual_invers_kinematic1(self):
		Jinv = np.linalg.inv(self.Jr)
		if(self.strategy == 2): #Strategy ketika bola lebih dekat dengan robot2
			#digunakan ketika robot kehilangan bola
			#robot tidak langsung berhenti, tapi tetap bergerak ke arah dimana bola terdeteksi untuk beberapa saat
			if(np.linalg.norm(self.ball_pos) == 0):
				self.lost_ball_secs += 1
			else:
				self.ball_error = self.ball_pos - self.ball_des
				self.lost_ball_secs = 0
			if(self.lost_ball_secs > self.lost_ball_thresh):
				self.ball_error = self.ball_error * 0
				
			e_norm = np.linalg.norm(self.ball_error)
			
			if(self.ball_pos[2,0] > 0.4) or (self.ball_pos[2,0] < -0.4):
				self.lock = 0			
			elif(self.ball_pos[0,0] < 0.7):				
				self.lock = 1			
			
			if(self.robot2_ball_flag != 0):
				self.lock = 0
				self.get_ball_secs = self.get_ball_secs + 1				
				if(self.get_ball_secs > 5):
					dribbler.x = self.drib_slow
					dribbler.y = self.drib_slow
				else:
					dribbler.x = self.drib_fast
					dribbler.y = self.drib_fast
				self.ball_error = self.ball_error * 0
				gain = 0
			elif(e_norm < 1.5):
				gain = self.visual_lamda1
			else:
				self.get_ball_secs = 0
				gain = self.visual_lamda
				dribbler.x = 0
				dribbler.y = 0			
				
			if(self.lock==1):
				self.get_ball_secs = 0
				gain = self.visual_lamda
				dribbler.x = 200
				dribbler.y = 200
				w = gain*Jinv*self.ball_error
				if(self.ball_error[2,0] < 0.15 and self.ball_error[2,0] < -0.15):
					w[0,0] = 0.
					w[1,0] = 270
					w[2,0] = -300
				elif(self.ball_error[2,0] > 0.15 and self.ball_error[2,0] > -0.15):
					w[0,0] = 0.
					w[1,0] = 320
					w[2,0] = -300
				
				elif(self.ball_error[2,0] < 0.2 and self.ball_error[2,0] < -0.2):
					w[0,0] = 0.
					w[1,0] = 270
					w[2,0] = -290
				elif(self.ball_error[2,0] > 0.2 and self.ball_error[2,0] > -0.2):
					w[0,0] = 0.
					w[1,0] = 270
					w[2,0] = -290
				else:
					w[0,0] = 0.
					w[1,0] = 300
					w[2,0] = -320
			else:
				w = gain*Jinv*self.ball_error				
		else:
			self.ball_error = self.ball_error * 0
			gain = 0
			self.get_ball_secs = 0
			w = gain*Jinv*self.ball_error
		dribbler_publisher.publish(dribbler)
		self.pwm_leveling(w)
	
	def corner_visual_invers_kinematic(self):
		Jinv = np.linalg.inv(self.Jr)
		error = self.ball_pos - self.ball_des
		if(np.linalg.norm(self.ball_pos) == 0):
			error = error * 0		
		e_norm = np.linalg.norm(error)
		if(e_norm < self.treshold_visual_error):
			error = error * 0
			if(self.corner_pos == 0):
				self.circular_trajectory_kinematics()
			#self.corner_pos = 1		
		else:
			if(self.corner_pos != 0):
				error = error * 0
			elif((self.pose[2,0]>np.radians(175))or (self.pose[2,0]<np.radians(-150))):
				error = error * 0		
			w = self.visual_lamda1*Jinv*error
			self.pwm_leveling(w)
	
	def robot_invers_kinematic(self):
		drib_spd = np.array([0,0])
		Jinv = np.linalg.inv(self.Jr)
		e_norm = np.linalg.norm(self.er)
		self.get_ball_secs = self.get_ball_secs + 1		
		if(e_norm < self.treshold_error):
			self.er[0,0]= 0.
			self.er[1,0]= 0.
			self.er[2,0]= 0.
		if(self.er[0,0]<-0.1):
			if(self.er[0,0]<-1.0):
				dribbler.x = 200
				dribbler.y = 200
			else:
				dribbler.x = 150
				dribbler.y = 150
			self.get_ball_secs = 0		
		
		elif(self.er[1,0]>0.1)or(self.er[2,0]>0.1):
			if(self.er[1,0]>0.5):
				dribbler.x = 180
				dribbler.y = 150
			else:
				dribbler.x = 150
				dribbler.y = 80
			self.get_ball_secs = 0
		elif(self.er[1,0]<-0.1)or(self.er[2,0]<-0.1):
			if(self.er[1,0]<-0.5):
				dribbler.x = 150
				dribbler.y = 180
			else:
				dribbler.x = 80
				dribbler.y = 150
			self.get_ball_secs = 0
		elif(self.er[0,0]>0.1):
			if(self.er[0,0]>1.0):
				dribbler.x = -20
				dribbler.y = -20
			else:
				dribbler.x = -5
				dribbler.y = -5
			self.get_ball_secs = 0		
		else:
			if(self.get_ball_secs < 10):
				if(abs(self.last_er[0,0]) > 1.0) or (abs(self.last_er[1,0]) > 1.0) or (abs(self.last_er[2,0]) > 1.0):
					dribbler.x = 150
					dribbler.y = 150
				else:
					dribbler.x = 100
					dribbler.y = 100
				#dribbler.x = 150
				#dribbler.y = 150
			else:
				dribbler.x = 20
				dribbler.y = 20			
		self.last_er = self.er
		w = self.lamda*Jinv*self.er
		#print "er", self.er
		dribbler_publisher.publish(dribbler)
		self.pwm_leveling(w)			
	
	def stop_motor(self):
		motor.x = 0 #belakang
		motor.y = 0 #kanan
		motor.z = 0 #kiri		
		dribbler.x = 0
		dribbler.y = 0
		self.get_ball_secs = 0
		pwm_publisher.publish(motor)
		dribbler_publisher.publish(dribbler)
	
	def stop_motion(self):
		motor.x = 0 #belakang
		motor.y = 0 #kanan
		motor.z = 0 #kiri		
		pwm_publisher.publish(motor)		
	
	def main(self):
		if(self.game_mode == 0):
			self.stop_motor()
					
		elif(self.game_mode == 1):
			if(self.robot2_ball_flag != 0) or (self.robot1_ball_flag != 0) or (self.strategy == 1):
				if(self.robot2_ball_flag == 1) and (self.stop_ball < 20):					
					self.stop_motion()
					print "STOP A WHILE ..."
				else:
					self.obstacle_avoidance_kinematic()
					print "GOTO TARGET"
				self.stop_ball += 1
			else:
				if(self.pass_ball == 1):
					self.visual_invers_kinematic()
					print "ATTACK BALL"
				else:
					self.visual_invers_kinematic1()
					print "DIRECT BALL"
				self.get_ball_secs = 0
				self.stop_ball = 0
			if(self.kick_ball == 1):
				if(self.pass_ball == 1):
					#pass ball command
					if(self.robot2_ball_flag == 0):
						self.pass_ball = 0
						pass_complete_publisher.publish(1)
						self.kick_ball = 0
					print "PASS BALL!!!"
				else:
					#shoot ball command
					print "SHOOT BALL!!!"
		
		elif(self.game_mode == 2) or (self.game_mode == 3):
			self.get_ball_secs = 0
			if(self.position_code == 11) and (self.attack == 0):
				if(self.corner_pos == 0):
					self.corner_visual_invers_kinematic()	
				else:
					self.circular_trajectory_kinematics()
			else:
				self.obstacle_avoidance_kinematic()
		
		elif(self.game_mode == 4):
			self.robot_invers_kinematic()
		
		elif(self.game_mode == 5):
			self.corner_visual_invers_kinematic()

if __name__ == "__main__":
	rospy.init_node("robot_kinematics_node_rb2")
	pwm_publisher = rospy.Publisher("robot2/pwm", Vector3, queue_size = 3)
	dribbler_publisher = rospy.Publisher("robot2/dribbler", Vector3, queue_size = 3)
	goal_reached_publisher = rospy.Publisher("robot2/command_flag", Int32MultiArray, queue_size = 1)
	pass_complete_publisher = rospy.Publisher("team/pass_complete_flag", Int32, queue_size = 1)
	server = Server()
	motor = Vector3()
	dribbler = Vector3()
	try:
		rospy.Subscriber("/robot2/cmd_vel", Twist, server.cmd_vel_callback)
		rospy.Subscriber("/robot2/obstacle_position", Float32MultiArray, server.obstacle_position_callback)
		rospy.Subscriber("/robot2/pose", Vector3, server.pose_callback)
		rospy.Subscriber("/robot2/target_pose", Vector3, server.pose_des_callback)
		rospy.Subscriber("/robot2/ball_position", Vector3, server.ball_pos_callback)
		rospy.Subscriber("/robot2/ball_flag", Int32, server.ball_flag_callback)
		rospy.Subscriber("/robot1/ball_flag", Int32, server.robot1_ball_flag_callback)
		rospy.Subscriber("/team/base_station_command", Int32MultiArray, server.game_mode_callback)
		rospy.Subscriber("/team/game_strategy", Int32MultiArray, server.game_strategy_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
