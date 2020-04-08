#!/usr/bin/env python

import rospy
import numpy as np
import random as rd
from numpy.linalg import inv
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

class Server:
	def __init__(self):
		#destination coordinate list
		self.check_destination = np.matrix([[0. ,-2., 0.], [0., 2., 0], [-4., 2., 0], [-4., -2., 0]])
		self.start_destination = np.matrix([[-1.5, 1.5, 0.],[-2.5, -1.5, 0.]])
		#ROBOT1 VARIABLES
		self.pose1 = np.matrix([[0., 1.5, 0.]]).T
		self.pose_des1 = np.matrix([[0., 0., 0.]]).T		
		self.robot_destination1 = np.matrix([[-0.4, 0.7, -75.0, -1.5, 0.3, 0.  ], #kickoff		0
											[-3.5, 2.0, -30., -1.0, 1.5, 0.  ], #freekick		1
											[-3.5, 2.0, -30., -1.0, 1.5, 0.  ], #goalkick 		2
											[-3.0, 2.25, -50., -3.0, 0.5, 0.  ], #throw in		3
											[ 3.0, 2.0,  25., -3.0, 1.5, 0.], #corner kick	4
											[-1.0, 0.0,  0.,  -1.0, 0.0, 0.  ], #penalty		5
											[-4.5, 3.0,  0.,  -4.5, 3.0, 0.  ], #repair			6
											[-2.5, 1.5,  0.,  -2.5, 1.5, 0.  ], #goal			7
											[-2.5, 1.5,  0.,  -2.5, 1.5, 0.  ], #yellow card	8
											[-2.5, 1.5,  0.,  -2.5, 1.5, 0.  ], #red card		9
											[-1.5, 1.5,  0.,  -1.5, 1.5, 0. ], #drop ball		10
											[ 3.86, 2.5, 25., -3.0, 1.5, 0.], #corner left  	11
											[ 2.0,-1.5, -30., -3.0, 0.0, 0.],#corner right 	12
											[ 1.0, 1.0,   0., -4.5, 0.0, 0.] #test pose 		13
										])
		self.ball_position1 = 0.
		self.counter1 = 0
		self.manual_protection1 = 0
		self.last_game_mode1 = 99
		self.robot1_game_mode = 0
		self.robot1_ball_flag = 0			
		#ROBOT2 VARIABLES
		self.pose2 = np.matrix([[4.5, 3., 0.]]).T
		self.pose_des2 = np.matrix([[0., 0., 0.]]).T		
		self.robot_destination2 = np.matrix([[-0.4, -0.7, 75.0, -1.5, -0.3, 0.  ], #kickoff		0
											[-3.5, -2.0, 30., -1.0, -1.5, 0.  ], #freekick		1
											[-3.5, -2.0, 30., -1.0, -1.5, 0.  ], #goalkick 		2
											[-3.0, -2.25, 50., -3.0, -0.5, 0.  ], #throw in		3
											[ 3.0, -2.0,  -25., -3.0, -1.5, 0.], #corner kick	4
											[-1.0, 0.0,  0.,  -1.0, -0.0, 0.  ], #penalty		5
											[-4.5, -3.0,  0.,  -4.5, -3.0, 0.  ], #repair			6
											[-2.5, -1.5,  0.,  -2.5, -1.5, 0.  ], #goal			7
											[-2.5, -1.5,  0.,  -2.5, -1.5, 0.  ], #yellow card	8
											[-2.5, -1.5,  0.,  -2.5, -1.5, 0.  ], #red card		9
											[-1.5, -1.5,  0.,  -1.5, -1.5, 0. ], #drop ball		10
											[ 3.86, -2.5, -25., -3.0, -1.5, 0.], #corner left  	11
											[ 2.0, 0, -30., -2.0, -2.0, -145.],#corner right 	12
											[ 1.0, -1.0,   0., -4.5, -1.0, 0.] #test pose 		13
										])
		self.ball_position2 = 0.
		self.counter2 = 0
		self.manual_protection2 = 0
		self.last_game_mode2 = 99		
		self.robot2_game_mode = 0
		self.robot2_ball_flag = 0
		#===========================================================================================
		self.shoot_position_x = np.array([-1.5, -1.0, 0.0, 1.0, 2.0])
		self.shoot_position_y = np.array([-2.0, -1.5, -1.0 , 0.0, 1.0, 1.5, 2.0])
		self.r1_x_target = 0
		self.r1_y_target = 0
		self.r2_x_target = 0
		self.r2_y_target = 0		
		self.lost_ball_threshold = 30
		#conditional variables
		self.game_mode = 0
		self.attack_mode = 0
		self.position_code = 0
		self.start_status = np.matrix([[0, 0]]).T		
		self.error_threshold = 0.5		
		self.last_strategy = 99
		self.passing_ball = 0
		self.r1_lost_ball_secs = 0
		self.r2_lost_ball_secs = 0
		#===========================================================================================
		print "Program is running"
	
	def reset_start_status_callback1(self, dat):
		self.start_status[0] = dat.data	
	
	def pose_callback1(self, dat):
		self.pose1[0,0] = dat.x
		self.pose1[1,0] = dat.y
		self.pose1[2,0] = dat.z
		self.compute1()
		
	def flag_callback1(self, dat):
		if(dat.data[0] == 1):
			self.counter1 = self.counter1 + 1
			if(self.counter1 > 3):
				self.counter1 = 0
		self.compute1()
	
	def robot1_ball_position_callback(self, dat):
		ball_data = np.matrix([[0., 0., 0.]]).T
		ball_data[0,0] = dat.x
		ball_data[1,0] = dat.y
		ball_data[2,0] = dat.z
		self.ball_position1 = np.linalg.norm(ball_data)
		self.strategy_decider()
	
	def robot1_ball_flag_callback(self, dat):
		self.robot1_ball_flag = dat.data
		if(dat.data != 0):
			if(self.r1_lost_ball_secs > self.lost_ball_threshold):
				self.r1_x_target = self.shoot_position_x[rd.randint(0,4)]
				self.r1_y_target = self.shoot_position_y[rd.randint(0,6)]
		
	def reset_start_status_callback2(self, dat):
		self.start_status[1] = dat.data
	
	def pose_callback2(self, dat):
		self.pose2[0,0] = dat.x
		self.pose2[1,0] = dat.y
		self.pose2[2,0] = dat.z
		self.compute2()
		
	def flag_callback2(self, dat):
		if(dat.data[0] == 1):
			self.counter2 = self.counter2 + 1
			if(self.counter2 > 3):
				self.counter2 = 0
		self.compute2()
	
	def robot2_ball_position_callback(self, dat):
		ball_data = np.matrix([[0., 0., 0.]]).T
		ball_data[0,0] = dat.x
		ball_data[1,0] = dat.y
		ball_data[2,0] = dat.z
		self.ball_position2 = np.linalg.norm(ball_data)
		self.strategy_decider()	
	
	def robot2_ball_flag_callback(self, dat):
		self.robot2_ball_flag = dat.data
		if(dat.data != 0):
			if(self.r2_lost_ball_secs > self.lost_ball_threshold):
				self.r2_x_target = self.shoot_position_x[rd.randint(0,4)]
				self.r2_y_target = self.shoot_position_y[rd.randint(0,6)]
	
	def game_mode_callback(self, dat):
		game_mode = dat.data[0]
		self.position_code = dat.data[1]
		self.attack_mode = dat.data[2]
		if(self.attack_mode == 1):
			self.passing_ball = 0
		if(game_mode == 2):#positioning
			if(self.position_code == 0) or (self.position_code == 14) or (self.position_code == 16):
				self.passing_ball = 2 # 1
			elif(self.position_code == 15) or (self.position_code == 17):
				self.passing_ball = 2			
			else:
				self.passing_ball = 0			
			self.robot1_game_mode = game_mode
			self.robot2_game_mode = game_mode				
		elif(game_mode == 4):
			self.manual_protection1 = 0
			self.manual_protection2 = 0
			if(self.attack_mode == 0):
				self.robot1_game_mode = 4
				self.robot2_game_mode = 0
			else:
				self.robot1_game_mode = 0
				self.robot2_game_mode = 4
		else:
			self.robot1_game_mode = game_mode
			self.robot2_game_mode = game_mode
		self.compute1()
		self.compute2()
	
	def pass_status_flag_callback(self, dat):
		self.passing_ball = 0
		game_strategy.data[1] = 0
	
	def compute1(self):
		if(self.robot1_game_mode == 0):
			if(self.last_game_mode1 != self.robot1_game_mode):
				self.last_game_mode1 = self.robot1_game_mode
				print "Robot 1 STOP"
		
		elif(self.robot1_game_mode == 1): #game Start
			if(self.last_game_mode1 != self.robot1_game_mode):
				self.last_game_mode1 = self.robot1_game_mode
				if(self.attack_mode == 0):
					print "Robot 1 Start"			
				else:
					print "Robot 1 Direct Start"
			if(self.robot1_ball_flag != 0):				
				if(self.passing_ball != 0):
					self.pose_des1[0,0] = self.pose1[0,0]
					self.pose_des1[1,0] = self.pose1[1,0]					
					rot = np.matrix([[np.cos(self.pose1[2,0]), -np.sin(self.pose1[2,0])],[np.sin(self.pose1[2,0]), np.cos(self.pose1[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r1x = np.matrix([[self.pose1[0,0], self.pose1[1,0]]]).T
					r2x = np.matrix([[self.pose2[0,0], self.pose2[1,0]]]).T
					tj = rot_inv*(r2x - r1x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des1[2,0] = self.pose1[2,0] + a
					#print r2x, r1x, a
				else:
					self.pose_des1[0,0] = self.r1_x_target
					self.pose_des1[1,0] = self.r1_y_target
					rot = np.matrix([[np.cos(self.pose1[2,0]), -np.sin(self.pose1[2,0])],[np.sin(self.pose1[2,0]), np.cos(self.pose1[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r1x = np.matrix([[self.pose1[0,0], self.pose1[1,0]]]).T
					gw = np.matrix([[4.5, 0.0]]).T
					tj = rot_inv*(gw - r1x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des1[2,0] = self.pose1[2,0] + a
					#print r2x, gw, a
				pos_des1.x = self.pose_des1[0,0]
				pos_des1.y = self.pose_des1[1,0]
				pos_des1.z = self.pose_des1[2,0]
				pose_des_publisher1.publish(pos_des1)				
				#print "1.PUB COMPLETE"
				
			elif(self.robot2_ball_flag != 0) or (game_strategy.data[0] == 2):
				if(self.passing_ball != 0):
					self.pose_des1[0,0] = self.pose1[0,0]
					self.pose_des1[1,0] = self.pose1[1,0]
					rot = np.matrix([[np.cos(self.pose1[2,0]), -np.sin(self.pose1[2,0])],[np.sin(self.pose1[2,0]), np.cos(self.pose1[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r1x = np.matrix([[self.pose1[0,0], self.pose1[1,0]]]).T
					r2x = np.matrix([[self.pose2[0,0], self.pose2[1,0]]]).T
					tj = rot_inv*(r2x - r1x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des1[2,0] = self.pose1[2,0] + a 					
				else:
					self.pose_des1[0,0] = -3.0
					self.pose_des1[1,0] =  0.0
					self.pose_des1[2,0] =  0.0
				pos_des1.x = self.pose_des1[0,0]
				pos_des1.y = self.pose_des1[1,0]
				pos_des1.z = self.pose_des1[2,0]
				pose_des_publisher1.publish(pos_des1)			
				#print "2.PUB COMPLETE"
			
		elif(self.robot1_game_mode == 2): #positioning
			if(self.start_status[0,0] == 0):
				self.pose_des1[0,0] = self.start_destination[0,0]
				self.pose_des1[1,0] = self.start_destination[0,1]
				self.pose_des1[2,0] = self.start_destination[0,2]*np.pi/180.
				error = self.pose_des1 - self.pose1
				error[2,0] = 0
				e_norm = np.linalg.norm(error)
				if(e_norm<self.error_threshold):
					new_amcl1.pose.pose.position.x = self.start_destination[0,0]
					new_amcl1.pose.pose.position.y = self.start_destination[0,1]
					new_amcl1.pose.pose.position.z = 0.
					new_amcl1.pose.pose.orientation.x = 0.
					new_amcl1.pose.pose.orientation.y = 0.
					new_amcl1.pose.pose.orientation.z = 0.0051
					new_amcl1.pose.pose.orientation.w = 0.99
					new_amcl1.pose.covariance[0] = 0.25
					new_amcl1.pose.covariance[7] = 0.25
					new_amcl1.pose.covariance[35] = 0.068
					self.start_status[0,0] = 1
					error = error * 0			
					amcl_pose_publisher1.publish(new_amcl1)
					start_publisher1.publish(1)
			else:
				if(self.attack_mode == 0): #attack position
					self.pose_des1[0,0] = self.robot_destination1[self.position_code,0]
					self.pose_des1[1,0] = self.robot_destination1[self.position_code,1]
					self.pose_des1[2,0] = self.robot_destination1[self.position_code,2]*np.pi/180.		
				else: #defend position
					self.pose_des1[0,0] = self.robot_destination1[self.position_code,3]
					self.pose_des1[1,0] = self.robot_destination1[self.position_code,4]
					self.pose_des1[2,0] = self.robot_destination1[self.position_code,5]*np.pi/180.					
					
			pos_des1.x = self.pose_des1[0,0]
			pos_des1.y = self.pose_des1[1,0]
			pos_des1.z = self.pose_des1[2,0]
			pose_des_publisher1.publish(pos_des1)
			
		elif(self.robot1_game_mode == 3): #field check
			#get pose desired from cek_desired
			self.pose_des1[0,0] = self.check_destination[self.counter,0]
			self.pose_des1[1,0] = self.check_destination[self.counter,1]
			self.pose_des1[2,0] = self.check_destination[self.counter,2]*np.pi/180.
			#compute error
			error = self.pose_des1 - self.pose1
			e_norm = np.linalg.norm(error)
			if(e_norm<0.2):
				error = error * 0
			pose_des_publisher1.publish(pos_des1)
			
		elif(self.robot1_game_mode == 4): #manual mode
			if(self.manual_protection1 == 0):
				cmd_vel1.linear.x = 0
				cmd_vel1.linear.y = 0
				cmd_vel1.angular.z = 0
				cmd_vel_publisher1.publish(cmd_vel1)			
				self.manual_protection1 = 1
			if(self.last_game_mode1 != self.robot1_game_mode):
				self.last_game_mode1 = self.robot1_game_mode
				print "Robot 1 Manual Mode"
			
		
		elif(self.robot1_game_mode == 5): #circular trajectory mode
			if(self.last_game_mode1 != self.robot1_game_mode):
				self.last_game_mode1 = self.robot1_game_mode
				print "Circular tj Mode"
		
		if(self.robot1_ball_flag == 0):
			self.r1_lost_ball_secs += 1
		else:
			self.r1_lost_ball_secs = 0		
		
		
	def compute2(self):
		if(self.robot2_game_mode == 0):
			if(self.last_game_mode2 != self.robot2_game_mode):
				self.last_game_mode2 = self.robot2_game_mode
				print "Robot 2 STOP"
		
		elif(self.robot2_game_mode == 1): #game Start
			if(self.last_game_mode2 != self.robot2_game_mode):
				self.last_game_mode2 = self.robot2_game_mode
				if(self.attack_mode == 0):
					print "Robot 2 Start"			
				else:
					print "Robot 2 Direct Start"		
			#print self.robot2_ball_flag
			if(self.robot2_ball_flag != 0):				
				if(self.passing_ball != 0):
					self.pose_des2[0,0] = self.pose2[0,0]
					self.pose_des2[1,0] = self.pose2[1,0]					
					rot = np.matrix([[np.cos(self.pose2[2,0]), -np.sin(self.pose2[2,0])],[np.sin(self.pose2[2,0]), np.cos(self.pose2[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r2x = np.matrix([[self.pose2[0,0], self.pose2[1,0]]]).T
					r1x = np.matrix([[self.pose1[0,0], self.pose1[1,0]]]).T
					tj = rot_inv*(r1x - r2x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des2[2,0] = self.pose2[2,0] + a
					#print r2x, r1x, a
				else:
					self.pose_des2[0,0] = self.r2_x_target
					self.pose_des2[1,0] = self.r2_y_target
					rot = np.matrix([[np.cos(self.pose2[2,0]), -np.sin(self.pose2[2,0])],[np.sin(self.pose2[2,0]), np.cos(self.pose2[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r2x = np.matrix([[self.pose2[0,0], self.pose2[1,0]]]).T
					gw = np.matrix([[4.5, 0.0]]).T
					tj = rot_inv*(gw - r2x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des2[2,0] = self.pose2[2,0] + a
					#print r2x, gw, a
				pos_des2.x = self.pose_des2[0,0]
				pos_des2.y = self.pose_des2[1,0]
				pos_des2.z = self.pose_des2[2,0]
				pose_des_publisher2.publish(pos_des2)				
				#print "1.PUB COMPLETE"
				
			elif(self.robot1_ball_flag != 0) or (game_strategy.data[0] == 1):
				if(self.passing_ball != 0):
					self.pose_des2[0,0] = self.pose2[0,0]
					self.pose_des2[1,0] = self.pose2[1,0]
					rot = np.matrix([[np.cos(self.pose2[2,0]), -np.sin(self.pose2[2,0])],[np.sin(self.pose2[2,0]), np.cos(self.pose2[2,0])]])
					rot_inv = np.linalg.inv(rot)
					r2x = np.matrix([[self.pose2[0,0], self.pose2[1,0]]]).T
					r1x = np.matrix([[self.pose1[0,0], self.pose1[1,0]]]).T
					tj = rot_inv*(r1x - r2x)
					a = np.arctan2(tj[1,0],tj[0,0])
					self.pose_des2[2,0] = self.pose2[2,0] + a 					
				else:
					self.pose_des2[0,0] = -3.0
					self.pose_des2[1,0] =  0.0
					self.pose_des2[2,0] =  0.0
				pos_des2.x = self.pose_des2[0,0]
				pos_des2.y = self.pose_des2[1,0]
				pos_des2.z = self.pose_des2[2,0]
				pose_des_publisher2.publish(pos_des2)			
				#print "2.PUB COMPLETE"			
		
		elif(self.robot2_game_mode == 2): #positioning
			if(self.start_status[1,0] == 0):
				self.pose_des2[0,0] = self.start_destination[1,0]
				self.pose_des2[1,0] = self.start_destination[1,1]
				self.pose_des2[2,0] = self.start_destination[1,2]*np.pi/180.
				error = self.pose_des2 - self.pose2
				error[2,0] = 0
				e_norm = np.linalg.norm(error)
				if(e_norm<self.error_threshold):
					new_amcl2.pose.pose.position.x = self.start_destination[1,0]
					new_amcl2.pose.pose.position.y = self.start_destination[1,1]
					new_amcl2.pose.pose.position.z = 0.
					new_amcl2.pose.pose.orientation.x = 0.
					new_amcl2.pose.pose.orientation.y = 0.
					new_amcl2.pose.pose.orientation.z = 0.0051
					new_amcl2.pose.pose.orientation.w = 0.99
					new_amcl2.pose.covariance[0] = 0.25
					new_amcl2.pose.covariance[7] = 0.25
					new_amcl2.pose.covariance[35] = 0.068
					self.start_status[1,0] = 1
					error = error * 0			
					amcl_pose_publisher2.publish(new_amcl2)
					start_publisher2.publish(1)
			else:
				if(self.attack_mode == 0): #attack position
					self.pose_des2[0,0] = self.robot_destination2[self.position_code,0]
					self.pose_des2[1,0] = self.robot_destination2[self.position_code,1]
					self.pose_des2[2,0] = self.robot_destination2[self.position_code,2]*np.pi/180.		
				else: #defend position
					self.pose_des2[0,0] = self.robot_destination2[self.position_code,3]
					self.pose_des2[1,0] = self.robot_destination2[self.position_code,4]
					self.pose_des2[2,0] = self.robot_destination2[self.position_code,5]*np.pi/180.					
					
			pos_des2.x = self.pose_des2[0,0]
			pos_des2.y = self.pose_des2[1,0]
			pos_des2.z = self.pose_des2[2,0]
			pose_des_publisher2.publish(pos_des2)
			
		elif(self.robot2_game_mode == 3): #field check
			#get pose desired from cek_desired
			self.pose_des2[0,0] = self.check_destination[self.counter,0]
			self.pose_des2[1,0] = self.check_destination[self.counter,1]
			self.pose_des2[2,0] = self.check_destination[self.counter,2]*np.pi/180.
			#compute error
			error = self.pose_des2 - self.pose2
			e_norm = np.linalg.norm(error)
			if(e_norm<0.2):
				error = error * 0
			pose_des_publisher2.publish(pos_des2)
			
		elif(self.robot2_game_mode == 4): #manual mode
			if(self.manual_protection2 == 0):
				cmd_vel2.linear.x = 0
				cmd_vel2.linear.y = 0
				cmd_vel2.angular.z = 0
				cmd_vel_publisher2.publish(cmd_vel1)			
				self.manual_protection2 = 1
			if(self.last_game_mode2 != self.robot2_game_mode):
				self.last_game_mode2 = self.robot2_game_mode
				print "Robot 2 Manual Mode"
		
		elif(self.robot2_game_mode == 5): #circular trajectory mode
			if(self.last_game_mode2 != self.robot2_game_mode):
				self.last_game_mode2 = self.robot2_game_mode
				print "Circular tj Mode"
		
		if(self.robot2_ball_flag == 0):
			self.r2_lost_ball_secs += 1
		else:
			self.r2_lost_ball_secs = 0
	
	def strategy_decider(self):
		if(self.passing_ball == 0): #game on
			if(self.ball_position2 <= self.ball_position1):
				if(self.ball_position2 == 0):
					game_strategy.data[0] = 2
				else:
					game_strategy.data[0] = 2
			elif(self.ball_position2 > self.ball_position1):
				if(self.ball_position1 == 0):
					game_strategy.data[0] = 2
				else:
					game_strategy.data[0] = 1						
		elif(self.passing_ball == 1): #robot 1 pass the ball procedure
			game_strategy.data[0] = 1			
		elif(self.passing_ball == 2): #robot 2 pass the ball procedure
			game_strategy.data[0] = 2		
		#print self.passing_ball			
		strategy_publisher.publish(game_strategy)			
		
if __name__ == "__main__":
	rospy.init_node("robot_command_velocity_node")
	#ROBOT 1 PUBLISHER
	cmd_vel_publisher1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
	pose_des_publisher1 = rospy.Publisher("/robot1/target_pose", Vector3, queue_size=3) 
	start_publisher1 = rospy.Publisher("/robot1/start_flag", Int32, queue_size=1) 
	amcl_pose_publisher1 = rospy.Publisher("/robot1/initialpose", PoseWithCovarianceStamped, queue_size=10)
	cmd_vel1 = Twist()
	pos_des1 = Vector3()
	new_amcl1 = PoseWithCovarianceStamped()
	#ROBOT 2 PUBLISHER
	cmd_vel_publisher2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=2)
	pose_des_publisher2 = rospy.Publisher("/robot2/target_pose", Vector3, queue_size=3) 
	start_publisher2 = rospy.Publisher("/robot2/start_flag", Int32, queue_size=1) 
	amcl_pose_publisher2 = rospy.Publisher("/robot2/initialpose", PoseWithCovarianceStamped, queue_size=10)
	cmd_vel2 = Twist()
	pos_des2 = Vector3()
	new_amcl2 = PoseWithCovarianceStamped()
	#TEAM PUBLISHER
	strategy_publisher = rospy.Publisher("/team/game_strategy", Int32MultiArray, queue_size = 2)
	game_strategy = Int32MultiArray()
	game_strategy.data = [0, 0]	
	server = Server()
	try:
		#ROBOT 1 SUBSCRIBER
		rospy.Subscriber("/robot1/pose", Vector3, server.pose_callback1)
		rospy.Subscriber("/robot1/command_flag", Int32MultiArray, server.flag_callback1)	#complete task flag
		rospy.Subscriber("/robot1/start_flag", Int32, server.reset_start_status_callback1)		#reset start status
		rospy.Subscriber("/robot1/ball_flag", Int32, server.robot1_ball_flag_callback)		
		rospy.Subscriber("/robot1/ball_position", Vector3, server.robot1_ball_position_callback) #callback ball position from robot 1
		#ROBOT 2 SUBSCRIBER
		rospy.Subscriber("/robot2/pose", Vector3, server.pose_callback2)
		rospy.Subscriber("/robot2/command_flag", Int32MultiArray, server.flag_callback2)	#complete task flag
		rospy.Subscriber("/robot2/start_flag", Int32, server.reset_start_status_callback2)		#reset start status
		rospy.Subscriber("/robot2/ball_position", Vector3, server.robot2_ball_position_callback) #callback ball position from robot 1
		rospy.Subscriber("/robot2/ball_flag", Int32, server.robot2_ball_flag_callback)		
		#BASE STATION SUBSCRIBER
		rospy.Subscriber("/team/base_station_command", Int32MultiArray, server.game_mode_callback) #[game_mode, positioning_code, attack_mode]
		rospy.Subscriber("/team/pass_complete_flag", Int32, server.pass_status_flag_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
