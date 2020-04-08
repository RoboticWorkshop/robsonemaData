#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Vector3

import sys, select, termios, tty

msg = """
Welcome to -- ROBSONEMA BASE STATION TELEOP KEY --
---------------------------
===== robot2 general command =====
s = Stop

CTRL-C to quit
"""
#skill, position, info, goalkeeper command
position = {
		's':(0.,0.,0.,"Stop"),
		'S':(1.,0.,0.,"Start"),
		'p':(1.,0.,1.,"Direct Start"),
		'q':(2.,0.,0.,"Kick Off Attack"),
		'Q':(2.,0.,1.,"Kick Off Defend"),
		'w':(2.,1.,0.,"Freekick Attack"),
		'W':(2.,1.,1.,"Freekick Defend"),
		'e':(2.,2.,0.,"Goal Kick Attack"),
		'E':(2.,2.,1.,"Goal Kick Defend"),
		'v':(2.,3.,0.,"Throw In Attack"),
		'V':(2.,3.,1.,"Throw In Defend"),				
		'a':(2.,4.,0.,"Corner Kick Attack"),
		'A':(2.,4.,1.,"Corner Kick Defend"),
		'd':(2.,5.,0.,"Penalty"),
		'D':(2.,5.,1.,"Penalty"),
		'z':(2.,6.,0.,"Repair"),
		'Z':(2.,6.,1.,"Repair"),
		'x':(2.,7.,0.,"Goal"),
		'X':(2.,7.,1.,"Goal"),
		'c':(2.,10.,0.,"Drop Ball"),
		'C':(2.,10.,1.,"Drop Ball"),
		'r':(2.,11.,0.,"Corner Kick Left Attack"),
		'R':(2.,11.,1.,"Corner Kick Left Defend"),
		'f':(2.,12.,0.,"Corner Kick Right Attack"),
		'F':(2.,12.,1.,"Corner Kick Right Defend"),
		't':(2.,13.,0.,"Check Position"),
		'T':(2.,13.,1.,"Home"),		
		'o':(2.,14.,0.,"Goal Kick Left"),
		'O':(2.,15.,1.,"Goal Kick Right"),
		'i':(2.,16.,0.,"free Kick Left"),
		'I':(2.,17.,1.,"free Kick Right"),			
		'l':(3.,0.,0.,"Robot 1 Check Mode"),
		'L':(3.,0.,1.,"Robot 2 Check Mode"),		
		'm':(4.,0.,0.,"Robot 1 Manual Mode"),
		'M':(4.,0.,1.,"Robot 2 Manual Mode"),				
		'n':(5.,0.,0.,"Robot 1 Circular Tj"),
		'N':(5.,0.,1.,"Robot 2 Circular Tj"),
		}

flag= {
	'p':(1., 0., "Robot 1 Start Ready"),
	'P':(0., 1., "Robot 2 Start Ready"),
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('position_teleop_node')
	mode_publisher = rospy.Publisher('/team/base_station_command', Int32MultiArray, queue_size = 2)
	#robot1_start_publisher = rospy.Publisher('/robot1/start_flag', Int32, queue_size = 2)
	#robot2_start_publisher = rospy.Publisher('/team/base_station_command', Int32MultiArray, queue_size = 2)
	command = Int32MultiArray()
	command.data = [0,0,0]
	try:
		print(msg)
		while(1):
			key = getKey()
			if key in position.keys():
				mod = position[key][0]
				cod = position[key][1]
				att = position[key][2]
				pos = position[key][3]
				command.data[0] = mod
				command.data[1] = cod
				command.data[2] = att
				mode_publisher.publish(command)			
				print(command, pos)
			elif key in flag.keys():
				r1 = flag[key][0]
				r2 = flag[key][1]
				pos = flag[key][2]
				#if(r1 == 1):
				#	robot1_start_publisher.publish(0)
				print(pos)
			else:
				mod = 0
				cod = 0
				att = 0
				pos = "Stop"
				command.data[0] = mod
				command.data[1] = cod
				command.data[2] = att
				mode_publisher.publish(command)
				if (key == '\x03'):
					break				
		
	except Exception as e:
		print(e)

	finally:
		head = 0
		cod = 0
		od = 0
		command.data[0] = mod 
		command.data[1] = att 
		mode_publisher.publish(command)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
