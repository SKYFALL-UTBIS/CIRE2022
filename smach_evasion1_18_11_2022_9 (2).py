#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import smach

import ros_numpy
from utils_evasion import *

import sys, tf, moveit_commander, random
import moveit_msgs.msg

from time import sleep

from moveit_commander import RobotCommander, MoveGroupCommander

########## Functions for takeshi states ##########


def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 


def move_forward(var1):
    move_base(0.3,0,0,3.9)
    var1 = False
    return(True)
def move_backward():
    move_base(-0.15,0,0,1.5)
def turn_left():
    move_base(0.0,0,0.12*np.pi,2)
def turn_right():
    move_base(0.0,0,-0.12*np.pi,2)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito

        right_scan=lectura[:300]
        left_scan=lectura[300:]
        ront_scan=lectura[300:360]

        sd,si=0,0
        if np.mean(left_scan)< 3: si=1
        if np.mean(right_scan)< 3: sd=1

    except:
        sd,si=0,0    

    return si,sd

def gaze_point(x,y,z):
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
 
    
    e =tf.transformations.euler_from_quaternion(rot)
    

    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ

def cuello():
    head = moveit_commander.MoveGroupCommander('head')
    head_pose = head.get_current_joint_values()
    head_pose[0]=1.5
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ    
#head = moveit_commander.MoveGroupCommander('head')
 #   print('robot Estado S_2')
        #####Accion
  #  head.go(np.array((.50*np.pi,0)))
  #head.go(np.array((-.50*np.pi,0)))
  # head.go(np.array((0,0)))


##### Define state INITIAL #####
class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])
        self.counter = 0
        
    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado
        print('robot Estado S_0')
        # se toman las lecturas cuantizadas
        #si,sd=get_lectura_cuant()
        
        #if (si==0 and sd==0): 
            #####Accion
        #	move_forward()
        #	return 'outcome1'

               
        #if (band == 0):
        while not move_forward(True):
           	print("jaja")
          
# band = 1	
        return 'outcome1'
        #else:
         #  band = 0
          # return 'outcome1'
        #if (si==1 and sd==1): return 'outcome4'
        #while True:
           #U = move_forward(True) 
           #if U: 
              #print("jaja")
              #break
        

        #return 'outcome1'



class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
        
    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado
        print('robot Estado S_1')
        #####Accion
        arm =  moveit_commander.MoveGroupCommander('arm')
        joint_goal = arm.get_current_joint_values()
        joint_goal[0] = 0.5
        joint_goal[1] = -1.5
        joint_goal[2] = 0.0
        joint_goal[3] = 0.0
        joint_goal[4] = 0.0
        joint_goal[5] = 0.0
        arm.set_joint_value_target(joint_goal)
        while True: 
           if arm.go(True): 
              print("jaja")
              break
        
        joint_goal = arm.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.0
        joint_goal[2] = 0.0
        joint_goal[3] = -0.3
        joint_goal[4] = 0.0
        joint_goal[5] = 0.0
        arm.set_joint_value_target(joint_goal)
        #arm.go()
        while True: 
           if arm.go(True): 
              print("jaja")
              break
        return 'outcome1'



class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
 #      while not cuello():
 #          print("jaja")
        
       #head.set_joint_value_target(head_pose)
       #ucc=head.go()
       #return succ

       moveit_commander.roscpp_initialize(sys.argv)
       head = moveit_commander.MoveGroupCommander('head')
       head_pose = head.get_current_joint_values()
       head_pose[0]=0.5
       head_pose[1]=0.0
       head.set_joint_value_target(head_pose)
       while True: 
           if head.go(True): 
              print("jaja")
              break
       return 'outcome1'
        



class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_3')
        #####Accion
        moveit_commander.roscpp_initialize(sys.argv)
        head = moveit_commander.MoveGroupCommander('head')
        head_pose = head.get_current_joint_values()
        head_pose[0]=0.0
        head_pose[1]=0.0
        head.set_joint_value_target(head_pose)
        while True: 
           if head.go(True): 
              print("jaja")
              break
        return 'outcome1'
        



class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_4')
        #####Accion
        moveit_commander.roscpp_initialize(sys.argv)
        head = moveit_commander.MoveGroupCommander('head')
        head_pose = head.get_current_joint_values()
        head_pose[0]=-0.5
        head_pose[1]=0.0
        head.set_joint_value_target(head_pose)
        while True: 
           if head.go(True): 
              print("jaja")
              break
        return 'outcome1'
         
         


class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_5')
        #####Accion
        #"sleep()
        return 'outcome1' 



class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_6')
        #####Accion
        #turn_left()
        return 'outcome1' 



class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0        

    def execute(self,userdata):
        print('robot Estado S_7')
        #####Accion
        turn_left()
        return 'outcome1' 



class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_8')
        #####Accion
        move_forward()
        return 'outcome1' 



class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_9')
        #####Accion
        move_forward()
        return 'outcome1' 



class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_10')
        #####Accion
        turn_right()
        return 'outcome1' 



class S11(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_11')
        #####Accion
        turn_right()
        return 'outcome1'                                  
        





def init(node_name):
    global laser, base_vel_pub
    rospy.init_node(node_name)
    loop = rospy.Rate(1)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    laser = Laser()  
    band = 0
    moveit_commander.roscpp_initialize(sys.argv)
    #arm =  moveit_commander.MoveGroupCommander('arm') 
    #head = moveit_commander.MoveGroupCommander('head')
    


#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0
    sm.userdata.clear = False
    

    with sm:
        #State machine for evasion
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_1','outcome2':'s_0','outcome3':'END'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_3','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_4','outcome2':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_5','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_5','outcome2':'END'})
        #smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_1','outcome2':'END'})
        #smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        #smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_9','outcome2':'END'})
        #smach.StateMachine.add("s_9",   S9(),  transitions = {'outcome1':'s_10','outcome2':'END'})
        #smach.StateMachine.add("s_10",  S10(), transitions = {'outcome1':'s_11','outcome2':'END'})
        #smach.StateMachine.add("s_11",  S11(), transitions = {'outcome1':'s_0','outcome2':'END'})

    outcome = sm.execute()


























