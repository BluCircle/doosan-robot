#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
#add for test tf
import tf.transformations as tr
import numpy as np
import math

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

def shutdown():
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0
 
def get_current_Tbe(posx_zyz):
    angle_zyz = posx_zyz[0][3:]
    euler_zyz = np.array( [num / 180 * math.pi for num in angle_zyz] )
    rotation_mtx = tr.euler_matrix(*euler_zyz, 'rzyz')
    rotation_mtx[0:3,3] = [posx_zyz[0][0],posx_zyz[0][1],posx_zyz[0][2] ]
    euler_xyz = tr.euler_from_matrix(rotation_mtx, 'rxyz')
    angle_xyz = [num * 180 / math.pi for num in euler_xyz]
    return rotation_mtx, angle_xyz

def get_posx_from_mtx(rotation_mtx):
    euler_zyz = tr.euler_from_matrix(rotation_mtx, 'rzyz')
    #rotation_mtx

    pass

  
if __name__ == "__main__":
    rospy.init_node('test_tf_py')
    rospy.on_shutdown(shutdown)
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    set_velx(50,50)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(100,100)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx=[50, 50]
    accx=[100, 100]

    p1= posj(0,0,0,0,0,0)                    #joint
    p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint
    p2= posj(0.0, 0.0, 90.0, 0.0, 0.0, 0.0) #joint

    fCog = [0.0, 0.0, 0.0]
    finertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    p0 =[0.0, 0, 0.0, 0, 0, 0]
    add_tcp("tcp1", p0)
    add_tool("tool1", 5.3, fCog, finertia)
    set_tool("tool1")
    set_tcp("tcp1")
    print(get_tool())
    print(get_tcp())

    movej(p2, vel=100, acc=100)
    posx_zyz = get_current_posx()
    print(posx_zyz)
    
    fCog = [0.0, 0.0, 110.0]
    finertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    p0 =[-12.0, 0, 220.5, 0, -90, 0]
    add_tcp("tcp1", p0)
    add_tool("tool1", 5.3, fCog, finertia)
    set_tool("tool1")
    set_tcp("tcp1")
    print(get_tool())
    print(get_tcp())

    posx_zyz = get_current_posx()
    print(posx_zyz)
    print(get_current_tool_flange_posx())

    Tbe, angle_xyz = get_current_Tbe(posx_zyz)
    print(Tbe)
    print(angle_xyz)

    T_xyz = tr.euler_matrix(45 * math.pi / 180, 0, 0, 'rxyz')
    Tbe = Tbe@T_xyz
    print(Tbe)

    euler_zyz = tr.euler_from_matrix(Tbe, 'rzyz')
    angle_zyz = [num * 180 / math.pi for num in euler_zyz]
    posx_result = [Tbe[0][3],Tbe[1][3],Tbe[2][3],angle_zyz[0],angle_zyz[1],angle_zyz[2]]
    print(posx_result)
    posx_result = posx(posx_result)
    movel(posx_result)


    posx_zyz = get_current_posx()
    print(posx_zyz)
    print(get_current_tool_flange_posx())

    Tbe, angle_xyz = get_current_Tbe(posx_zyz)
    print(Tbe)
    print(angle_xyz)

    T_xyz = tr.euler_matrix(-45 * math.pi / 180, 0, 0, 'rxyz')
    Tbe = Tbe@T_xyz
    print(Tbe)

    euler_zyz = tr.euler_from_matrix(Tbe, 'rzyz')
    angle_zyz = [num * 180 / math.pi for num in euler_zyz]
    posx_result = [Tbe[0][3],Tbe[1][3],Tbe[2][3],angle_zyz[0],angle_zyz[1],angle_zyz[2]]
    print(posx_result)
    posx_result = posx(posx_result)
    movel(posx_result)


    # euler_zyz = np.array(posx_zyz[0][3:])
    # print(euler_zyz)

    # movej(p2, vel=100, acc=100)
    # posx_zyz = get_current_posx()
    # angle_zyz = posx_zyz[0][3:]
    # euler_zyz = np.array( [num / 180 * math.pi for num in angle_zyz] )
    
    # rotation_mtx = tr.euler_matrix(*euler_zyz, 'rzyz')
    # rotation_mtx[0:3,3] = [posx_zyz[0][0],posx_zyz[0][1],posx_zyz[0][2] ]
    
    # euler_xyz = tr.euler_from_matrix(rotation_mtx, 'rxyz')
    # angle_xyz = [num * 180 / math.pi for num in euler_xyz]
    
    # print(posx_zyz)
    # print(euler_zyz)
    # print(rotation_mtx)
    # print(euler_xyz)
    # print(angle_xyz)
