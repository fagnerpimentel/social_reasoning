#!/usr/bin/env python
import copy
import numpy
from util import *

def get_navigation_type():
    return 'APPROACH'

def get_proxemic():
    return 1.2

def get_approach_pose(person):
    approach_pose = copy.deepcopy(person.pose)
    dist = get_proxemic()

    [yaw,_,_] = quaternion_to_euler(
        person.pose.orientation.x,person.pose.orientation.y,
        person.pose.orientation.z,person.pose.orientation.w)

    type = get_navigation_type()
    if(type == 'APPROACH'):
        yaw = yaw
    elif(type == 'FOLLOW'):
        yaw = yaw + numpy.pi
    elif(type == 'SIDEBYSIDE'):
        yaw = yaw + numpy.pi/2
    else:
        dist = 0

    dx = dist * numpy.cos(yaw)
    dy = dist * numpy.sin(yaw)

    approach_pose.position.x += dx
    approach_pose.position.y += dy

    alfa = numpy.arctan2(
        approach_pose.position.y - person.pose.position.y,
        approach_pose.position.x - person.pose.position.x)
    q = euler_to_quaternion(0,0,alfa)
    approach_pose.orientation.x = q[0]
    approach_pose.orientation.y = q[1]
    approach_pose.orientation.z = q[2]
    approach_pose.orientation.w = q[3]

    return approach_pose
