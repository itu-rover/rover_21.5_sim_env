#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy
from control.srv import inout,inoutResponse
import random

def assign(req):
    req.input=bool(random.getrandbits(1))
    return inoutResponse(req.input)


def server():
    rospy.init_node('artag_check')
    s = rospy.Service('artag_check', inout, assign)
    print("Ready to give response") 
    rospy.spin()

if __name__ == "__main__":
    server()