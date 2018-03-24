#!/usr/bin/env python

import rospy



class Master:
    """Master Node
    
    Processes commands from alexa and sends appropriate actions to arduino_motor

    """
    # set up constants
    def __init__(self):
        # create a new node
        rospy.init_node('master', anonymous=True)
        
