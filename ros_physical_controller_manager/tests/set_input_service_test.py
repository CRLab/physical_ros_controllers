#!/usr/bin/python

import rospy
from external_controller_msgs.srv import SetInput

rospy.wait_for_service('set_input_service')
semg_true = rospy.ServiceProxy('set_input_service', SetInput)
respl = semg_true('switch', True)
print(respl.result)
