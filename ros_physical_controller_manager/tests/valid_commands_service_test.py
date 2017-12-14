#!/usr/bin/python

import rospy
from external_controller_msgs.msg import ValidCommands
from external_controller_msgs.srv import ValidCommandsService

commands = ["back", "test", "exit"]


def handle_commands(srv):
    return {'commands': commands, 'parent': "test_parent", 'menutype': "test_menutype"}


valid_commands_service = rospy.Service('valid_commands_service', ValidCommandsService, handle_commands)
valid_commands_topic = rospy.Publisher('/valid_commands', ValidCommands, queue_size=10)

rospy.init_node('valid_commands_test')
r = rospy.Rate(0.1)  # 10hz
msg = ValidCommands()
msg.commands = commands
msg.parent = "test_parent"
msg.menutype = "test_menutype"
while not rospy.is_shutdown():
    valid_commands_topic.publish(msg)
    r.sleep()
