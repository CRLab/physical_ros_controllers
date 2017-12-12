#!/usr/bin/python

import rospy
from std_msgs.msg import String
from external_controller_msgs.msg import ValidCommands
from external_controller_msgs.msg import RawExecuteCommand
from external_controller_msgs.srv import SetInput

class ControllerNode:
    def __init__(self):
        self.alexa = True
        self.semg = True
        self.switch = True

        # rospy.init_node('controller_mgr', anonymous=True)

        rospy.Subscriber('/raw_execute_command', RawExecuteCommand, self.callback)
        self.publisher_network = rospy.Publisher('/execute_command', String, queue_size=10)

        self.input_manager = rospy.Service('set_input_service', SetInput, self.handle_input_status)


    def callback(self, msg):
        source = msg.input_source
        if (source == 'alexa' and self.alexa) or (source == 'semg' and self.semg) or (source == 'switch' and self.switch):
            self.publisher_network.publish(msg.command)


    def handle_input_status(self, srv):
        if srv.input_source == 'alexa': self.alexa = srv.status
        if srv.input_source == 'semg': self.semg = srv.status
        if srv.input_source == 'switch': self.switch = srv.status
        print("Received input manager update. Details: ")
        print(srv.input_source)
        print(srv.status)
        print("Alexa, sEMG, switch status:")
        print(self.alexa)
        print(self.semg)
        print(self.switch)

        return True

if __name__=='__main__':
    rospy.init_node('controller_node')
    controller_node = ControllerNode()
    rospy.spin()



# def listener():
#     rospy.init_node('controller_mgr', anonymous = True)
#     rospy.Subscriber('/ExecuteOption', ExecuteOption, callback)
#
# def publisher():
#     publisher_network = rospy.Publisher('/Network_Command', String, queue_size=10)
#

