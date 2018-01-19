#!/usr/bin/python

import rospy
from std_msgs.msg import String
from external_controller_msgs.msg import RawExecuteCommand
from external_controller_msgs.srv import SetInput, CurrentInputs, CurrentInputsResponse


class ControllerNode:
    def __init__(self):
        self.alexa = True
        self.semg = True
        self.switch = True
        self.mouse = True

        self.raw_execute_command_subscriber = rospy.Subscriber('/raw_execute_command', RawExecuteCommand, self.callback)
        self.publisher_network = rospy.Publisher('/execute_command', String, queue_size=10)

        self.input_manager = rospy.Service('/set_input_service', SetInput, self.handle_input_status)
        self.current_inputs_service = rospy.Service('/current_inputs', CurrentInputs, self.get_current_inputs)

    def callback(self, msg):
        source = msg.input_source
        if (source == 'alexa' and self.alexa) \
                or (source == 'semg' and self.semg) \
                or (source == 'switch' and self.switch)\
                or (source == 'mouse' and self.mouse):
            self.publisher_network.publish(msg.command)

    def handle_input_status(self, srv):
        rospy.loginfo("Updating input status: {}".format(srv))
        if srv.input_source == 'alexa':
            self.alexa = srv.status

        if srv.input_source == 'semg':
            self.semg = srv.status

        if srv.input_source == 'switch':
            self.switch = srv.status

        if srv.input_source == 'mouse':
            self.mouse = srv.status

        rospy.loginfo("Received input manager update. Details: {}, {}".format(srv.input_source, srv.status))
        rospy.loginfo("Statuses: Alexa={} | sEMG={} | switch={} | mouse={}".format(self.alexa, self.semg, self.switch, self.mouse))

        return True

    def get_current_inputs(self, srv):
        rospy.loginfo("Getting current inputs")
        response = CurrentInputsResponse()
        response.inputs = ['alexa', 'semg', 'switch', 'mouse']
        response.statuses = [self.alexa, self.semg, self.switch, self.mouse]
        return response


if __name__ == '__main__':
    rospy.init_node('ros_physical_controller')
    controller_node = ControllerNode()
    rospy.spin()
