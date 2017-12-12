# ros_physical_controller_manager

Handles input from various controllers.

Listens on
`/raw_execute_command` topic. If a message's input source has a status of True, it will publish
that to `/execute_command`

To change the state of input sources, it hosts  a service,
`set_input_service`, where you can send a SetInput.srv that is set up as follows:

    string input_source
    bool status
    ---
    bool result #for now, it returns true if successful
    
Example usage with rospy (also see set_input_service_test.py):

    rospy.wait_for_service('set_input_service')
    set_input_test = rospy.ServiceProxy('set_input_service', SetInput)
    respl = set_input_test('switch', True)
    print(respl.result)