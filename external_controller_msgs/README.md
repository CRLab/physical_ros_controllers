# external_controller_msgs
A helper package for the BCI interfaces.


Import into your python script like so:

    from external_controller_msgs.msg import ValidCommands
    from external_controller_msgs.srv import ValidCommandsService

Example of subscribing to the /valid_commands topic (python):

    subscriber = rospy.Subscriber("/valid_commands", ValidCommands, callback)

Example of connecting to the /valid_commands_service server:

    rospy.wait_for_service('valid_commands_service')
    valid_commands_service = rospy.ServiceProxy('valid_commands_service', ValidCommandsService)
    resp = valid_commands_service('')



# ROS topics and services guide

### topic: valid_commands
Contains valid commands at current point in the program. Only updated when there is a change.

topic: `/valid_commands`

ValidCommands.msg format:

    string[] commands
    string parent #command last executed
    string menutype #can be "menu" or "submenu"

### service: valid_commands_service (for initialization)
Returns valid commands at current point in the program.

server: `valid_commands_service`

ValidCommandsService.srv format:

    string request #empty string
    --- #stuff to return
    string[] commands
    string parent #command last executed
    string menutype #can be "menu" or "submenu"




### topic: currently_selected_command
topic: `/currently_selected_command`

type: `std_msgs/String`

### topic: raw_currently_selected_command
topic: `/raw_currently_selected_command`

RawCurrentlySelectedCommand.msg format:

    string input_source
    string command


### topic: raw_execute_command
topic: `/raw_execute_command`

RawExecuteCommand.msg format:

    string input_source
    string command

### topic: execute_command
This is the command that the robot should execute, published by the `ros_physical_controller_manager`

topic: `/execute_command`

type: `std_msgs/String`

### service: set_input_service
Turns inputs on and off, `status` should be True for on and False for off

server: `set_input_service`

SetInput.srv:

    string input_source
    bool status
    ---
    bool result

### topic: valid_environments
topic: `/valid_environments`

ValidEnvironments.msg:

    string[] environments
    string current_environment    

### service: valid_environments_service
topic: `/valid_environments_service`

ValidEnvironmentsService.srv:

    string request
    ---
    string[] environments
    string current_environment



### service: set_environment_service
server: `set_environment_service`

SetEnvironment.srv:

    string environment
    bool status
    ---
    bool result



### topic: valid_inputs
topic: `valid_inputs`

ValidInputs.msg:

    string[] inputs
    string[] statuses

### service: valid_inputs_service
server: `valid_inputs_service`

ValidInputsService.srv:

    string request
    ---
    string[] inputs
    string[] statuses

### topic: crui_bot
topic: `/crui_bot_status`

format: `std_msgs/String`

The message string can be either `ready` or `loading`
