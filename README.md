# physical_ros_controllers


Start a test valid_commands_service server ("back", "test", "exit" )using:

    rosrun ros_physical_controller_manager valid_commands_service.py
    
### ROS topics and services used by this package

Needs these topics from backend:

    valid_commands
    
Need these services from backend:
    
    valid_commands_service    

Publishes these topics:

    /raw_execute_command
    /execute_command
    /currently_selected_command
    /raw_currently_selected_command
    /valid_inputs

    
Creates the following service servers:

    /set_input_service
    /valid_inputs_server

### topic: valid_commands
Contains valid commands at current point in the program. Only updated when there is a change.

topic: `/valid_commands`

ValidCommands.msg format:

    string[] commands
    string parent #command last executed
    string menutype #can be "menu" or "submenu"

### service: valid_commands_service (for initialization)
Returns valid commands at current point in the program. Each controller calls the server at startup
to get a list of commands. (otherwise, the user may try to select something before the commands get published on the topic)

server: `valid_commands_service`

ValidCommandsService.srv format:

    string request #empty string
    --- #stuff to return
    string[] commands
    string parent #command last executed
    string menutype #can be "menu" or "submenu"


### topic: raw_currently_selected_command


### topic: currently_selected_command
Applies to the Ultimate Switch and mouse. Updates the topic with whatever command the user is currently hovering on.
Relevant for updating the UI.

topic: `/currently_selected_command`

type: `std_msgs/String`
    
### topic: raw_execute_command
topic: `/raw_execute_command`

RawExecuteCommand.msg format:

    string input_source
    string command
    
### topic: execute_command
This is the command that goes to the backend (aka what the robot should execute), published by the `ros_physical_controller_manager` after it
has filtered for disabled inputs.

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

