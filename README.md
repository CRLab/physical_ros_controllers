# physical_ros_controllers

A series of packages dedicated to allowing convenient use of various assistive input devices into a ROS network

## Launch file configuration
```xml
<launch>
    <group ns="physical_ros_controllers">
        <!-- Controller Manager -->
        <node name="controller_manager" pkg="ros_physical_controller_manager" type="ros_physical_controller_manager_node.py" output="screen"/>
        <!-- Controllers -->
        <node name="alexa_ros_node" pkg="alexa_ros_controller" type="alexa_ros_node.js" output="screen"/>
        <node name="switch_ros_node" pkg="switch_ros_controller" type="switch_ros_node.py" output="screen"/>
        <node name="semg_ros_node" pkg="semg_ros_controller" type="semg_ros_node.js" output="screen"/>
    </group>
</launch>
```

## More Info
- [ros_physical_controller_manager](ros_physical_controller_manager/README.md)
- [external_controller_msgs](external_controller_msgs/README.md)
- [alex_ros_controller](alexa_ros_controller/README.md)
- [semg_ros_controller](semg_ros_controller/README.md)
- [switch_ros_controller](switch_ros_controller/README.md)

## Testing

Start a test valid_commands_service server ("back", "test", "exit") with:
```bash
$ python ros_physical_controller_manager/tests/valid_commands_service_test.py
...in another tab...
$ rosrun semg_ros_controller semg_ros_node.js
```
  
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
    
For more info on the message types look [here](external_controller_msgs/README.md)
