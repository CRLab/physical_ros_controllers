# ros_physical_controller_manager
Central management node that handles input from all other input controllers

## Running
Ensure you have a roscore master running
```bash
$ roscore
```
Then run the ros_physical_controller_manager_node
```bash
rosrun ros_physical_controller_manager ros_physical_controller_manager_node.py
```

## Usage
Example usage with rospy (also see set_input_service_test.py):

```python
import rospy
import external_controller_msgs.srv

rospy.wait_for_service('set_input_service')
set_input_test = rospy.ServiceProxy('set_input_service', external_controller_msgs.srv.SetInput)
response = set_input_test('switch', True)
print(response.result)
```

## Runtime dependencies
This application expects the following:
- A node on the network is receiving `/execute_commands`
- Nodes that provide `/raw_execute_command`s
- A UI node that provides `set_input_service` calls

Your controller ROS node will have to be listening/publishing on the following topics:
- Publisher(`/raw_execute_command`, external_controller_msgs.msg.RawExecuteCommand)

Your receiver ROS node will have to be listening/publishing on the following topics
- Subscriber(`/execute_command`, external_controller_msgs.msg.ExecuteCommand)

Your UI ROS node will have to be advertising to the following service:
- Service(`set_input_service`, external_controller_msgs.srv.SetInput)
