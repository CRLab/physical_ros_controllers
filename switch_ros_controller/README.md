# switch_ros_controller
A Python script that uses ROS to broadcast messages from an Ultimate Switch to a ROS network

## Setup
```bash
$ sudo apt install python-pyaudio
$ pip install pygame --user

```

## Running
Ensure you have a roscore master running
```bash
$ roscore
```
Then run the switch_ros_controller
```bash
rosrun switch_ros_controller switch_ros_node.py
```

## Usage
Hold down the switch between 0.1 to 3.0 seconds for a "next" command,
and between 3.0 and 7.0 seconds for a "select" command.

Feed test data with this:

    rostopic pub -r .1 /valid_commands external_controller_msgs/ValidCommands '{data: ["next grasp", "back", "select grasp", "test"]}'

This spins up a simple ROS publisher to publish test data.
The -r flag denotes the frequency of the publishing in Hz (here, once every 10s)

The `external_controller_msgs/ValidCommands` specifies the type of msg we want to send,
in this case a custom msg type in the package `external_controller_msgs`

And to keep track of what's going on, use rostopic echo:

    rostopic echo /currently_selected_command
    rostopic echo /raw_execute_command

## Runtime dependencies
This application expects the following:
- A node on the network is sending and receiving commands data

Your ROS node will have to be listening/publishing on the following topics:
- Publisher(`/raw_execute_command`, external_controller_msgs.msg.RawExecuteCommand)
- Subscriber(`/valid_commands`, external_controller_msgs.msg.ValidCommands)
- Service(`valid_commands_service`, external_controller_msgs.srv.CurrentCommands)
