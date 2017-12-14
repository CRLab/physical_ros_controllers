# semg_ros_controller
A ROS script to receive HTTP commands from an sEMG controlled tablet app and publish received messages over a ROS network

## Setup
Ensure you have node installed. NVM is a good manager for this. Then install npm dependencies
```bash
$ wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh | bash
$ nvm install node
$ npm install
```

## Running
Ensure you have a roscore master running
```bash
$ roscore
```
Then make sure rosbridge websocket is running
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
This opens up a ROSBridge connection on (default) localhost port 9090
Then run the semg listener
```bash
node scripts/semg_ros_node.js
```

## Runtime dependencies
This application expects two things:
- A node on the network is sending and receiving commands data
- The CenterOut android app is running on a local network

Your ROS node will have to be listening/publishing on the following topics:
- Publisher(`/raw_execute_command`, external_controller_msgs.msg.RawExecuteCommand)
- Subscriber(`/valid_commands`, external_controller_msgs.msg.ValidCommands)
- Service(`valid_commands_service`, external_controller_msgs.srv.CurrentCommands)

Change the IP address variable in semg_ros_node.js to the local IP of the host machine
The default port of 5000 is the same as the CenterOut APK so it should not be changed
 