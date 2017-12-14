# alexa_ros_controller
Submits received voice command actions to a ROS network from an Amazon Alexa.

## Setup
Ensure you have node installed. NVM is a good manager for this. Make sure you also have [ngrok](https://ngrok.com/) installed. Then install npm dependencies
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

Because Amazon expects an HTTPS server, ngrok is a good candidate for this. 
```bash
$ screen
$ ngrok http 3000
ctrl a+d
```
port 3000. Amazon Alexa json's are forwarded there via ngrok, so make sure that's set up correctly on the Amazon Developer Portal by paste the ngrok https link onto the Alexa Skill "Configuration" page.

Then run the alex_ros_node
```bash
node scripts/alexa_ros_node.js
```

## Runtime dependencies
This application expects the following:
- A node on the network is sending and receiving commands data

Your ROS node will have to be listening/publishing on the following topics:
- Publisher(`/raw_execute_command`, external_controller_msgs.msg.RawExecuteCommand)
- Subscriber(`/valid_commands`, external_controller_msgs.msg.ValidCommands)
- Service(`valid_commands_service`, external_controller_msgs.srv.CurrentCommands)

## Description
The script `libs/ros_api.js` will initialize all ros subscribers/publishers and therefore must only be run after rosbridge has been initialized.