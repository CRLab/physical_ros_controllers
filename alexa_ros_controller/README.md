# alexa_ros_controller
Submits received voice command actions to a ROS network from an Amazon Alexa.

Install dependencies by running `npm install` in this folder

First start up ROSBridge Websocket Server:
    
    roslaunch rosbridge_server rosbridge_websocket.launch

This opens up a ROSBridge connection on (default) localhost port 9090
which then the Javascript file `rosbridge.js` can connect to.

Then start up ngrok:

    ngrok http 3000

Run like so:
    
    node express_server.js
    
This listens for json's sent by Amazon Alexa on localhost
port 3000. Amazon Alexa json's are forwarded there via ngrok, so make sure
that's set up correctly on the Amazon Developer Portal by paste the ngrok https link onto the Alexa Skill "Configuration" page.

You can feed test data through the `valid_commands` topic and `valid_commands_service` service with:

    rosrun ros_physical_controller_manager valid_commands_service.py
    


### API

Uses an API of three ROS topics:

    /CurrentOptions
    /CurrentlySelectedOption
    /ExecuteOption

It listens on the /CurrentOptions topic for valid interface commands.
Once a valid voice command is received, it publishes that command to /ExecuteOption.

The voice interface doesn't publish anything on /CurrentlySelectedOption.

Listen to what it does:
    
    rostopic echo /ExecuteOption
    
If having problems with running express_server.js (e.g. can't find dependencies)
run `npm init` on the project directory (using default options),
then delete the node_modules folder,
and run `npm install` in the project directory. It will automatically reinstall all the required dependencies.