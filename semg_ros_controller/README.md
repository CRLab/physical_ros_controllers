#semg_ros_controller

Install dependencies by running `npm install` in this folder



Sets up an express server to send and receive
json phrases from the CenterOut android app via HTTP protocol. Publishes
the selected phrase/command to `/raw_execute_command`

You can feed test data through the `valid_commands` topic and `valid_commands_service` service with:

    rosrun ros_physical_controller_manager valid_commands_service.py

To start:

    node semg_server.js
    
Set the IP address in semg_server.js to the local IP of the machine.
Default port used by the Android app is 5000, so set semg_server.js to that also. 