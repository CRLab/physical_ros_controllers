#semg_ros_controller

Sets up an express server to send and receive
json phrases from the CenterOut android app via HTTP protocol. Publishes
the selected phrase/command to `/raw_execute_command`

To start:

    node semg_server.js
    
Set the IP address to the local IP of the machine.
Default port used is 5000. 