# graspit_switch_controller
Control the BCI experiment using an assistive switch.

Hold down the switch between 0.1 to 3.0 seconds for a "next" command,
and between 3.0 and 7.0 seconds for a "select" command.

Uses an API of three ROS topics:

    /CurrentOptions
    /CurrentlySelectedOption
    /ExecuteOption
    
Where /CurrentOptions is a list of current valid commands
in the interface of the type `CurrentOptions.msg`, which contains a
list of strings.
/CurrentlySelectedOption is a `std_msgs` String of the
command the user is currently "hovering" over, and /ExecuteOption is also a String,
specifying the option that the user has selected for execution.

## How to run

    rosrun graspit_switch_controller switch_controller.py 
    
Feed test data with this:

    rostopic pub -r .1 /CurrentOptions external_controller_msgs/CurrentOptions '{data: ["next grasp", "back", "select grasp", "test"]}'

This spins up a simple ROS publisher to publish test data.
The -r flag denotes the frequency of the publishing in Hz (here, once every 10s)

The `external_controller_msgs/CurrentOptions` specifies the type of msg we want to send,
in this case a custom msg type in the package `external_controller_msgs`

And to keep track of what's going on, use rostopic echo:

    rostopic echo /CurrentlySelectedOption
    rostopic echo /ExecuteOption
