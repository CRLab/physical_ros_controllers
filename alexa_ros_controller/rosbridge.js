'use strict';

var ROSLIB 		  = require('roslib');
var valid_commands;

var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});
 
ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

// ros.on('close', function() {
//     console.log('Connection to websocket server closed.');
// });


// Creating subscriber
var phrases_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/valid_commands',
    messageType : 'external_controller_msgs/ValidCommands'
});

//callback function will update a var array holding valid phrases
phrases_listener.subscribe(function(message) {
    console.log('Current valid options from ' + phrases_listener.name + ': ' + message.commands);
    valid_commands = message.commands; //.split(',')
    // console.log(valid_phrases)
    // listener.unsubscribe();
});


// Calling a service

var valid_commands_service = new ROSLIB.Service({
    ros : ros,
    name : '/valid_commands_service',
    serviceType : 'external_controller_msgs/ValidCommandsService'
});

var request = new ROSLIB.ServiceRequest({
    request : ''
});

valid_commands_service.callService(request, function(result) {
    console.log('Result for service call on '
        + valid_commands_service.name
        + ': '
        + result.commands);
    valid_commands = result.commands;
});

// Creating publisher
var command_publisher = new ROSLIB.Topic({
    ros : ros,
    name : '/raw_execute_command',
    messageType : 'external_controller_msgs/RawExecuteCommand'
});

var currently_selected_publisher = new ROSLIB.Topic({
    ros : ros,
    name : '/currently_selected_command',
    messageType : 'std_msgs/String'
});

//
// var test = new ROSLIB.Message({
//     data : 'test_publish'
// });

// command_publisher.publish(test);

module.exports = {
    returnValidPhrases: function() {
        return valid_commands;
    },
    publishCommand: function(command){
        var command_to_publish = new ROSLIB.Message({
            input_source : 'alexa',
            command : command
        });
        command_publisher.publish(command_to_publish);
    }
};
// //implement function that will return array of current valid phrases
// function returnValidPhrases(){
//     return valid_phrases;
// }
//
// // publishing command (called from lambda.js)
// function publishCommand(command){
//     var command_to_publish = new ROSLIB.Message({
//         data : command
//     });
//     command_publisher.publish(command_to_publish);
// }
//
