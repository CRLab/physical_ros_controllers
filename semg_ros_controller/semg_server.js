'use strict';

const express         = require('express');
const http            = require('http');
const fs              = require('fs');
const bodyParser      = require('body-parser');
var   roslib 		  = require('roslib');

const SERVER_PORT     = 5000;
const SERVER_IP       = '192.168.1.34';

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


// Creating subscriber
var valid_commands_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/valid_commands',
    messageType : 'external_controller_msgs/ValidCommands'
});

//callback function will update a var array holding valid phrases
valid_commands_topic.subscribe(function(message) {
    console.log('Current valid options from ' + valid_commands_topic.name + ': ' + message.commands);
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

const app = express();

app.use(bodyParser.json()); // support json encoded bodies, using type json only was limiting
app.use(bodyParser.urlencoded({ extended: true })); // support encoded bodies

app.post('/', function (req, res) {
    // var phrase = req.form['phrase'];
    var phrase = req.body.phrase;
    console.log(phrase);
    var command_to_publish = new ROSLIB.Message({
        input_source : "semg",
        command : phrase
    });
    command_publisher.publish(command_to_publish);
    res.send(phrase);

});

app.get('/', function(req,res){
    res.send({'phrases': valid_commands});
});

var httpServer = http.createServer(app);

httpServer.listen(SERVER_PORT, SERVER_IP,function (){
    console.log('semg server ready on ' + SERVER_IP+":"+SERVER_PORT+" via http.");
});
