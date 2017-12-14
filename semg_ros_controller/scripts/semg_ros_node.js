#!/usr/bin/env node

'use strict';

const express         = require('express');
const http            = require('http');
const bodyParser      = require('body-parser');
var   rosnodejs       = require('rosnodejs');
var   ip              = require("ip");

const SERVER_PORT     = 5000;
// const SERVER_IP       = '192.168.1.34';
const SERVER_IP       = ip.address();

rosnodejs.initNode('/physical_ros_controllers/semg_ros_node', {}).then(function() {
    const nh = rosnodejs.nh;

    var valid_commands;

    const valid_commands_sub = nh.subscribe('/valid_commands', 'external_controller_msgs/ValidCommands', function(message) {
        rosnodejs.log.info('Current valid options from ' + '/valid_commands' + ': ' + message.commands);
        valid_commands = message.commands; //.split(',')
    });

    rosnodejs.log.info('Started node, now waiting for service. if you do not see "Result for service call on..." then the service was not found.');

    const valid_commands_service = nh.serviceClient('valid_commands_service', 'external_controller_msgs/CurrentCommands');
    valid_commands_service.call()
        .then(function(result) {
            rosnodejs.log.info('Result for service call on valid_commands_service: ' + result.commands);
            valid_commands = result.commands;
        })
        .catch(function(err) {
            rosnodejs.log.error("Could not find service 'valid_commands_service': " + err);
        });

    const command_publisher = nh.advertise('/raw_execute_command', 'external_controller_msgs/RawExecuteCommand');

    const app = express();

    app.use(bodyParser.json()); // support json encoded bodies, using type json only was limiting
    app.use(bodyParser.urlencoded({ extended: true })); // support encoded bodies

    app.post('/', function (req, res) {
        var phrase = req.body.phrase;
        rosnodejs.log.info("About to publish: " + phrase);
        var command_to_publish = {
            input_source : "semg",
            command : phrase
        };
        command_publisher.publish(command_to_publish);
        res.send(phrase);
    });

    app.get('/', function(req,res){
        res.send({'phrases': valid_commands});
    });

    var httpServer = http.createServer(app);

    httpServer.listen(SERVER_PORT, SERVER_IP, function (){
        rosnodejs.log.info('semg server ready on ' + SERVER_IP + ":" + SERVER_PORT + " via http.");
    });

    var gracefulShutdown = function() {
        rosnodejs.log.info("Received kill signal, shutting down gracefully.");
        httpServer.close(function() {
            rosnodejs.log.info("Closed out remaining connections.");
        });
    };

    // Listen for shutdown from ROS
    rosnodejs.on('shutdown', gracefulShutdown);
});
