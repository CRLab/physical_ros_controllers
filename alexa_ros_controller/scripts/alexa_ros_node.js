#!/usr/bin/env node

'use strict';

const express         = require('express');
const http            = require('http');
const bodyParser      = require('body-parser');
const context 		  = require('aws-lambda-mock-context');

// lambda.js contains the lambda function for Alexa as in http://github.com/alexa/alexa-skills-kit-sdk-for-nodejs
var   lambda          = require('../libs/lambda.js');
const ros_api         = require("../libs/ros_api.js");
const rosnodejs       = require("rosnodejs");

const SERVER_PORT     = 3000;
const SERVER_IP       = 'localhost';

const app = express();

app.use(bodyParser.json({ type: 'application/json' }));

// your service will be available on <YOUR_IP>/alexa
app.post('', function (req, res) {
    var ctx = context();
    lambda.handler(req.body,ctx);
    ctx.Promise
        .then(function(resp) {  return res.status(200).json(resp); })
        .catch(function(err) {  ros_api.log(err); });
});

var httpServer = http.createServer(app);

httpServer.listen(SERVER_PORT, SERVER_IP,function (){
    ros_api.log('Alexa Skill service ready on ' + SERVER_IP + ":" + SERVER_PORT + " via http.");
});

var gracefulShutdown = function() {
    rosnodejs.log.info("Received kill signal, shutting down gracefully.");
    httpServer.close(function() {
        rosnodejs.log.info("Closed out remaining connections.");
    });
};

// Listen for shutdown from ROS
rosnodejs.on('shutdown', gracefulShutdown);