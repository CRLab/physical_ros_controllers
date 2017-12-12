'use strict';

const express         = require('express');
const http            = require('http');
const fs              = require('fs');
const bodyParser      = require('body-parser');
const context 		  = require('aws-lambda-mock-context');
var   roslib 		  = require('roslib');

// lambda.js contains the lambda function for Alexa as in http://github.com/alexa/alexa-skills-kit-sdk-for-nodejs
var   lambda          = require('./lambda.js');

const SERVER_PORT     = 3000;
const SERVER_IP       = 'localhost';

//
// // SSL Certificate stuff for https
// var privateKey  = fs.readFileSync(<PATH TO YOUR privateKey file>, 'utf8');
// var certificate = fs.readFileSync(<PATH TO YOUR certificate file>, 'utf8');
// var ca = fs.readFileSync(<PATH TO YOUR crt file>).toString();
// var credentials = {key: privateKey, cert: certificate,ca:ca};

const app = express();

app.use(bodyParser.json({ type: 'application/json' }));



// your service will be available on <YOUR_IP>/alexa
app.post('', function (req, res) {
    var ctx = context();
    lambda.handler(req.body,ctx);
    ctx.Promise
        .then(function(resp) {  return res.status(200).json(resp); })
        .catch(function(err) {  console.log(err); });
});

var httpServer = http.createServer(app);

httpServer.listen(SERVER_PORT, SERVER_IP,function (){
    console.log('Alexa Skill service ready on ' + SERVER_IP+":"+SERVER_PORT+" via http.");
});
