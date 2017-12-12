"use strict";

var   Alexa           = require("alexa-sdk");
var   rosbridge       = require('./rosbridge.js'); //for communication with ROS

const APP_ID = undefined;
const HELP_MESSAGE = 'You can give me any command you see through the Graspit interface. Please tell me your command.';
const HELP_REPROMPT = 'What can I help you with?';
const STOP_MESSAGE = 'Goodbye!';


var handlers = {
    "HelloIntent": function () {
        this.response.speak("What's up");
        this.emit(':responseReady');
    },
    "AMAZON.HelpIntent": function () {
        this.response.speak(HELP_MESSAGE).listen(HELP_REPROMPT);
        //this.response.speak(HELP_MESSAGE); //link to help text in YAML
        this.emit(':responseReady');
    },
    "AMAZON.CancelIntent": function () {
        this.response.speak(STOP_MESSAGE);
        this.emit(':responseReady');
    },

    "SendCommandIntent": function () {
        const commandSlot = this.event.request.intent.slots.Command;
        var commandName;
        var validPhrases = rosbridge.returnValidPhrases();
        console.log("phrases: " + validPhrases);

        if(commandSlot && commandSlot.value){
            commandName = commandSlot.value;

            console.log(commandName);
            if(validPhrases.indexOf(commandName) != -1){
                this.response.speak("Command received");
                console.log("valid command received: " + commandName);
                rosbridge.publishCommand(commandName);

            }
            else{
                this.response.speak("Invalid command received");
            }
        }
        else{
            this.response.speak("No command received");
        }

        //const command = myCommandList[commandName]

        this.emit(':responseReady');
    }


};

//what port does this use...3000?
exports.handler = function (event, context, callback) { //for Lambda AWS
    var alexa = Alexa.handler(event, context);
    alexa.appId = undefined;
    alexa.registerHandlers(handlers);
    alexa.execute();
};