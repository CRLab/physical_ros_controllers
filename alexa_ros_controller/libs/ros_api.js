'use strict';

const rosnodejs = require('rosnodejs');

var thatModule = module;

rosnodejs.initNode('/physical_ros_controllers/alexa_ros_node', {}).then(function() {
    const nh = rosnodejs.nh;

    var valid_commands;

    const valid_commands_sub = nh.subscribe('/valid_commands', 'external_controller_msgs/ValidCommands', function(message) {
        rosnodejs.log.info('Current valid options from ' + valid_commands_topic.name + ': ' + message.commands);
        valid_commands = message.commands;
    });

    rosnodejs.log.info('Started node, now waiting for service. if you do not see "Result for service call on..." then the service was not found.');

    const valid_commands_service = nh.serviceClient('/valid_commands_service', 'external_controller_msgs/CurrentCommands');
    valid_commands_service.call().then(function(result) {
        rosnodejs.log.info('Result for service call on ' + valid_commands_service.name + ': ' + result.commands);
        valid_commands = result.commands;
    });

    const command_publisher = nh.advertise('/raw_execute_command', 'external_controller_msgs/RawExecuteCommand');

    thatModule.exports = {
        returnValidPhrases: function() {
            return valid_commands;
        },
        publishCommand: function(command){
            var command_to_publish = {
                input_source : 'alexa',
                command : command
            };
            command_publisher.publish(command_to_publish);
        },
        log: function(message) {
            rosnodejs.log.info(message);
        }
    };
});

module.exports.log = function(message) {
    rosnodejs.log.info(message);
};
