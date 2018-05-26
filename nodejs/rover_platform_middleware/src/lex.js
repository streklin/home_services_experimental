#!/usr/bin/env node
'use strict';

const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const blackboardQueryMsg = rosnodejs.require('rover_platform').srv.blackboardQuery;

exports.lexResponder = function() {


    let instance = null;

    let blackboardClient = null;

    let lexIntents = {
        "LabelPlace": (slots) => {

            let command = {
                "Command": "setCurrentLabel",
                "Vertex": {
                    "label": slots.Room
                }
            };


            let request = new blackboardQueryMsg.Request();
            request.query = JSON.stringify(command);
            blackboardClient.call(request);

            return "OK";
        },
        "GetStatus": (slots) => {
            return "OK";
        },
        "GetLocation": (slots) => {
            return "OK";
        },
        "StartMapping": (slots) => {
            return "OK";
        },
        "StopMapping": (slots) => {
            return "OK";
        },
        "TravelTo": (slots) => {
            return "OK";
        }
    };

    let lexResponder = function(rosNode, robotState) {
        this.robotState = robotState;
        blackboardClient = rosNode.serviceClient('/blackboard', 'rover_platform/blackboardQuery');
    };

    lexResponder.getInstance = function(rosNode, robotState) {
        if (instance === null) {
            instance = new lexResponder(rosNode, robotState);
        }

        return instance;
    };

    lexResponder.prototype.processIntent = function(data) {

        if (data.intent === undefined || data.intent === null) throw "";
        if (!lexIntents.hasOwnProperty(data.intent)) throw "processIntent - unknown intent";

        return lexIntents[data.intent](data.variables);

    };

    return lexResponder.getInstance;
};