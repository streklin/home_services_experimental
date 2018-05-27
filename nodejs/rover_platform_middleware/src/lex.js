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

            return new Promise((resolve, reject) => {
                let request = new blackboardQueryMsg.Request();
                request.query = JSON.stringify(command);
                blackboardClient.call(request);
                resolve("OK");
            });


        },
        "GetStatus": (slots) => {
            // currently don't have a status node, will need to design one.

        },
        "GetLocation": (slots) => {

            let command = {
                "Command": "queryCurrentLocation"
            };

            return new Promise( (resolve, reject) => {
                let request = new blackboardQueryMsg.Request();
                request.query = JSON.stringify(command);

                blackboardClient.call(request)
                    .then((response) => {
                        resolve("OK");
                    })
                    .catch(() => {
                        reject("I do not know where I am!");
                    });
            });
        },
        "StartMapping": (slots) => {
        },
        "StopMapping": (slots) => {
        },
        "TravelTo": (slots) => {
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