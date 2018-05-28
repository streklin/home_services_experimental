#!/usr/bin/env node
'use strict';

const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const blackboardQueryMsg = rosnodejs.require('rover_platform').srv.blackboardQuery;

exports.lexResponder = function() {

    let instance = null;

    let blackboardClient = null;
    let robotStateService = null;

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
            return new Promise( (resolve, reject) => {
                // currently only checks the battery power of the iRobot Create 2
                let status = robotStateService.status();
                resolve(status);
            });

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
                        let vertex = JSON.parse(response.response);
                        resolve("I am currently in the " + vertex.Vertex.label);
                    })
                    .catch((err) => {
                        console.log(err);
                        reject("I do not know where I am!");
                    });
            });
        },
        "StartMapping": (slots) => {
            return new Promise( (resolve, reject) => {
                robotStateService.activateAutoMapBehavior();
                resolve("OK");
            });
        },
        "StopMapping": (slots) => {
            return new Promise( (resolve, reject) => {
                robotStateService.disableAutoMapBehavior();
                resolve("OK");
            });
        },
        "TravelTo": (slots) => {
            return new Promise( (resolve, reject) => {
                let room = slots.Room;
                robotStateService.travelTo(room);
                resolve("OK");
            });
        }
    };

    let lexResponder = function(rosNode, robotState) {
        robotStateService = robotState;
        blackboardClient = rosNode.serviceClient('/blackboard', 'rover_platform/blackboardQuery');
    };

    lexResponder.getInstance = function(rosNode, robotState) {
        if (instance === null) {
            instance = new lexResponder(rosNode, robotState);
        }

        return instance;
    };

    lexResponder.prototype.processIntent = function(data) {

        console.log(data)

        if (data.intent === undefined || data.intent === null) throw "";
        if (!lexIntents.hasOwnProperty(data.intent)) throw "processIntent - unknown intent";

        return lexIntents[data.intent](data.variables);

    };

    return lexResponder.getInstance;
};