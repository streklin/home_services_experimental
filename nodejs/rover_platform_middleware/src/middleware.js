#!/usr/bin/env node
'use strict';


const express = require('express');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const robotStateManager = require('./robotState');
const bodyParser = require('body-parser');
const awsLex = require('./lex');

const app = express();
let robotState = null;
let lexHandler = null;

// update this later
app.use(function (req, res, next) {
    res.header("Access-Control-Allow-Origin", "*");
    res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
    next();
});

app.use(bodyParser.json());


// send forward twist command
app.get('/robot/drive/forward', (req, res) => {
    robotState.driveRobot('F');
    res.send("OK!");
});

// send backwards twist control
app.get('/robot/drive/backward', (req, res) => {
    robotState.driveRobot('B');
    res.send("OK");
});

// send right twist
app.get('/robot/drive/right', (req, res) => {
    robotState.driveRobot('R');
    res.send("OK");
});

// send left twist
app.get('/robot/drive/left', (req, res) => {
    robotState.driveRobot('L');
    res.send("OK");
});

// send stop
app.get('/robot/drive/stop', (req, res) => {
    robotState.driveRobot('S');
    res.send("OK");
});

app.get('/robot/images/map', (req, res) => {
    res.sendFile('/home/gene/catkin_ws/src/rover_platform/mapImage.png');
});

app.get('/robot/images/camera', (req, res) => {
    res.sendFile('/home/gene/catkin_ws/src/rover_platform/camImage.png');
});

app.get('/robot/state/toggleAutoMap', (req, res) => {
    robotState.toggleAutoMap();
    res.send("OK");
});

app.get('/robot/state/getState', (req, res) => {
    const state = robotState.getRobotState();
    res.send(state);
});

app.post('/robot/chat/lex', (req, res) => {
    let promise = lexHandler.processIntent(req.body);
    promise
        .then((result) => {
            res.send(result);
        })
        .catch((err) => {
            res.send("There was a problem processing your request.");
        });
});

rosnodejs.initNode('/middleware', { onTheFly: true})
    .then((rosNode) => {
        robotState = robotStateManager.robotStateManager()(rosNode);
        lexHandler = awsLex.lexResponder()(rosNode, robotState);
        app.listen(8080, () => console.log('Example app listening on port 8080!'));
    });

