#!/usr/bin/env node
'use strict';

const express = require('express');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

const app = express();

let robotState = {
    isAutoMapActive: false,
    leftBumperPressed: false,
    rightBumperPressed: false,
    wheelsDropped: false,
    spotLightOn: false,
    debrisLightOn: false,
    isLeftSensorActive: false,
    isFrontLeftSensorActive: false,
    isCenterLeftSensorActive: false,
    isCenterRightSensorActive: false,
    isFrontRightSensorActive: false,
    isRightSensorActive: false,
    batteryPower: 0.0
};

// update this later
app.use(function (req, res, next) {
    res.header("Access-Control-Allow-Origin", "*");
    res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
    next();
});

let remoteControlPublisher = null;
let autoMapStatePublisher = null;
let activateBehaviorPublisher = null;
let deActivateBehaviorPublisher = null;

const createDriveCmd = (cmd) => {
    const msg = new std_msgs.String();
    msg.data = cmd;
    remoteControlPublisher.publish(msg);
};

// send forward twist command
app.get('/robot/drive/forward', (req, res) => {
    createDriveCmd('F');
    res.send("OK!");
});

// send backwards twist control
app.get('/robot/drive/backward', (req, res) => {
    createDriveCmd('B');
    res.send("OK");
});

// send right twist
app.get('/robot/drive/right', (req, res) => {
    createDriveCmd('R');
    res.send("OK");
});

// send left twist
app.get('/robot/drive/left', (req, res) => {
    createDriveCmd('L');
    res.send("OK");
});

// send stop
app.get('/robot/drive/stop', (req, res) => {
    createDriveCmd('S');
    res.send("OK");
});

app.get('/robot/images/map', (req, res) => {
    res.sendFile('/home/gene/catkin_ws/src/rover_platform/mapImage.png');
});

app.get('/robot/images/camera', (req, res) => {
    res.sendFile('/home/gene/catkin_ws/src/rover_platform/camImage.png');
});

function activateAutoMapBehavior() {
    const msg = new std_msgs.String();
    msg.data = 'remote_control_behavior/automap_behavior';
    activateBehaviorPublisher.publish(msg);
}

function disableAutoMapBehavior() {
    const msg = new std_msgs.String();
    msg.data = 'remote_control_behavior/automap_behavior';
    deActivateBehaviorPublisher.publish(msg);
}

function enableRemoteControl() {
    const msg = new std_msgs.String();
    msg.data = 'remote_control_behavior';
    activateBehaviorPublisher.publish(msg);
}

function disableRemoteControl() {
    const msg = new std_msgs.String();
    msg.data = 'remote_control_behavior';
    deActivateBehaviorPublisher.publish(msg);
}

app.get('/robot/state/toggleAutoMap', (req, res) => {
    robotState.isAutoMapActive = !robotState.isAutoMapActive;

    if (robotState.isAutoMapActive) {
        activateAutoMapBehavior();
        disableRemoteControl();
    } else {
        disableAutoMapBehavior();
        enableRemoteControl();
    }

    res.send("OK");

});

app.get('/robot/state/getState', (req, res) => {
    res.send(robotState);
});

rosnodejs.initNode('/middleware', { onTheFly: true})
    .then((rosNode) => {
        // register publishers
        remoteControlPublisher = rosNode.advertise('/remoteControl', std_msgs.String);
        autoMapStatePublisher = rosNode.advertise('/setExploreState', std_msgs.Bool);
        activateBehaviorPublisher = rosNode.advertise('activateBehavior', std_msgs.String);
        deActivateBehaviorPublisher = rosNode.advertise('deActivateBehavior', std_msgs.String);

        rosNode.subscribe("battery/charge_ratio", std_msgs.Float32, (msg) => {
            robotState.batteryPower = msg.data * 100;
        });

        rosNode.subscribe("/bumper", "ca_msgs/Bumper", (msg) => {
            robotState.leftBumperPressed = msg.is_left_pressed;
            robotState.rightBumperPressed = msg.is_right_pressed;
            robotState.isLeftSensorActive = msg.is_light_left;
            robotState.isFrontLeftSensorActive = msg.is_light_front_left;
            robotState.isCenterLeftSensorActive = msg.is_light_center_left;
            robotState.isCenterRightSensorActive = msg.is_light_center_right;
            robotState.isFrontRightSensorActive = msg.is_light_front_right;
            robotState.isRightSensorActive = msg.is_light_right;
        });


        app.listen(8080, () => console.log('Example app listening on port 8080!'));
    });
