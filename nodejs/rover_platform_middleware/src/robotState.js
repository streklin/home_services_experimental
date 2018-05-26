#!/usr/bin/env node
'use strict';

const Immutable = require('immutable');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

exports.robotStateManager = function() {

    let instance = null;

    let stateMachine = function(rosNode) {
        this.remoteControlPublisher = rosNode.advertise('/remoteControl', std_msgs.String);
        this.autoMapStatePublisher = rosNode.advertise('/activate_explore', std_msgs.Bool);

        rosNode.subscribe("battery/charge_ratio", std_msgs.Float32, (msg) => {
            this.updateBatteryCharge(msg);
        });

        rosNode.subscribe("/bumper", "ca_msgs/Bumper", (msg) => {
            this.updateBumperState(msg);
        });
    };

    stateMachine.getInstance = function(rosNode) {
        if (instance === null) {
            instance = new stateMachine(rosNode);
        }

        return instance;
    };

    let robotState = Immutable.Map({
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
    });

    stateMachine.prototype.updateBumperState = function(msg) {
        const bumperMap = Immutable.Map({
            leftBumperPressed: msg.is_left_pressed,
            rightBumperPressed: msg.is_right_pressed,
            isLeftSensorActive: msg.is_light_left,
            isFrontLeftSensorActive: msg.is_light_front_left,
            isCenterLeftSensorActive: msg.is_light_center_left,
            isCenterRightSensorActive: msg.is_light_center_right,
            isFrontRightSensorActive: msg.is_light_front_right,
            isRightSensorActive: msg.is_light_right,
        });

        robotState = robotState.merge(bumperMap);
    };

    stateMachine.prototype.getRobotState = function() {
        return robotState.toJS();
    };

    stateMachine.prototype.updateBatteryCharge = function(msg) {
        robotState = robotState.Set('batteryPower', msg.data * 100);
    };

    stateMachine.prototype.toggleAutoMap = function() {
        if (robotState.isAutoMapActive) {
            activateAutoMapBehavior.call(this);
        } else {
            disableAutoMapBehavior.call(this);
        }
    };

    stateMachine.prototype.driveRobot = function(command) {
        const msg = new std_msgs.String();
        msg.data = command;
        this.remoteControlPublisher.publish(msg);
    };

    function activateAutoMapBehavior() {
        let msg = new std_msgs.Bool();
        msg.data = true;
        this.autoMapStatePublisher.publish(msg);
    }

    function disableAutoMapBehavior() {
        let msg = new std_msgs.Bool();
        msg.data = false;
        this.autoMapStatePublisher.publish(msg);
    }

    return stateMachine.getInstance;

};



