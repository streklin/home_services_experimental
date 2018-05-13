import React from 'react';
import './CreateSensors.css';

const createSensors = (props) => {

    let sensorClasses = {
        leftSensorClass: ['sensor leftSensor'],
        leftFrontSensorClass: ['sensor leftFrontSensor'],
        leftCenterSensorClass: ['sensor leftCenterSensor'],
        rightCenterSensorClass: ['sensor rightCenterSensor'],
        rightFrontSensorClass: ['sensor rightFrontSensor'],
        rightSensorClass: ['sensor rightSensor'],
        leftBumperClass: ['bumperSensor left'],
        rightBumperClass: ['bumperSensor right']
    };

    console.log(props.sensors.isLeftSensorActive);

    if (props.sensors.isLeftSensorActive) sensorClasses.leftSensorClass.push('on');
    if (props.sensors.isFrontLeftSensorActive) sensorClasses.leftFrontSensorClass.push('on');
    if (props.sensors.isCenterLeftSensorActive) sensorClasses.leftCenterSensorClass.push('on');
    if (props.sensors.isCenterRightSensorActive) sensorClasses.rightCenterSensorClass.push('on');
    if (props.sensors.isFrontLeftSensorActive) sensorClasses.rightFrontSensorClass.push('on');
    if (props.sensors.isRightSensorActive) sensorClasses.rightSensorClass.push('on');
    if (props.sensors.leftBumperPressed) sensorClasses.leftBumperClass.push('on');
    if (props.sensors.rightBumperPressed) sensorClasses.rightBumperClass.push('on');


    return (
        <div className="CreateSensors">

            <div className="LightSensors">
                <div className={sensorClasses.leftSensorClass.join(' ')}></div>
                <div className={sensorClasses.leftFrontSensorClass.join(' ')}></div>
                <div className={sensorClasses.leftCenterSensorClass.join(' ')}></div>
                <div className={sensorClasses.rightCenterSensorClass.join(' ')}></div>
                <div className={sensorClasses.rightFrontSensorClass.join(' ')}></div>
                <div className={sensorClasses.rightSensorClass.join(' ')}></div>
            </div>

            <div className="Bumpers">
                <div className={sensorClasses.leftBumperClass.join(' ')}></div>
                <div className={sensorClasses.rightBumperClass.join(' ')}></div>
            </div>

            <div className="Create">
                <div className="left-wheel"></div>
                <div className="center-ring"></div>
                <div className="right-wheel"></div>
            </div>

        </div>
    );
};

export default createSensors;