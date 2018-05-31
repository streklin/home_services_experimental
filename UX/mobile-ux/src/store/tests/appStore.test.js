import reducer from '../reducers/appStore';
import React from 'react';
import * as actionTypes from '../actions';

describe('appStore Reducer', () => {

    it('toggles the side menu when TOGGLE_SIDE_MENU action is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.TOGGLE_SIDE_MENU
        });

        expect(state.isMenuOpen).toBe(true);

        state = reducer(state, {
            type: actionTypes.TOGGLE_SIDE_MENU
        });

        expect(state.isMenuOpen).toBe(false);
    });

    it('toggles the automap flag when TOGGLE_AUTO_MAP action is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.TOGGLE_AUTO_MAP
        });

        expect(state.robotState.isAutoMapActive).toBe(true);

        state = reducer(state, {
            type: actionTypes.TOGGLE_AUTO_MAP
        });

        expect(state.robotState.isAutoMapActive).toBe(false);
    });

    it('updates the sensor states when UPDATE_SENSOR_STATES is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.UPDATE_SENSOR_STATES,
            data: {
                leftBumperPressed: true,
                rightBumperPressed: true,
                wheelsDropped: true,
                spotLightOn: true,
                debrisLightOn: true,
                isLeftSensorActive: true,
                isFrontLeftSensorActive: true,
                isCenterLeftSensorActive: true,
                isCenterRightSensorActive: true,
                isFrontRightSensorActive: true,
                isRightSensorActive: true,
                batteryPower: 95.0
            }
        });

        expect(state.robotState.leftBumperPressed).toBe(true);
        expect(state.robotState.rightBumperPressed).toBe(true);
        expect(state.robotState.wheelsDropped).toBe(true);
        expect(state.robotState.spotLightOn).toBe(true);
        expect(state.robotState.debrisLightOn).toBe(true);
        expect(state.robotState.isLeftSensorActive).toBe(true);
        expect(state.robotState.isFrontLeftSensorActive).toBe(true);
        expect(state.robotState.isCenterLeftSensorActive).toBe(true);
        expect(state.robotState.isCenterRightSensorActive).toBe(true);
        expect(state.robotState.isFrontRightSensorActive).toBe(true);
        expect(state.robotState.isRightSensorActive).toBe(true);
        expect(state.robotState.batteryPower).toEqual(95.0);

    });

    it('updates the camera url when UPDATE_CAMERA_URL is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.UPDATE_CAMERA_URL,
            data: "/map?t=10"
        });

        expect(state.camUrl).toEqual("/map?t=10");
    });

    it('updates the map url when UPDATE_MAP_URL is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.UPDATE_MAP_URL,
            data: '/map?t=10'
        });

        expect(state.mapUrl).toEqual('/map?t=10');

    });

    it('updates the conversation when UPDATE_CONVERSATION is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.UPDATE_CONVERSATION,
            data: (
                <div className="userRequest">
                    Hello
                </div>
            )
        });

        expect(state.conversation).toHaveLength(1);

    });

    it("sets the showError flag when SHOW_ERROR_MODAL is triggered", () => {
        let state = reducer(undefined, {
            type: actionTypes.SHOW_ERROR_MODAL
        });

        expect(state.showError).toBe(true);
    });

    it('sets the isChatLocked flag to true when LOCK_CHAT is triggered', () => {
        let state = reducer(undefined, {
            type: actionTypes.LOCK_CHAT
        });

        expect(state.isChatLocked).toBe(true);

    });

    it('unlocked the chat when UNLOCK_CHAT is triggered', () => {

        let state = reducer(undefined, {
            type: null
        });

        state.isChatLocked = true;

        state = reducer(state, {
            type: actionTypes.UNLOCK_CHAT
        });

        expect(state.isChatLocked).toBe(false);

    });

    it('sets the login token when SET_LOGIN_TOKEN is triggered', () => {
         let state = reducer(undefined, {
             type: null
         });

         state = reducer(state, {
             type: actionTypes.SET_LOGIN_TOKEN,
             data: "ABCD"
         });

         expect(state.token).toEqual("ABCD");
    });

    it('clears the login token when CLEAR_LOGIN_TOKEN is tirggered', () => {
        let state = reducer(undefined, {
            type: null
        });

        state.token = "ABCD";

        state = reducer(state, {
            type: actionTypes.CLEAR_LOGIN_TOKEN
        });

        expect(state.token).toBe(null);
    });

});