import axios from 'axios';
import React from 'react';
import AWS from 'aws-sdk';
import * as actionTypes from './actions';

const SERVER_URL = 'http://192.168.0.13:8080/';

export function updateCameraImg() {

    return (dispatch, getState) => {

        let newCamUrl = SERVER_URL + "robot/images/camera?t=" + new Date().getTime();
        dispatch({
            type: actionTypes.UPDATE_CAMERA_URL,
            data: newCamUrl
        });

    };

}

export function updateMapImg() {

    return (dispatch, getState) => {
        let newMapUrl = SERVER_URL + "robot/images/map?t=" + new Date().getTime();
        dispatch({
            type: actionTypes.UPDATE_MAP_URL,
            data: newMapUrl
        });
    };

}

export function toggleAutoMap() {

    return (dispatch, getState) => {

        let state = getState();
        let token = state.appStore.token;

        axios
            .get(SERVER_URL + "robot/state/toggleAutoMap", {
                headers: {
                    'Authorization': 'Bearer ' + token
                }
            })
            .then(() => {
                dispatch({
                   type: actionTypes.TOGGLE_AUTO_MAP
                });
            });
    };

}

function sendChatRequestToRobot(dispatch, data) {
    if (data.dialogState !== 'Fulfilled') return;

    let requestBody = {
        intent: data.intentName,
        variables: data.slots
    };

    axios.post(SERVER_URL + "robot/chat/lex", requestBody)
        .then((response) => {

            dispatch({
                type: actionTypes.UNLOCK_CHAT
            });

            if (response.data === "OK") return;

            // need to append marvins response to the chat
            let newResponse = (
                <div className="botResponse">
                    {response.data}
                </div>
            );

            dispatch({
                type: actionTypes.UPDATE_CONVERSATION,
                data: newResponse
            });

        });
}

export function sendChatRequest(query) {
    return (dispatch, getState) => {

        dispatch({
            type: actionTypes.LOCK_CHAT
        });

        let newRequest = (
            <div className="userRequest">
                {query}
            </div>
        );

        let lexruntime = new AWS.LexRuntime();
        let lexUserId = 'chatbot-demo' + Date.now();
        let sessionAttributes = {};

        let params =  {
            botAlias: '$LATEST',
            botName: 'FoxwellRobotPrototype',
            inputText: query,
            userId: lexUserId,
            sessionAttributes: sessionAttributes
        };

        lexruntime.postText(params, (err, data) => {
            if (err) {
                dispatch({
                    type: actionTypes.SHOW_ERROR_MODAL
                });

                return;
            }

            if (data) {

                let newResponse = (
                    <div className="botResponse">
                        {data.message}
                    </div>
                );

                dispatch({
                    type: actionTypes.UPDATE_CONVERSATION,
                    data: newRequest
                });

                dispatch({
                    type: actionTypes.UPDATE_CONVERSATION,
                    data: newResponse
                });

                sendChatRequestToRobot(dispatch, data);

            }
        });
    };
}

// these functions do not update state
export function callForward() {
    axios.get(SERVER_URL + "robot/drive/forward");
}

export function callBackward() {
    axios.get(SERVER_URL + "robot/drive/backward");
}

export function callLeft() {
    axios.get(SERVER_URL + "robot/drive/left");
}

export function callRight() {
    axios.get(SERVER_URL + "robot/drive/right");
}

export function callStop() {
    axios.get(SERVER_URL + "robot/drive/stop");
}

export function login(username, password) {

    return (dispatch) => {
        axios.post(SERVER_URL + "robot/login", {
            username: username,
            password: password
        })
            .then( (response) => {
                console.log(response);

                if (response.data.status === "OK") {
                    dispatch({
                        type: actionTypes.SET_LOGIN_TOKEN,
                        data: response.data.token
                    });
                } else {
                    dispatch({
                        type: actionTypes.SHOW_ERROR_MODAL
                    });
                }

            } );
    };
}