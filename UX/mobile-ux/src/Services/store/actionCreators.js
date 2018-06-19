import axios from 'axios';
import React from 'react';
import AWS from 'aws-sdk';
import * as actionTypes from './actions';
import {SERVER_URL} from "../config";


export function enableAutoMap() {
    return (dispatch, getState) => {
        let state = getState();
        let token = state.appStore.token;

        axios.get(SERVER_URL + 'robot/automap/activate', {
            headers: {
                'Authorization': 'Bearer ' + token
            }
        })
            .then(() => {
                dispatch({
                    type: actionTypes.ENABLE_AUTOMAP
                });
            })
            .catch( (err) => {
                dispatch({
                    type: actionTypes.SHOW_ERROR_MODAL
                });
            });
    };
}

export function disableAutoMap() {
    return (dispatch, getState) => {
        let state = getState();
        let token = state.appStore.token;

        axios.get(SERVER_URL + 'robot/automap/deactivate', {
            headers: {
                'Authorization': 'Bearer ' + token
            }
        })
            .then(() => {
                dispatch({
                    type: actionTypes.DISABLE_AUTOMAP
                });
            })
            .catch( (err) => {
                dispatch({
                    type: actionTypes.SHOW_ERROR_MODAL
                });
            });
    }
}



function sendChatRequestToRobot(dispatch, data, token) {
    if (data.dialogState !== 'Fulfilled') return;


    let requestBody = {
        intent: data.intentName,
        variables: data.slots
    };

    axios.post(SERVER_URL + "robot/chat/lex", requestBody, {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    })
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

        let state = getState();
        let token = state.appStore.token;

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

                sendChatRequestToRobot(dispatch, data, token);

            }
        });
    };
}

// these functions do not update state
export function callForward(token) {
    axios.get(SERVER_URL + "robot/drive/forward", {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    });
}

export function callBackward(token) {
    axios.get(SERVER_URL + "robot/drive/backward", {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    });
}

export function callLeft(token) {
    axios.get(SERVER_URL + "robot/drive/left", {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    });
}

export function callRight(token) {
    axios.get(SERVER_URL + "robot/drive/right", {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    });
}

export function callStop(token) {
    axios.get(SERVER_URL + "robot/drive/stop", {
        headers: {
            'Authorization': 'Bearer ' + token
        }
    });
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