import React, {Component} from 'react';
import axios from 'axios';
import {SERVER_URL} from '../../settings';
import AWS from 'aws-sdk';
import Footer from '../Footer/Footer';
import './ChatBot.css';

class ChatBot extends Component {

    state = {
        conversation: [],
        chatText: "",
        isLocked: false
    };

    lockTextBox = () => {
        let newState = Object.assign({}, this.state);
        newState.isLocked = true;
        this.setState(newState);
    };

    unLockTextBox = () => {
        let newState = Object.assign({}, this.state);
        newState.isLocked = false;
        newState.chatText = "";
        this.setState(newState);
    };

    appendRequest = (request) => {
        let newState = Object.assign({}, this.state);
        let newRequest = (
            <div className="userRequest">
                {request}
            </div>
        );

        newState.conversation.push(newRequest);
        this.setState(newState);
    };

    appendResponse = (response) => {
        let newState = Object.assign({}, this.state);
        let newRequest = (
            <div className="botResponse">
                {response}
            </div>
        );

        newState.conversation.push(newRequest);
        this.setState(newState);
    };

    sendRequestToMarvin = (req) => {
        if (req.dialogState !== "Fulfilled") {
            return;
        }

        let requestBody = {
            intent: req.intentName,
            variables: req.slots
        };

        axios.post(SERVER_URL + "robot/chat/lex", requestBody);
    };

    submitChatRequest = (query) => {

        this.lockTextBox();

        this.appendRequest(query);

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
                console.log(err, err.stack);
            }

            if (data) {
                sessionAttributes = data.sessionAttributes;
                this.sendRequestToMarvin(data);
                this.appendResponse(data.message);
                this.unLockTextBox();
            }
        });
    };

    onChange = (event) => {
        let newState = Object.assign({}, this.state);
        newState.chatText = event.target.value;
        this.setState(newState);
    };

    onKeyPressed = (event) => {

        if (event.keyCode !== 13) {
            return;
        }

        if (this.state.isLocked) return;
        this.submitChatRequest(this.state.chatText);
    };

    render() {

        return (
            <div className="ChatBot">
                <div className="conversation">
                    {this.state.conversation}
                </div>
                <div className="chatInput">
                    <input
                        type="text"
                        value={this.state.chatText}
                        placeholder="talk to marvin"
                        onKeyDown={this.onKeyPressed}
                        onChange={this.onChange}
                    />
                </div>
                <Footer />
            </div>
        );
    }
}

export default ChatBot;