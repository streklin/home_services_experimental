import React, { Component } from 'react';
import * as actionCreators from '../../store/actionCreators';
import { connect } from 'react-redux';

export class ChatBot extends Component {

    state = {
        chatText: ""
    };

    onChange = (event) => {
        if (this.props.isChatLocked) return;

        let newState = Object.assign({}, this.state);
        newState.chatText = event.target.value;
        this.setState(newState);
    };

    onKeyPressed = (event) => {

        if (event.keyCode !== 13) {
            return;
        }

        if (this.props.isChatLocked) return;

        this.props.sendChatRequest(this.state.chatText);
    };

    render() {
        return (
            <div className="ChatBot">
                <div className="conversation">
                    {this.props.conversation}
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
            </div>

        )
    };
}

const mapStateToProps = (state) => {
    return {
        conversation: state.appStore.conversation,
        isChatLocked: state.appStore.isChatLocked
    }
};

const mapDispatchToProps = (dispatch) => {
    return {
        sendChatRequest: (query) => dispatch(actionCreators.sendChatRequest(query))
    }
};

export default connect(mapStateToProps, mapDispatchToProps)(ChatBot);