import React from 'react';
import * as actionCreators from '../../store/actionCreators';
import './RemoteControl.css';

const remoteControl = (props) => {
    return (
        <div className="remote-controls">

            <div className="controls-container">
                <div onClick={() => actionCreators.callForward(props.token)}  className="remote-up remote-button">
                    <i className="fas fa-caret-up"></i>
                </div>

                <div className="remote-middle remote-button">
                    <div onClick={() => actionCreators.callLeft(props.token)} className="remote-left remote-button">
                        <i className="fas fa-caret-left"></i>
                    </div>

                    <div onClick={() => actionCreators.callStop(props.token)} className="remote-stop remote-button">
                        <i className="fas fa-circle"></i>
                    </div>

                    <div onClick={() => actionCreators.callRight(props.token)} className="remote-right remote-button">
                        <i className="fas fa-caret-right"></i>
                    </div>
                </div>

                <div onClick={() => actionCreators.callBackward(props.token)} className="remote-down remote-button">
                    <i className="fas fa-caret-down"></i>
                </div>

            </div>

        </div>
    );
};

export default remoteControl;