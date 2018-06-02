import React from 'react';
import './RemoteCamera.css';

const remoteCamera = (props) => {
    return (
        <div className="RemoteCamera">
            <img src={props.imageData} alt="Robot Camera" />
        </div>
    )
};

export default remoteCamera;