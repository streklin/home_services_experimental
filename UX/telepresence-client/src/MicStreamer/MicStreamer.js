import React from 'react';

const micStreamer = (props) => {

    let constraints = {
        audio: true,
        video: false
    };

    navigator
        .mediaDevices
        .getUserMedia(constraints)
        .then((stream) => {
            let audioContext = window.AudioContext;
            let context = new audioContext();

            let audioInput = context.createMediaStreamSource(stream);
            let bufferSize = 2048;

            let recorder = context.createScriptProcessor(bufferSize, 1, 1);

            recorder.onaudioprocess = (event) => {
                let left = event.inputBuffer.getChannelData(0);
                props.socket.emit(props.broadcastTopic, left);
            };

            audioInput.connect(recorder);

            recorder.connect(context.destination);
        })
        .catch((err) => {
            console.log("ERROR: ", err);
        });


    return (
        <div></div>
    );
};

export default micStreamer;