#!/usr/bin/env node
'use strict';

const server = require('http').createServer();
const io = require('socket.io')(server);
const ab2str = require('arraybuffer-to-string');

const port = 8000;

exports.dataStreamer = function() {

    let instance = null;

    let dataStreamer = function(rosNode) {

        // start the socket.io server
        io.listen(port);
        console.log('streaming on port ', port);

        // get the video and audio stream from the client and pass it on
        io.on('connection', (client) => {

            client.on('webcamImage', (data) => {
                io.sockets.emit('telepresenceVideoFrame', data);
            });

            client.on('pcmAudioChunk', (data) => {
                // play the audio chunk
                io.sockets.emit('pcmBroadcast', data);

            });
        });

        // subscribe to the image topics
        rosNode.subscribe("/camera/image_raw/compressed", "sensor_msgs/CompressedImage", (msg) => {
            let base64Encoded = ab2str(msg.data, 'base64');
            io.sockets.emit("robot-camera", base64Encoded);
        });

        rosNode.subscribe("/map/image", "sensor_msgs/Image", (msg) => {
            let base64Encoded = ab2str(msg.data, 'base64');
            io.sockets.emit('robot-map', base64Encoded);
        });
    };

    dataStreamer.getInstance = function(rosNode) {
        if (instance === null) {
            instance = new dataStreamer(rosNode);
        }

        return instance;
    };


    return dataStreamer.getInstance;

};