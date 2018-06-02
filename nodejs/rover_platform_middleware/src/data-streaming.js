#!/usr/bin/env node
'use strict';

const server = require('http').createServer();
const io = require('socket.io')(server);
const ab2str = require('arraybuffer-to-string');
const jpeg = require('jpeg-js');

const port = 8000;

exports.dataStreamer = function() {

    let instance = null;

    let dataStreamer = function(rosNode) {

        // start the socket.io server
        io.listen(port);
        console.log('streaming on port ', port);

        // subscribe to the image topics
        rosNode.subscribe("/camera/image_raw/compressed", "sensor_msgs/CompressedImage", (msg) => {
            let base64Encoded = ab2str(msg.data, 'base64');
            io.sockets.emit("robot-camera", base64Encoded);
        });

        rosNode.subscribe("/map/image", "sensor_msgs/Image", (msg) => {
/*            let bufferedData = Buffer.from(msg.data);

            let rawImageData = {
                data: bufferedData,
                width: msg.width,
                height: msg.height
            };

            let jpegImageData = jpeg.encode(rawImageData);
            let base64Encoded = ab2str(jpegImageData.data, 'base64');*/
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