import React, {Component} from 'react';
import openSocket from 'socket.io-client';
import PCMPlayer from './pcmPlayer';
import './App.css';
import {BASE_SOCKET} from "../../mobile-ux/src/Services/config";


let player = null;

class App extends Component {

    state = {
        imgData: null,
        audioUrl: null,
        chunks: []
    };


    start = () => {

        player = new PCMPlayer({
            encoding: '32bitFloat',
            channels: 1,
            sampleRate: 44100,
            flushingTime: 5
        });


        this.socket = openSocket(BASE_SOCKET);

        this.socket.on('telepresenceVideoFrame', (data) => {
            let newState = {
                imgData: data,
                chunks: this.state.chunks
            };

            this.setState(newState);
        });

        this.socket.on('serverPCMBroadcast', (data) => {
            let dataArray = Object.keys(data).map(i => data[i]);
            let floatData = Float32Array.from(dataArray);
            player.feed(floatData);

        });

    };


    render() {
        return (
            <div className="App">
                <img src={this.state.imgData} alt="web cam"/>
                <button onClick={this.start}>Start</button>
            </div>
        );
    }
}

export default App;
