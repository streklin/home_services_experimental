import React, {Component} from 'react';
import openSocket from 'socket.io-client';
import './App.css';

class App extends Component {

    state = {
        imgData: null
    };


    componentDidMount() {
        this.socket = openSocket('http://192.168.0.13:8000');

        this.socket.on('broadcastImage', (data) => {
            let newState = {
                imgData: data
            };

            this.setState(newState);

        });
    }

    render() {
        return (
            <div className="App">
                <img src={this.state.imgData} alt="web cam"/>
            </div>
        );
    }
}

export default App;
