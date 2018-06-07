import React, {Component} from 'react';
import { connect } from 'react-redux';
import { Redirect } from 'react-router-dom';
import * as actionCreators from '../../Services/store/actionCreators';
import './Login.css';
import foxwellRoboticsLogo from '../../assets/foxwellRoboticsLogo.png';

class Login extends Component {

    state = {
        username: "",
        password: ""
    };

    login = () => {
        this.props.doLogin(this.state.username, this.state.password);
    };

    onChange = (event, field) => {
        let newState = Object.assign({}, this.state);
        newState[field] = event.target.value;
        this.setState(newState);
    };

    render() {

        if (this.props.token !== null) {
            return <Redirect to='/'/>
        }

        return (
            <div className="Login">
                <img src={foxwellRoboticsLogo} alt="Foxwell Robotics" />
                <div className="loginContainer">
                    <div className="loginInput">
                        <label>Username:</label>
                        <input value={this.state.username} onChange={(event) => this.onChange(event, "username")} type="text" placeholder="Username" />
                    </div>
                    <div className="loginInput">
                        <label>Password:</label>
                        <input value={this.state.password} onChange={(event) => this.onChange(event, "password")} type="password" placeholder="password" />
                    </div>
                    <button onClick={this.login}>Login</button>
                </div>
            </div>
        );
    }
}

const mapStateToProps = (state) => {
    return {
        token: state.appStore.token
    }
};

const mapDispatchToProps = (dispatch) => {
    return {
        doLogin: (username, password) => dispatch(actionCreators.login(username, password))
    }
};

export default connect(mapStateToProps, mapDispatchToProps)(Login);