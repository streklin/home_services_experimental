import React, {Component} from 'react';
import { connect } from 'react-redux';
import * as actionCreators from '../../store/actionCreators';
import './Login.css';

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
        return (
            <div className="Login">
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
        );
    }
}

const mapDispatchToProps = (dispatch) => {
    return {
        doLogin: (username, password) => dispatch(actionCreators.login(username, password))
    }
};

export default connect(null, mapDispatchToProps)(Login);