import React, {Component} from 'react';
import { Redirect } from 'react-router-dom';
import {connect} from 'react-redux';

const loginRedirect = (WrappedComponent) => {
    const mapStateToProps = (state) => {
        return {
            token: state.appStore.token
        };
    };

    class LoginRedirectCtrl extends Component {

        render() {
            if (this.props.token === null) {
                return <Redirect to='/login'/>
            } else {
                return <WrappedComponent {...this.props} />
            }
        }

    }

    return connect(mapStateToProps)(LoginRedirectCtrl);


};

export default loginRedirect;