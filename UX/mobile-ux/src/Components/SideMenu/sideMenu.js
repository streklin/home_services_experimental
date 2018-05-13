import React from 'react';
import './sideMenu.css';
import { NavLink } from 'react-router-dom';
import Footer from '../../Components/Footer/Footer';

const sideMenu = (props) => {

    return (
        <div className="sideMenu">
            <div className="menuOptionList">
                <div className="menuOption">
                    <NavLink to="/" exact={true} activeClassName="active">
                        <span>Control Robot</span>
                    </NavLink>
                </div>
                <div className="menuOption">
                    <NavLink to="/chat" exact={true} activeClassName="active">
                        <span>Chat</span>
                    </NavLink>
                </div>
                <div className="menuOption">
                    <span>Telepresence</span>
                </div>
                <div className="menuOption">
                    <NavLink to="/status" exact={true} activeClassName="active">
                        <span>Status</span>
                    </NavLink>
                </div>
            </div>

            <Footer/>
        </div>
    );

};

export default sideMenu;