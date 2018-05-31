#!/usr/bin/env node
'use strict';

const jwt = require('jsonwebtoken');
const bcrypt = require('bcryptjs');

exports.authentication = function() {
    let instance = null;

    const secret_key = process.env.ROS_AUTH_SECRET;
    const ros_username = process.env.ROS_USERNAME;
    const ros_password = process.env.ROS_PASSWORD;

    function authentication() {}

    authentication.getInstance = function() {
        if (instance === null) instance = new authentication();
        return instance;
    };

    authentication.prototype.login = function(username, password) {
        if (username === null || username === undefined || username === "") throw "username was not provided";
        if (password === null || password === undefined || password === "") throw "password was not provided";

        if (ros_username !== username) return null;

        let token = null;

        if (bcrypt.compareSync(password, ros_password)) {
            token = jwt.sign({ id: ros_username}, secret_key, {
                expiresIn: 86400 // expires in 24 hours
            });
        }


        return token;
    };

    authentication.prototype.authorize = function(req, res, next) {

        if (req.url === "/robot/login") {
            next();
            return;
        }

        if (req.method === "OPTIONS") {
            next();
            return;
        }

        let token = req.headers['authorization'];
        if (!token) return res.status(401).send({ auth: false, message: 'No token provided.' });

        token = token.replace('Bearer ', '');

        jwt.verify(token, secret_key, function(err, decoded) {

            if (err) {
                console.log(err);
                return res.status(401).send({ auth: false, message: 'Failed to authenticate token.' });
            }

            next();
        });
    };

    return authentication.getInstance;
};