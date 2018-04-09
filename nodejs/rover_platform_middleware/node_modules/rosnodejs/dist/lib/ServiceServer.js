/*
 *    Copyright 2016 Rethink Robotics
 *
 *    Copyright 2016 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

"use strict";

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

var net = require('net');
var NetworkUtils = require('../utils/network_utils.js');
var ros_msg_utils = require('../ros_msg_utils');
var base_serializers = ros_msg_utils.Serialize;
var SerializationUtils = require('../utils/serialization_utils.js');
var DeserializeStream = SerializationUtils.DeserializeStream;
var Deserialize = SerializationUtils.Deserialize;
var Serialize = SerializationUtils.Serialize;
var TcprosUtils = require('../utils/tcpros_utils.js');
var EventEmitter = require('events');
var Logging = require('./Logging.js');

var _require = require('../utils/ClientStates.js'),
    REGISTERING = _require.REGISTERING,
    REGISTERED = _require.REGISTERED,
    SHUTDOWN = _require.SHUTDOWN;

var ServiceServer = function (_EventEmitter) {
  _inherits(ServiceServer, _EventEmitter);

  function ServiceServer(options, callback, nodeHandle) {
    _classCallCheck(this, ServiceServer);

    var _this = _possibleConstructorReturn(this, (ServiceServer.__proto__ || Object.getPrototypeOf(ServiceServer)).call(this));

    _this._service = options.service;

    _this._type = options.type;

    _this._port = null;

    _this._nodeHandle = nodeHandle;

    _this._log = Logging.getLogger('ros.rosnodejs');

    _this._requestCallback = callback;

    _this._messageHandler = options.typeClass;

    _this._clients = {};

    _this._state = REGISTERING;

    _this._register();
    return _this;
  }

  _createClass(ServiceServer, [{
    key: 'getService',
    value: function getService() {
      return this._service;
    }
  }, {
    key: 'getType',
    value: function getType() {
      return this._type;
    }
  }, {
    key: 'getPersist',
    value: function getPersist() {
      return this._persist;
    }
  }, {
    key: 'isCallInProgress',
    value: function isCallInProgress() {
      return this._calling;
    }
  }, {
    key: 'getServiceUri',
    value: function getServiceUri() {
      return NetworkUtils.formatServiceUri(this._port);
    }
  }, {
    key: 'getClientUris',
    value: function getClientUris() {
      return Object.keys(this._clients);
    }

    /**
     * The ROS client shutdown code is a little noodly. Users can close a client through
     * the ROS node or the client itself and both are correct. Either through a node.unadvertise()
     * call or a client.shutdown() call - in both instances a call needs to be made to the ROS master
     * and the client needs to tear itself down.
     */

  }, {
    key: 'shutdown',
    value: function shutdown() {
      this._nodeHandle.unadvertiseService(this.getService());
    }
  }, {
    key: 'isShutdown',
    value: function isShutdown() {
      return this._state === SHUTDOWN;
    }
  }, {
    key: 'disconnect',
    value: function disconnect() {
      var _this2 = this;

      this._state = SHUTDOWN;

      Object.keys(this._clients).forEach(function (clientId) {
        var client = _this2._clients[clientId];

        client.$deserializeStream.removeAllListeners();

        client.end();
        client.destroy();
      });

      this._clients = {};
    }
  }, {
    key: 'handleClientConnection',
    value: function handleClientConnection(client, header) {
      var _this3 = this;

      if (this.isShutdown()) {
        return;
      }
      // else
      // TODO: verify header data
      this._log.debug('Service %s handling new client connection ', this.getService());

      var error = TcprosUtils.validateServiceClientHeader(header, this.getService(), this._messageHandler.md5sum());
      if (error) {
        this._log.error('Error while validating service %s connection header: %s', this.getService(), error);
        client.end(Serialize(TcprosUtils.createTcpRosError(error)));
        return;
      }

      var respHeader = TcprosUtils.createServiceServerHeader(this._nodeHandle.getNodeName(), this._messageHandler.md5sum(), this.getType());
      client.write(respHeader);

      client.$persist = header['persistent'] === '1';

      // bind to message handler
      client.$messageHandler = this._handleMessage.bind(this, client);
      client.$deserializeStream.on('message', client.$messageHandler);

      client.on('close', function () {
        delete _this3._clients[client.name];
        _this3._log.debug('Service client %s disconnected!', client.name);
      });

      this._clients[client.name] = client;
      this.emit('connection', header, client.name);
    }
  }, {
    key: '_handleMessage',
    value: function _handleMessage(client, data) {
      this._log.trace('Service  ' + this.getService() + ' got message! ' + data.toString('hex'));
      // deserialize msg
      var req = this._messageHandler.Request.deserialize(data);

      // call service callback
      var resp = new this._messageHandler.Response();
      var success = this._requestCallback(req, resp);

      var serializeResponse = TcprosUtils.serializeServiceResponse(this._messageHandler.Response, resp, success);

      // send service response
      client.write(serializeResponse);

      if (!client.$persist) {
        this._log.debug('Closing non-persistent client');
        client.end();
        delete this._clients[client.name];
      }
    }
  }, {
    key: '_register',
    value: function _register() {
      var _this4 = this;

      this._nodeHandle.registerService(this.getService()).then(function (resp) {
        // if we were shutdown between the starting the registration and now, bail
        if (_this4.isShutdown()) {
          return;
        }

        _this4._state = REGISTERED;
        _this4.emit('registered');
      });
    }
  }]);

  return ServiceServer;
}(EventEmitter);

module.exports = ServiceServer;