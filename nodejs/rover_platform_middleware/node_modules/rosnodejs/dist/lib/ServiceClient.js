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

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

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
    REGISTERED = _require.REGISTERED,
    SHUTDOWN = _require.SHUTDOWN;

/**
 * @class ServiceCall
 * A small utility class for ServiceClient...
 * basically just a struct.
 */


var ServiceCall = function ServiceCall(request, resolve, reject) {
  _classCallCheck(this, ServiceCall);

  this.request = request;
  this.resolve = resolve;
  this.reject = reject;

  this.serviceClient = null;
};

/**
 * @class ServiceClient
 * ServiceClient provides an interface to querying a service in ROS.
 * Typically ROS service calls are blocking. This isn't an option for JS though.
 * To accommodate multiple successive service calls, calls are queued along with
 * resolve/reject handlers created for that specific call. When a call completes, the
 * next call in the queue is handled
 */


var ServiceClient = function (_EventEmitter) {
  _inherits(ServiceClient, _EventEmitter);

  function ServiceClient(options, nodeHandle) {
    _classCallCheck(this, ServiceClient);

    var _this = _possibleConstructorReturn(this, (ServiceClient.__proto__ || Object.getPrototypeOf(ServiceClient)).call(this));

    _this._service = options.service;

    _this._type = options.type;

    _this._persist = !!options.persist;

    _this._maxQueueLength = options.queueLength || -1;

    _this._resolve = !!options.resolve;

    _this._calling = false;

    _this._log = Logging.getLogger('ros.rosnodejs');

    _this._nodeHandle = nodeHandle;

    _this._messageHandler = options.typeClass;

    _this._serviceClient = null;

    _this._callQueue = [];

    _this._currentCall = null;

    // ServiceClients aren't "registered" anywhere but it's not
    // waiting to get registered either so REGISTERING doesn't make sense...
    // Hence, we'll just call it REGISTERED.
    _this._state = REGISTERED;
    return _this;
  }

  _createClass(ServiceClient, [{
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
    key: 'close',
    value: function close() {
      // don't remove service client if call is in progress
      if (!this.isCallInProgress()) {
        this._serviceClient = null;
      }
    }
  }, {
    key: 'shutdown',
    value: function shutdown() {
      this._state = SHUTDOWN;
      if (this._currentCall) {
        this._currentCall.reject('SHUTDOWN');
      }
    }
  }, {
    key: 'isShutdown',
    value: function isShutdown() {
      return this._state === SHUTDOWN;
    }

    /**
     * Call the service - if a current call is in progress, nothing will be done
     * @return {Promise}
     */

  }, {
    key: 'call',
    value: function call(request) {
      var _this2 = this;

      return new Promise(function (resolve, reject) {
        var newCall = new ServiceCall(request, resolve, reject);
        _this2._callQueue.push(newCall);

        // shift off old calls if user specified a max queue length
        if (_this2._maxQueueLength > 0 && _this2._callQueue.length > _this2._maxQueueLength) {
          var oldCall = _this2._callQueue.shift();
          var err = new Error('Unable to complete service call because of queue limitations');
          err.code = 'E_ROSSERVICEQUEUEFULL';
          oldCall.reject(err);
        }

        // if there weren't any other calls in the queue and there's no current call, execute this new call
        // otherwise new call will be handled in order when others complete
        if (_this2._callQueue.length === 1 && _this2._currentCall === null) {
          _this2._executeCall();
        }
      });
    }
  }, {
    key: '_executeCall',
    value: function _executeCall() {
      var _this3 = this;

      if (this.isShutdown()) {
        return;
      } else if (this._callQueue.length === 0) {
        this._log.warn('Tried executing service call on empty queue');
        return;
      }
      // else
      var call = this._callQueue.shift();
      this._currentCall = call;
      this._calling = true;

      this._initiateServiceConnection(call).then(function () {
        _this3._throwIfShutdown();

        return _this3._sendRequest(call);
      }).then(function (msg) {
        _this3._throwIfShutdown();

        _this3._calling = false;
        _this3._currentCall = null;

        _this3._scheduleNextCall();

        call.resolve(msg);
      }).catch(function (err) {
        if (!_this3.isShutdown()) {
          // this probably just means the service didn't exist yet - don't complain about it
          // We should still reject the call
          if (err.code !== 'EROSAPIERROR') {
            _this3._log.error('Error during service ' + _this3.getService() + ' call ' + err);
          }

          _this3._calling = false;
          _this3._currentCall = null;

          _this3._scheduleNextCall();

          call.reject(err);
        }
      });
    }
  }, {
    key: '_scheduleNextCall',
    value: function _scheduleNextCall() {
      if (this._callQueue.length > 0 && !this.isShutdown()) {
        process.nextTick(this._executeCall.bind(this));
      }
    }
  }, {
    key: '_initiateServiceConnection',
    value: function _initiateServiceConnection(call) {
      var _this4 = this;

      // if we haven't connected to the service yet, create the connection
      // this will always be the case unless this is persistent service client
      // calling for a second time.
      if (!this.getPersist() || this._serviceClient === null) {
        return this._nodeHandle.lookupService(this.getService()).then(function (resp) {
          _this4._throwIfShutdown();

          var serviceUri = resp[2];
          var serviceHost = NetworkUtils.getAddressAndPortFromUri(serviceUri);

          // connect to the service's tcpros server
          return _this4._connectToService(serviceHost, call);
        });
      } else {
        // this is a persistent service that we've already set up
        call.serviceClient = this._serviceClient;
        return Promise.resolve();
      }
    }
  }, {
    key: '_sendRequest',
    value: function _sendRequest(call) {
      var _this5 = this;

      if (this._resolve) {
        call.request = this._messageHandler.Request.Resolve(call.request);
      }

      // serialize request
      var serializedRequest = TcprosUtils.serializeMessage(this._messageHandler.Request, call.request);

      call.serviceClient.write(serializedRequest);

      return new Promise(function (resolve, reject) {
        call.serviceClient.$deserializeStream.once('message', function (msg, success) {
          if (success) {
            resolve(_this5._messageHandler.Response.deserialize(msg));
          } else {
            var error = new Error(msg);
            error.code = 'E_ROSSERVICEFAILED';
            reject(error);
          }
        });
      });
    }
  }, {
    key: '_connectToService',
    value: function _connectToService(serviceHost, call) {
      var _this6 = this;

      return new Promise(function (resolve, reject) {
        _this6._log.debug('Service client %s connecting to %j', _this6.getService(), serviceHost);

        _this6._createCallSocketAndHandlers(serviceHost, call, reject);

        _this6._cacheSocketIfPersistent(call);

        var deserializer = new DeserializeStream();
        call.serviceClient.$deserializeStream = deserializer;
        call.serviceClient.pipe(deserializer);

        deserializer.once('message', function (msg) {
          if (!call.serviceClient.$initialized) {
            var header = TcprosUtils.parseTcpRosHeader(msg);
            if (header.error) {
              reject(new Error(header.error));
              return;
            }

            // stream deserialization for service response is different - set that up for next message
            deserializer.setServiceRespDeserialize();
            call.serviceClient.$initialized = true;
            resolve();
          }
        });
      });
    }
  }, {
    key: '_createCallSocketAndHandlers',
    value: function _createCallSocketAndHandlers(serviceHost, call, reject) {
      var _this7 = this;

      // create a socket connection to the service provider
      call.serviceClient = net.connect(serviceHost, function () {

        // Connection to service's TCPROS server succeeded - generate and send a connection header
        _this7._log.debug('Sending service client %s connection header', _this7.getService());

        var serviceClientHeader = TcprosUtils.createServiceClientHeader(_this7._nodeHandle.getNodeName(), _this7.getService(), _this7._messageHandler.md5sum(), _this7.getType(), _this7.getPersist());

        call.serviceClient.write(serviceClientHeader);
      });

      // bind a close handling function
      call.serviceClient.on('close', function () {
        call.serviceClient = null;
        // we could probably just always reset this._serviceClient to null here but...
        if (_this7.getPersist()) {
          _this7._serviceClient = null;
        }
      });

      // bind an error function - any errors connecting to the service
      // will cause the call to be rejected (in this._executeCall)
      call.serviceClient.on('error', function (err) {
        _this7._log.info('Service Client ' + _this7.getService() + ' error: ' + err);
        reject(err);
      });
    }
  }, {
    key: '_cacheSocketIfPersistent',
    value: function _cacheSocketIfPersistent(call) {
      // If this is a persistent service client, we're here because we haven't connected to this service before.
      // Cache the service client for later use. Future calls won't need to lookup the service with the ROS master
      // or deal with the connection header.
      if (this.getPersist()) {
        this._serviceClient = call.serviceClient;
      }
    }
  }, {
    key: '_throwIfShutdown',
    value: function _throwIfShutdown() {
      if (this.isShutdown()) {
        throw new Error('SHUTDOWN');
      }
    }
  }]);

  return ServiceClient;
}(EventEmitter);

module.exports = ServiceClient;