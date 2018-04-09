/*
 *    Copyright 2017 Rethink Robotics
 *
 *    Copyright 2017 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing,
 *    software distributed under the License is distributed on an "AS
 *    IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 *    express or implied. See the License for the specific language
 *    governing permissions and limitations under the License.
 */

"use strict";

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

var SerializationUtils = require('../../utils/serialization_utils.js');
var Serialize = SerializationUtils.Serialize;
var TcprosUtils = require('../../utils/tcpros_utils.js');
var EventEmitter = require('events');
var Logging = require('../Logging.js');

var _require = require('../../utils/ClientStates.js'),
    REGISTERING = _require.REGISTERING,
    REGISTERED = _require.REGISTERED,
    SHUTDOWN = _require.SHUTDOWN;

/**
 * Implementation class for a Publisher. Handles registration, connecting to
 * subscribers, etc. Public-facing publisher classes will be given an instance
 * of this to use
 */


var PublisherImpl = function (_EventEmitter) {
  _inherits(PublisherImpl, _EventEmitter);

  function PublisherImpl(options, nodeHandle) {
    _classCallCheck(this, PublisherImpl);

    var _this = _possibleConstructorReturn(this, (PublisherImpl.__proto__ || Object.getPrototypeOf(PublisherImpl)).call(this));

    _this.count = 0;

    _this._topic = options.topic;

    _this._type = options.type;

    _this._latching = !!options.latching;

    _this._tcpNoDelay = !!options.tcpNoDelay;

    if (options.queueSize) {
      _this._queueSize = options.queueSize;
    } else {
      _this._queueSize = 1;
    }

    /**
     * throttleMs interacts with queueSize to determine when to send
     * messages.
     *  < 0  : send immediately - no interaction with queue
     * >= 0 : place event at end of event queue to publish message
         after minimum delay (MS)
     */
    if (options.hasOwnProperty('throttleMs')) {
      _this._throttleMs = options.throttleMs;
    } else {
      _this._throttleMs = 0;
    }

    // OPTIONS STILL NOT HANDLED:
    //  headers: extra headers to include
    //  subscriber_listener: callback for new subscribers connect/disconnect

    _this._resolve = !!options.resolve;

    _this._lastSentMsg = null;

    _this._nodeHandle = nodeHandle;
    _this._nodeHandle.getSpinner().addClient(_this, _this._getSpinnerId(), _this._queueSize, _this._throttleMs);

    _this._log = Logging.getLogger('ros.rosnodejs');

    _this._subClients = {};

    _this._messageHandler = options.typeClass;

    _this._state = REGISTERING;
    _this._register();
    return _this;
  }

  /**
   * Uniquely identifies this publisher to the node's Spinner
   * @returns {string}
   */


  _createClass(PublisherImpl, [{
    key: '_getSpinnerId',
    value: function _getSpinnerId() {
      return 'Publisher://' + this.getTopic();
    }

    /**
     * Get the name of the topic this subscriber is listening on
     * @returns {string}
     */

  }, {
    key: 'getTopic',
    value: function getTopic() {
      return this._topic;
    }

    /**
     * Get the type of message this subscriber is handling
     *            (e.g. std_msgs/String)
     * @returns {string}
     */

  }, {
    key: 'getType',
    value: function getType() {
      return this._type;
    }

    /**
     * Check if this publisher is set to latch it's messages
     * @returns {boolean}
     */

  }, {
    key: 'getLatching',
    value: function getLatching() {
      return this._latching;
    }

    /**
     * Get the count of subscribers connected to this publisher
     * @returns {number}
     */

  }, {
    key: 'getNumSubscribers',
    value: function getNumSubscribers() {
      return Object.keys(this._subClients).length;
    }

    /**
     * Get the list of client addresses connect to this publisher.
     * Used for getBusInfo Slave API calls
     * @returns {Array}
     */

  }, {
    key: 'getClientUris',
    value: function getClientUris() {
      return Object.keys(this._subClients);
    }

    /**
     * Get the ros node this subscriber belongs to
     * @returns {RosNode}
     */

  }, {
    key: 'getNode',
    value: function getNode() {
      return this._nodeHandle;
    }

    /**
     * Clears and closes all client connections for this publisher.
     */

  }, {
    key: 'shutdown',
    value: function shutdown() {
      var _this2 = this;

      this._state = SHUTDOWN;
      this._log.debug('Shutting down publisher %s', this.getTopic());

      Object.keys(this._subClients).forEach(function (clientId) {
        var client = _this2._subClients[clientId];
        client.end();
      });

      // disconnect from the spinner in case we have any pending callbacks
      this._nodeHandle.getSpinner().disconnect(this._getSpinnerId());
      this._subClients = {};
    }

    /**
     * Check if this subscriber has been shutdown
     * @returns {boolean}
     */

  }, {
    key: 'isShutdown',
    value: function isShutdown() {
      return this._state === SHUTDOWN;
    }

    /**
     * Schedules the msg for publishing or publishes immediately if we're
     * supposed to (throttleTime < 0)
     * @param msg {object} object type matching this._type
     * @param [throttleMs] {number} optional override for publisher setting
     */

  }, {
    key: 'publish',
    value: function publish(msg, throttleMs) {
      if (this.isShutdown()) {
        return;
      }

      if (typeof throttleMs !== 'number') {
        throttleMs = this._throttleMs;
      }

      if (throttleMs < 0) {
        // short circuit JS event queue, publish "synchronously"
        this._handleMsgQueue([msg]);
      } else {
        this._nodeHandle.getSpinner().ping(this._getSpinnerId(), msg);
      }
    }

    /**
     * Pulls all msgs off queue, serializes, and publishes them to all clients.
     * @param msgQueue {Array} Array of messages. Type of each message matches this._type
     */

  }, {
    key: '_handleMsgQueue',
    value: function _handleMsgQueue(msgQueue) {
      var _this3 = this;

      // There's a small chance that we were shutdown while the spinner was locked
      // which could cause _handleMsgQueue to be called if this publisher was in there.
      if (this.isShutdown()) {
        return;
      }

      var numClients = this.getNumSubscribers();
      if (numClients === 0) {
        this._log.debugThrottle(2000, 'Publishing message on ' + this.getTopic() + ' with no subscribers');
      }

      try {
        msgQueue.forEach(function (msg) {
          if (_this3._resolve) {
            msg = _this3._messageHandler.Resolve(msg);
          }

          var serializedMsg = TcprosUtils.serializeMessage(_this3._messageHandler, msg);

          Object.keys(_this3._subClients).forEach(function (client) {
            _this3._subClients[client].write(serializedMsg);
          });

          // if this publisher is supposed to latch,
          // save the last message. Any subscribers that connects
          // before another call to publish() will receive this message
          if (_this3.getLatching()) {
            _this3._lastSentMsg = serializedMsg;
          }
        });
      } catch (err) {
        this._log.error('Error when publishing message on topic %s: %s', this.getTopic(), err.stack);
        this.emit('error', err);
      }
    }

    /**
     * Handles a new connection from a subscriber to this publisher's node.
     * Validates the connection header and sends a response header
     * @param subscriber {Socket} client that sent the header
     * @param header {Object} deserialized connection header.
     */

  }, {
    key: 'handleSubscriberConnection',
    value: function handleSubscriberConnection(subscriber, header) {
      var _this4 = this;

      var error = TcprosUtils.validateSubHeader(header, this.getTopic(), this.getType(), this._messageHandler.md5sum());

      if (error !== null) {
        this._log.error('Unable to validate subscriber connection header ' + JSON.stringify(header));
        subscriber.end(Serialize(error));
        return;
      }
      // else
      this._log.info('Pub %s got connection header %s', this.getTopic(), JSON.stringify(header));

      // create and send response
      var respHeader = TcprosUtils.createPubHeader(this._nodeHandle.getNodeName(), this._messageHandler.md5sum(), this.getType(), this.getLatching(), this._messageHandler.messageDefinition());
      subscriber.write(respHeader);

      // if this publisher had the tcpNoDelay option set
      // disable the nagle algorithm
      if (this._tcpNoDelay || header.tcp_nodelay === 1) {
        subscriber.setNoDelay(true);
      }

      subscriber.on('close', function () {
        _this4._log.info('Publisher client socket %s on topic %s disconnected', subscriber.name, _this4.getTopic());
        subscriber.removeAllListeners();
        delete _this4._subClients[subscriber.name];
        _this4.emit('disconnect');
      });

      subscriber.on('end', function () {
        _this4._log.info('Publisher client socket %s on topic %s ended the connection', subscriber.name, _this4.getTopic());
      });

      subscriber.on('error', function (err) {
        _this4._log.warn('Publisher client socket %s on topic %s had error: %s', subscriber.name, _this4.getTopic(), err);
      });

      // if we've cached a message from latching, send it now
      if (this._lastSentMsg !== null) {
        this._log.debug('Sending latched msg to new subscriber');
        subscriber.write(this._lastSentMsg);
      }

      // handshake was good - we'll start publishing to it
      this._subClients[subscriber.name] = subscriber;

      this.emit('connection', header, subscriber.name);
    }

    /**
     * Makes an XMLRPC call to registers this publisher with the ROS master
     */

  }, {
    key: '_register',
    value: function _register() {
      var _this5 = this;

      this._nodeHandle.registerPublisher(this._topic, this._type).then(function (resp) {
        // if we were shutdown between the starting the registration and now, bail
        if (_this5.isShutdown()) {
          return;
        }

        _this5._log.info('Registered %s as a publisher: %j', _this5._topic, resp);
        var code = resp[0];
        var msg = resp[1];
        var subs = resp[2];
        if (code === 1) {
          // registration worked
          _this5._state = REGISTERED;
          _this5.emit('registered');
        }
      }).catch(function (err) {
        _this5._log.error('Error while registering publisher %s: %s', _this5.getTopic(), err);
      });
    }
  }]);

  return PublisherImpl;
}(EventEmitter);

module.exports = PublisherImpl;