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

var NetworkUtils = require('../../utils/network_utils.js');
var SerializationUtils = require('../../utils/serialization_utils.js');
var DeserializeStream = SerializationUtils.DeserializeStream;
var Deserialize = SerializationUtils.Deserialize;
var Serialize = SerializationUtils.Serialize;
var TcprosUtils = require('../../utils/tcpros_utils.js');
var Socket = require('net').Socket;
var EventEmitter = require('events');
var Logging = require('../Logging.js');

var _require = require('../../utils/ClientStates.js'),
    REGISTERING = _require.REGISTERING,
    REGISTERED = _require.REGISTERED,
    SHUTDOWN = _require.SHUTDOWN;

var protocols = [['TCPROS']];

//-----------------------------------------------------------------------

/**
 * Implementation class for a Subscriber. Handles registration, connecting to
 * publishers, etc. Public-facing subscriber classes will be given an instance
 * of this to use
 */

var SubscriberImpl = function (_EventEmitter) {
  _inherits(SubscriberImpl, _EventEmitter);

  function SubscriberImpl(options, nodeHandle) {
    _classCallCheck(this, SubscriberImpl);

    var _this = _possibleConstructorReturn(this, (SubscriberImpl.__proto__ || Object.getPrototypeOf(SubscriberImpl)).call(this));

    _this.count = 0;

    _this._topic = options.topic;

    _this._type = options.type;

    if (options.queueSize) {
      _this._queueSize = options.queueSize;
    } else {
      _this._queueSize = 1;
    }

    /**
     * throttleMs interacts with queueSize to determine when to handle callbacks
     *  < 0  : handle immediately - no interaction with queue
     *  >= 0 : place event at end of event queue to handle after minimum delay (MS)
     */
    if (options.hasOwnProperty('throttleMs')) {
      _this._throttleMs = options.throttleMs;
    } else {
      _this._throttleMs = 0;
    }

    _this._msgHandleTime = null;

    _this._nodeHandle = nodeHandle;
    _this._nodeHandle.getSpinner().addClient(_this, _this._getSpinnerId(), _this._queueSize, _this._throttleMs);

    _this._log = Logging.getLogger('ros.rosnodejs');

    _this._messageHandler = options.typeClass;

    _this._pubClients = {};

    _this._pendingPubClients = {};

    _this._state = REGISTERING;
    _this._register();
    return _this;
  }

  /**
   * Uniquely identifies this subscriber to the node's Spinner
   * @returns {string}
   */


  _createClass(SubscriberImpl, [{
    key: '_getSpinnerId',
    value: function _getSpinnerId() {
      return 'Subscriber://' + this.getTopic();
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
     * Get count of the publishers currently connected to this subscriber
     * @returns {number}
     */

  }, {
    key: 'getNumPublishers',
    value: function getNumPublishers() {
      return Object.keys(this._pubClients).length;
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
     * Clears and closes all client connections for this subscriber.
     */

  }, {
    key: 'shutdown',
    value: function shutdown() {
      this._state = SHUTDOWN;
      this._log.debug('Shutting down subscriber %s', this.getTopic());

      Object.keys(this._pubClients).forEach(this._disconnectClient.bind(this));
      Object.keys(this._pendingPubClients).forEach(this._disconnectClient.bind(this));

      // disconnect from the spinner in case we have any pending callbacks
      this._nodeHandle.getSpinner().disconnect(this._getSpinnerId());
      this._pubClients = {};
      this._pendingPubClients = {};
    }

    /**
     * @returns {boolean} true if this subscriber has been shutdown
     */

  }, {
    key: 'isShutdown',
    value: function isShutdown() {
      return this._state === SHUTDOWN;
    }

    /**
     * @returns {Array} URIs of all current clients
     */

  }, {
    key: 'getClientUris',
    value: function getClientUris() {
      return Object.keys(this._pubClients);
    }

    /**
     * Send a topic request to each of the publishers in the list.
     * Assumes we have NOT connected to them.
     * @param pubs {Array} array of uris of nodes that are publishing this topic
     */

  }, {
    key: 'requestTopicFromPubs',
    value: function requestTopicFromPubs(pubs) {
      var _this2 = this;

      pubs.forEach(function (pubUri) {
        pubUri = pubUri.trim();
        _this2._requestTopicFromPublisher(pubUri);
      });
    }

    /**
     * Handle an update from the ROS master with the list of current publishers. Connect to any new ones
     * and disconnect from any not included in the list.
     * @param publisherList {Array.string}
     * @private
     */

  }, {
    key: '_handlePublisherUpdate',
    value: function _handlePublisherUpdate(publisherList) {
      var _this3 = this;

      var missingPublishers = new Set(Object.keys(this._pubClients));

      publisherList.forEach(function (pubUri) {
        pubUri = pubUri.trim();
        if (!_this3._pubClients.hasOwnProperty(pubUri)) {
          _this3._requestTopicFromPublisher(pubUri);
        }

        missingPublishers.delete(pubUri);
      });

      missingPublishers.forEach(function (pubUri) {
        _this3._disconnectClient(pubUri);
      });
    }

    /**
     * Sends a topicRequest XMLRPC message to the provided URI and initiates
     *  the topic connection if possible.
     * @param pubUri {string} URI of publisher to request a topic from
     */

  }, {
    key: '_requestTopicFromPublisher',
    value: function _requestTopicFromPublisher(pubUri) {
      var _this4 = this;

      var info = NetworkUtils.getAddressAndPortFromUri(pubUri);
      // send a topic request to the publisher's node
      this._log.debug('Sending topic request to ' + JSON.stringify(info));
      this._nodeHandle.requestTopic(info.host, info.port, this._topic, protocols).then(function (resp) {
        _this4._handleTopicRequestResponse(resp, pubUri);
      }).catch(function (err, resp) {
        // there was an error in the topic request
        _this4._log.warn('Error requesting topic on %s: %s, %s', _this4.getTopic(), err, resp);
      });
    }

    /**
     * disconnects and clears out the specified client
     * @param clientId {string}
     */

  }, {
    key: '_disconnectClient',
    value: function _disconnectClient(clientId) {
      var client = this._pubClients[clientId];

      var hasValidatedClient = !!client;
      if (!hasValidatedClient) {
        client = this._pendingPubClients[clientId];
      }

      if (client) {
        this._log.debug('Subscriber %s disconnecting client %s', this.getTopic(), clientId);
        client.end();

        client.removeAllListeners();
        client.$deserializer.removeAllListeners();

        client.$deserializer.end();
        client.unpipe(client.$deserializer);

        delete client.$deserializer;

        delete this._pubClients[clientId];
        delete this._pendingPubClients[clientId];

        if (hasValidatedClient) {
          this.emit('disconnect');
        }
      }
    }

    /**
     * Registers the subscriber with the ROS master
     * will connect to any existing publishers on the topic that are included in the response
     */

  }, {
    key: '_register',
    value: function _register() {
      var _this5 = this;

      this._nodeHandle.registerSubscriber(this._topic, this._type).then(function (resp) {
        // if we were shutdown between the starting the registration and now, bail
        if (_this5.isShutdown()) {
          return;
        }

        // else handle response from register subscriber call
        var code = resp[0];
        var msg = resp[1];
        var pubs = resp[2];
        if (code === 1) {
          // success! update state to reflect that we're registered
          _this5._state = REGISTERED;

          if (pubs.length > 0) {
            // this means we're ok and that publishers already exist on this topic
            // we should connect to them
            _this5.requestTopicFromPubs(pubs);
          }
          _this5.emit('registered');
        }
      }).catch(function (err, resp) {
        _this5._log.warn('Error during subscriber %s registration: %s', _this5.getTopic(), err);
      });
    }

    /**
     * Handles the response to a topicRequest message (to connect to a publisher)
     * @param resp {Array} xmlrpc response to a topic request
     */

  }, {
    key: '_handleTopicRequestResponse',
    value: function _handleTopicRequestResponse(resp, nodeUri) {
      var _this6 = this;

      if (this.isShutdown()) {
        return;
      }

      this._log.debug('Topic request response: ' + JSON.stringify(resp));

      // resp[2] has port and address for where to connect
      var info = resp[2];
      var port = info[2];
      var address = info[1];

      var client = new Socket();
      client.name = address + ':' + port;
      client.nodeUri = nodeUri;

      client.on('end', function () {
        _this6._log.info('Subscriber client socket %s on topic %s ended the connection', client.name, _this6.getTopic());
      });

      client.on('error', function (err) {
        _this6._log.warn('Subscriber client socket %s on topic %s had error: %s', client.name, _this6.getTopic(), err);
      });

      // hook into close event to clean things up
      client.on('close', function () {
        _this6._log.info('Subscriber client socket %s on topic %s disconnected', client.name, _this6.getTopic());
        _this6._disconnectClient(client.nodeUri);
      });

      // open the socket at the provided address, port
      client.connect(port, address, function () {
        if (_this6.isShutdown()) {
          client.end();
          return;
        }

        _this6._log.debug('Subscriber on ' + _this6.getTopic() + ' connected to publisher at ' + address + ':' + port);
        client.write(_this6._createTcprosHandshake());
      });

      // create a DeserializeStream to chunk out messages
      var deserializer = new DeserializeStream();
      client.$deserializer = deserializer;
      client.pipe(deserializer);

      // cache client in "pending" map.
      // It's not validated yet so we don't want it to show up as a client.
      // Need to keep track of it in case we're shutdown before it can be validated.
      this._pendingPubClients[client.nodeUri] = client;

      // create a one-time handler for the connection header
      // if the connection is validated, we'll listen for more events
      deserializer.once('message', this._handleConnectionHeader.bind(this, client));
    }

    /**
     * Convenience function - creates the connection header for this subscriber to send
     * @returns {string}
     */

  }, {
    key: '_createTcprosHandshake',
    value: function _createTcprosHandshake() {
      return TcprosUtils.createSubHeader(this._nodeHandle.getNodeName(), this._messageHandler.md5sum(), this.getTopic(), this.getType(), this._messageHandler.messageDefinition());
    }

    /**
     * Handles the connection header from a publisher. If connection is validated,
     * we'll start handling messages from the client.
     * @param client {Socket} publisher client who sent the connection header
     * @param msg {string} message received from the publisher
     */

  }, {
    key: '_handleConnectionHeader',
    value: function _handleConnectionHeader(client, msg) {
      if (this.isShutdown()) {
        this._disconnectClient(client.nodeUri);
        return;
      }

      var header = TcprosUtils.parseTcpRosHeader(msg);
      // check if the publisher had a problem with our connection header
      if (header.error) {
        this._log.error(header.error);
        return;
      }

      // now do our own validation of the publisher's header
      var error = TcprosUtils.validatePubHeader(header, this.getType(), this._messageHandler.md5sum());
      if (error) {
        this._log.error('Unable to validate subscriber ' + this.getTopic() + ' connection header ' + JSON.stringify(header));
        TcprosUtils.parsePubHeader(msg);
        client.end(Serialize(error));
        return;
      }
      // connection header was valid - we're good to go!
      this._log.debug('Subscriber ' + this.getTopic() + ' got connection header ' + JSON.stringify(header));

      // cache client now that we've verified the connection header
      this._pubClients[client.nodeUri] = client;
      // remove client from pending map now that it's validated
      delete this._pendingPubClients[client.nodeUri];

      // pipe all future messages to _handleMessage
      client.$deserializer.on('message', this._handleMessage.bind(this));

      this.emit('connection', header, client.name);
    }

    /**
     * Handles a single message from a publisher. Passes message off to
     * Spinner if we're queueing, otherwise handles it immediately.
     * @param msg {string}
     */

  }, {
    key: '_handleMessage',
    value: function _handleMessage(msg) {
      if (this._throttleMs < 0) {
        this._handleMsgQueue([msg]);
      } else {
        this._nodeHandle.getSpinner().ping(this._getSpinnerId(), msg);
      }
    }

    /**
     * Deserializes and events for the list of messages
     * @param msgQueue {Array} array of strings - each string is its own message.
     */

  }, {
    key: '_handleMsgQueue',
    value: function _handleMsgQueue(msgQueue) {
      var _this7 = this;

      try {
        msgQueue.forEach(function (msg) {
          _this7.emit('message', _this7._messageHandler.deserialize(msg));
        });
      } catch (err) {
        this._log.error('Error while dispatching message on topic %s: %s', this.getTopic(), err);
        this.emit('error', err);
      }
    }
  }]);

  return SubscriberImpl;
}(EventEmitter);

//-----------------------------------------------------------------------

module.exports = SubscriberImpl;