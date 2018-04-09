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

var EventEmitter = require('events');
var Ultron = require('ultron');

var _require = require('../utils/event_utils.js'),
    rebroadcast = _require.rebroadcast;

//-----------------------------------------------------------------------

/**
 * @class Subscriber
 * Public facing subscriber class. Allows users to listen to messages from
 * publishers on a given topic.
 */


var Subscriber = function (_EventEmitter) {
  _inherits(Subscriber, _EventEmitter);

  function Subscriber(impl) {
    _classCallCheck(this, Subscriber);

    var _this = _possibleConstructorReturn(this, (Subscriber.__proto__ || Object.getPrototypeOf(Subscriber)).call(this));

    ++impl.count;
    _this._impl = impl;
    _this._ultron = new Ultron(impl);

    _this._topic = impl.getTopic();
    _this._type = impl.getType();

    rebroadcast('registered', _this._ultron, _this);
    rebroadcast('connection', _this._ultron, _this);
    rebroadcast('disconnect', _this._ultron, _this);
    rebroadcast('error', _this._ultron, _this);
    rebroadcast('message', _this._ultron, _this);
    return _this;
  }

  /**
   * Get the topic this publisher is publishing on
   * @returns {string}
   */


  _createClass(Subscriber, [{
    key: 'getTopic',
    value: function getTopic() {
      return this._topic;
    }

    /**
     * Get the type of message this publisher is sending
     *            (e.g. std_msgs/String)
     * @returns {string}
     */

  }, {
    key: 'getType',
    value: function getType() {
      return this._type;
    }

    /**
     * Get the number of publishers currently connected to this subscriber
     * @returns {number}
     */

  }, {
    key: 'getNumPublishers',
    value: function getNumPublishers() {
      if (this._impl) {
        return this._impl.getNumPublishers();
      }
      // else
      return 0;
    }

    /**
     * Shuts down this subscriber. If this is the last subscriber on this topic
     * for this node, closes the subscriber and unregisters the topic from Master
     * @returns {Promise}
     */

  }, {
    key: 'shutdown',
    value: function shutdown() {
      if (this._impl) {
        var impl = this._impl;
        this._impl = null;
        this._ultron.destroy();
        this._ultron = null;

        --impl.count;
        if (impl.count <= 0) {
          return impl.getNode().unsubscribe(impl.getTopic());
        }

        this.removeAllListeners();
      }
      // else
      return Promise.resolve();
    }

    /**
     * Check if this publisher has been shutdown
     * @returns {boolean}
     */

  }, {
    key: 'isShutdown',
    value: function isShutdown() {
      return !!this._impl;
    }
  }]);

  return Subscriber;
}(EventEmitter);

//-----------------------------------------------------------------------

module.exports = Subscriber;