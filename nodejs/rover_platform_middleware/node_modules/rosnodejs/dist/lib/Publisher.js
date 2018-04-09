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

var EventEmitter = require('events');
var Ultron = require('ultron');

var _require = require('../utils/event_utils.js'),
    rebroadcast = _require.rebroadcast;

/**
 * @class Publisher
 * Public facing publishers class. Allows users to send messages to subscribers
 * on a given topic.
 */


var Publisher = function (_EventEmitter) {
  _inherits(Publisher, _EventEmitter);

  function Publisher(impl) {
    _classCallCheck(this, Publisher);

    var _this = _possibleConstructorReturn(this, (Publisher.__proto__ || Object.getPrototypeOf(Publisher)).call(this));

    ++impl.count;
    _this._impl = impl;
    _this._ultron = new Ultron(impl);

    _this._topic = impl.getTopic();
    _this._type = impl.getType();

    rebroadcast('registered', _this._ultron, _this);
    rebroadcast('connection', _this._ultron, _this);
    rebroadcast('disconnect', _this._ultron, _this);
    rebroadcast('error', _this._ultron, _this);
    return _this;
  }

  /**
   * Get the topic this publisher is publishing on
   * @returns {string}
   */


  _createClass(Publisher, [{
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
     * Check if this publisher is latching
     * @returns {boolean}
     */

  }, {
    key: 'getLatching',
    value: function getLatching() {
      if (this._impl) {
        return this._impl.getLatching();
      }
      // else
      return false;
    }

    /**
     * Get the numbber of subscribers currently connected to this publisher
     * @returns {number}
     */

  }, {
    key: 'getNumSubscribers',
    value: function getNumSubscribers() {
      if (this._impl) {
        return this._impl.getNumSubscribers();
      }
      // else
      return 0;
    }

    /**
     * Shuts down this publisher. If this is the last publisher on this topic
     * for this node, closes the publisher and unregisters the topic from Master
     * @returns {Promise}
     */

  }, {
    key: 'shutdown',
    value: function shutdown() {
      var topic = this.getTopic();
      if (this._impl) {
        var impl = this._impl;
        this._impl = null;
        this._ultron.destroy();
        this._ultron = null;

        --impl.count;
        if (impl.count <= 0) {
          return impl.getNode().unadvertise(impl.getTopic());
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

    /**
     * Schedule the msg for publishing - or publish immediately if we're
     * supposed to
     * @param msg {object} object type matching this._type
     * @param [throttleMs] {number} optional override for publisher setting
     */

  }, {
    key: 'publish',
    value: function publish(msg, throttleMs) {
      this._impl.publish(msg, throttleMs);
    }
  }]);

  return Publisher;
}(EventEmitter);

module.exports = Publisher;