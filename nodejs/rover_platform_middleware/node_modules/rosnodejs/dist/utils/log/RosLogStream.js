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

'use strict';

var _typeof = typeof Symbol === "function" && typeof Symbol.iterator === "symbol" ? function (obj) { return typeof obj; } : function (obj) { return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; };

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var bunyan = require('bunyan');
var timeUtils = require('../time_utils.js');

var RosgraphLogMsg = void 0;

var RosLogStream = function () {
  function RosLogStream(nh, rosgraphLogMsg, options) {
    _classCallCheck(this, RosLogStream);

    RosgraphLogMsg = rosgraphLogMsg;
    options = options || {};

    var queueSize = 200;
    if (options.hasOwnProperty('queueSize')) {
      queueSize = options.queueSize;
    }

    if (options.hasOwnProperty('formatter')) {
      this._formatter = options.formatter;
    } else {
      this._formatter = function (rec) {
        return rec.msg;
      };
    }

    this._nodeName = nh.getNodeName();

    this._rosoutPub = nh.advertise('/rosout', 'rosgraph_msgs/Log', { queueSize: queueSize, latching: true });
  }

  _createClass(RosLogStream, [{
    key: 'getPub',
    value: function getPub() {
      return this._rosoutPub;
    }
  }, {
    key: '_getRosLogLevel',
    value: function _getRosLogLevel(bunyanLogLevel) {
      // ROS log levels defined in rosgraph_msgs/Log
      // bunyan trace and debug levels will both map to ROS debug
      if (bunyanLogLevel === bunyan.TRACE) {
        return RosgraphLogMsg.Constants.DEBUG;
      }
      // else
      return RosgraphLogMsg.Constants[bunyan.nameFromLevel[bunyanLogLevel].toUpperCase()];
    }
  }, {
    key: 'write',
    value: function write(rec) {
      if (this._rosoutPub !== null) {
        var msg = new RosgraphLogMsg();

        if (typeof rec === 'string' || rec instanceof String) {
          // if user sets this stream to be type 'stream' instead of 'raw', we'll
          // get the formatted string and nothing else - just send it as is.
          msg.msg = rec;
        } else if ((typeof rec === 'undefined' ? 'undefined' : _typeof(rec)) === 'object') {
          // being used as a raw stream
          msg.header.stamp = timeUtils.dateToRosTime(rec.time);

          msg.name = this._nodeName;
          msg.file = rec.scope || rec.name;
          msg.level = this._getRosLogLevel(rec.level);
          var formattedMsg = this._formatter(rec);
          if (typeof formattedMsg === 'string' || formattedMsg instanceof String) {
            msg.msg = formattedMsg;
          } else {
            console.error('Unable to format message %j', rec);
            return;
          }
        }
        // console.log('ROSOUT: %j', msg);
        this._rosoutPub.publish(msg);
      }
    }
  }]);

  return RosLogStream;
}();

;

module.exports = RosLogStream;