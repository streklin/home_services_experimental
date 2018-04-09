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

var ConsoleLogStream = function () {
  function ConsoleLogStream(options) {
    _classCallCheck(this, ConsoleLogStream);

    if (options.hasOwnProperty('formatter')) {
      this._formatter = options.formatter;
    } else {
      this._formatter = function (rec) {
        return rec.msg;
      };
    }
  }

  _createClass(ConsoleLogStream, [{
    key: 'write',
    value: function write(rec) {
      var msg = void 0;
      if (typeof rec === 'string' || rec instanceof String) {
        console.log(rec);
        return;
      } else if ((typeof rec === 'undefined' ? 'undefined' : _typeof(rec)) === 'object') {
        var formattedMsg = this._formatter(rec);
        if (typeof formattedMsg === 'string' || formattedMsg instanceof String) {
          msg = formattedMsg;
        } else {
          console.error('Unable to format message %j', rec);
          return;
        }

        var logLevel = rec.level;
        if (logLevel <= bunyan.INFO) {
          console.info(msg);
        } else if (logLevel <= bunyan.WARN) {
          console.warn(msg);
        } else {
          // logLevel === bunyan.ERROR || bunyan.FATAL
          console.error(msg);
        }
      }
    }
  }]);

  return ConsoleLogStream;
}();

;

module.exports = ConsoleLogStream;