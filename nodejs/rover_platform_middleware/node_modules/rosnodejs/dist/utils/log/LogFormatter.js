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

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var moment = require('moment');
var bunyan = require('bunyan');
var timeUtils = require('../time_utils.js');

var DEFAULT_FORMAT = '[${severity}] [${time}] (${logger}): ${message}';
var CONSOLE_FORMAT = process.env.ROSCONSOLE_JS_FORMAT || DEFAULT_FORMAT;
var CONSOLE_TOKEN_REGEX = /\${([a-z|A-Z]+)}/g;

var LogFormatter = function () {
  function LogFormatter() {
    _classCallCheck(this, LogFormatter);

    this._tokens = [];

    this._parseFormat();
    this._numTokens = this._tokens.length;
  }

  _createClass(LogFormatter, [{
    key: '_parseFormat',
    value: function _parseFormat() {
      var match = void 0;
      var lastMatchIndex = 0;
      while ((match = CONSOLE_TOKEN_REGEX.exec(CONSOLE_FORMAT)) !== null) {
        var preToken = CONSOLE_FORMAT.substr(lastMatchIndex, match.index - lastMatchIndex);
        if (preToken.length > 0) {
          this._tokens.push(new DefaultToken(preToken));
        }
        this._tokens.push(this._getTokenizer(match[1]));
        lastMatchIndex = match.index + match[0].length;
      }
      var postToken = CONSOLE_FORMAT.substr(lastMatchIndex);
      if (postToken.length > 0) {
        this._tokens.push(new DefaultToken(postToken));
      }
    }
  }, {
    key: '_getTokenizer',
    value: function _getTokenizer(token) {
      switch (token) {
        case 'severity':
          return new SeverityToken();
        case 'message':
          return new MessageToken();
        case 'time':
          return new TimeToken();
        case 'logger':
          return new LoggerToken();
        case 'isodate':
          return new IsoDateToken();
        default:
          return new DefaultToken(token);
      }
    }
  }, {
    key: 'format',
    value: function format(rec) {
      var fields = this._tokens.map(function (token) {
        return token.format(rec);
      });
      return fields.join('');
    }
  }]);

  return LogFormatter;
}();

// ----------------------------------------------------------------------------------------
// Tokens used for log formatting

var DefaultToken = function () {
  function DefaultToken(val) {
    _classCallCheck(this, DefaultToken);

    this.val = val;
  }

  _createClass(DefaultToken, [{
    key: 'format',
    value: function format() {
      return this.val;
    }
  }]);

  return DefaultToken;
}();

var SeverityToken = function () {
  function SeverityToken() {
    _classCallCheck(this, SeverityToken);
  }

  _createClass(SeverityToken, [{
    key: 'format',
    value: function format(rec) {
      return bunyan.nameFromLevel[rec.level].toUpperCase();
    }
  }]);

  return SeverityToken;
}();

var MessageToken = function () {
  function MessageToken() {
    _classCallCheck(this, MessageToken);
  }

  _createClass(MessageToken, [{
    key: 'format',
    value: function format(rec) {
      return rec.msg;
    }
  }]);

  return MessageToken;
}();

var TimeToken = function () {
  function TimeToken() {
    _classCallCheck(this, TimeToken);
  }

  _createClass(TimeToken, [{
    key: 'format',
    value: function format(rec) {
      var recTime = rec.time;
      return '' + (recTime / 1000).toFixed(3);
    }
  }]);

  return TimeToken;
}();

var LoggerToken = function () {
  function LoggerToken() {
    _classCallCheck(this, LoggerToken);
  }

  _createClass(LoggerToken, [{
    key: 'format',
    value: function format(rec) {
      return rec.scope || rec.name;
    }
  }]);

  return LoggerToken;
}();

var IsoDateToken = function () {
  function IsoDateToken() {
    _classCallCheck(this, IsoDateToken);
  }

  _createClass(IsoDateToken, [{
    key: 'format',
    value: function format(rec) {
      return moment(rec.time).format('YYYY-MM-DDTHH:mm:ss.SSSZZ');
    }
  }]);

  return IsoDateToken;
}();

var logFormatter = new LogFormatter();
module.exports = logFormatter.format.bind(logFormatter);