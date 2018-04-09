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
var crypto = require('crypto');

//------------------------------------------------------------------------

/**
 * Logger is a minimal wrapper around a bunyan logger. It adds useful methods
 * to throttle/limit logging.
 * @class Logger
 */

var Logger = function () {
  function Logger(options) {
    _classCallCheck(this, Logger);

    options = options || {};

    this._name = options.name;

    if (options.$parent) {
      this._logger = options.$parent.child(options.childOptions);
    } else {
      this._logger = bunyan.createLogger({
        name: this._name,
        level: options.level || bunyan.INFO,
        streams: options.streams
      });
    }

    this._throttledLogs = new Map();
    this._onceLogs = new Set();

    var logMethods = new Set(['trace', 'debug', 'info', 'warn', 'error', 'fatal']);
    this._createLogMethods(logMethods);
    this._createThrottleLogMethods(logMethods);
    this._createOnceLogMethods(logMethods);
  }

  _createClass(Logger, [{
    key: 'getStreams',
    value: function getStreams() {
      return this._logger.streams;
    }
  }, {
    key: 'child',
    value: function child(childOptions) {
      // setup options
      var name = childOptions.name;
      delete childOptions.name;
      var options = {
        childOptions: childOptions,
        $parent: this._logger,
        name: name
      };

      // create logger
      return new Logger(options);
    }
  }, {
    key: 'level',
    value: function level() {
      var _level = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;

      this._logger.level(_level);
    }
  }, {
    key: 'setLevel',
    value: function setLevel(level) {
      this._logger.level(level);
    }
  }, {
    key: 'getLevel',
    value: function getLevel() {
      return this._logger.level();
    }
  }, {
    key: 'getName',
    value: function getName() {
      return this._name;
    }
  }, {
    key: 'addStream',
    value: function addStream(stream) {
      this._logger.addStream(stream, this.getLevel());
    }
  }, {
    key: 'clearStreams',
    value: function clearStreams() {
      this._logger.streams = [];
    }

    /**
     * Binds to bunyan logger's method for each method (info, debug, etc)
     * @param methods {Set.<String>}
     */

  }, {
    key: '_createLogMethods',
    value: function _createLogMethods(methods) {
      var _this = this;

      methods.forEach(function (method) {
        if (_this.hasOwnProperty(method)) {
          throw new Error('Unable to create method %s', method);
        }
        _this[method] = _this._logger[method].bind(_this._logger);
      });
    }

    /**
     * Attaches throttled logging functions to this object for each level method
     * (info, debug, etc)
     * e.g.
     *  logger.infoThrottle(1000, 'Hi');
     *  logger.debugThrottle(1000, 'Hi');
     * Logs are throttled by a String key taken from the second argument (first
     * should always be throttle time in ms). So if you're logging something with
     * variable values, using format strings should be preferred since it will
     * throttle appropriately while composition will not.
     * e.g.
     *   let i = 0;
     *   setInterval(() => {
     *     logger.infoThrottle(1000, 'Counter: %d', i); // prints once second
     *     logger.infoThrottle(1000, 'Counter: ' + i);  // prints twice a second
     *     ++i;
     *   }, 500);
     *
     * @param methods {Set.<String>}
     */

  }, {
    key: '_createThrottleLogMethods',
    value: function _createThrottleLogMethods(methods) {
      var _this2 = this;

      methods.forEach(function (method) {
        var throttleMethod = method + 'Throttle';
        if (_this2.hasOwnProperty(throttleMethod)) {
          throw new Error('Unable to create method %s', throttleMethod);
        }

        // there's currently a bug using arguments in a () => {} function
        _this2[throttleMethod] = function (throttleTimeMs, args) {
          // If the desired log level is enabled and the message
          // isn't being throttled, then log the message.
          if (this[method]() && !this._throttle.apply(this, arguments)) {
            return this[method].apply(this, Array.from(arguments).slice(1));
          }
          return false;
        }.bind(_this2);
      });
    }
  }, {
    key: '_createOnceLogMethods',
    value: function _createOnceLogMethods(methods) {
      var _this3 = this;

      methods.forEach(function (method) {
        var onceMethod = method + 'Once';
        if (_this3.hasOwnProperty(onceMethod)) {
          throw new Error('Unable to create method %s', onceMethod);
        }

        // there's currently a bug using arguments in a () => {} function
        _this3[onceMethod] = function (args) {
          if (this[method]() && this._once(arguments)) {
            return this[method].apply(this, arguments);
          }
          return false;
        }.bind(_this3);
      });
    }

    //--------------------------------------------------------------
    // Throttled loggers
    //  These will generally be slower. Performance will also degrade the more
    //  places where you throttle your logs. Keep this in mind. Make child loggers.
    //--------------------------------------------------------------

    /**
     * Handles throttling logic for each log statement. Throttles logs by attempting
     * to create a string log 'key' from the arguments.
     * @param throttleTimeMs {number}
     * @param args {Array} arguments provided to calling function
     * @return {boolean} should this log be throttled (if true, the log should not be written)
     */

  }, {
    key: '_throttle',
    value: function _throttle(throttleTimeMs) {
      var now = Date.now();

      for (var _len = arguments.length, args = Array(_len > 1 ? _len - 1 : 0), _key = 1; _key < _len; _key++) {
        args[_key - 1] = arguments[_key];
      }

      var throttlingMsg = this._getThrottleMsg(args);
      if (throttlingMsg === null) {
        // we couldn't get a msg to hash - fall through and log the message
        return false;
      }
      // else
      var msgHash = hashMessage(throttlingMsg);

      var throttledLog = this._throttledLogs.get(msgHash);

      if (throttledLog === undefined || now + 1 - throttledLog.getStartTime() >= throttledLog.getThrottleTime()) {
        var newThrottledLog = new ThrottledLog(now, throttleTimeMs);
        this._throttledLogs.set(msgHash, newThrottledLog);
        return false;
      }
      return true;
    }

    //--------------------------------------------------------------
    // Throttled loggers
    //  These will generally be slower. Performance will also degrade the more
    //  places where you throttle your logs. Keep this in mind. Make child loggers.
    //--------------------------------------------------------------

    /**
     * Handles once logic for each log statement. Throttles logs by attempting
     * to create a string log 'key' from the arguments.
     * @param args {Array} arguments provided to calling function
     * @return {boolean} should this be written
     */

  }, {
    key: '_once',
    value: function _once(args) {
      var throttleMsg = this._getThrottleMsg(args);
      if (throttleMsg === null) {
        // we couldn't get a msg to hash - fall through and log the message
        return true;
      }

      var logKey = hashMessage(throttleMsg);

      if (!this._onceLogs.has(logKey)) {
        this._onceLogs.add(logKey);
        return true;
      }
      return false;
    }
  }, {
    key: '_getThrottleMsg',
    value: function _getThrottleMsg(args) {
      var firstArg = args[0];
      if (typeof firstArg === 'string' || firstArg instanceof String) {
        return firstArg;
      } else if ((typeof firstArg === 'undefined' ? 'undefined' : _typeof(firstArg)) === 'object') {
        // bunyan supports passing an object as the first argument with
        // optional fields to add to the log record - the second argument
        // is the actual string 'log message' in this case, so just return that
        return args[1];
      }
      // fall through *womp womp*
      return null;
    }

    /**
     * Remove old throttled logs (logs that were throttled whose throttling time has passed) from the throttling map
     * @returns {Number} number of logs that were cleaned out
     */

  }, {
    key: 'clearExpiredThrottledLogs',
    value: function clearExpiredThrottledLogs() {
      var _this4 = this;

      var logsToRemove = [];
      var now = Date.now();
      this._throttledLogs.forEach(function (log, key) {
        if (now - log.getStartTime() >= log.getThrottleTime()) {
          logsToRemove.push(key);
        }
      });

      logsToRemove.forEach(function (logKey) {
        _this4._throttledLogs.delete(logKey);
      });

      return logsToRemove.length;
    }
  }, {
    key: 'getThrottledLogSize',
    value: function getThrottledLogSize() {
      return this._throttledLogs.size;
    }
  }]);

  return Logger;
}();

//-----------------------------------------------------------------------

/**
 * @class ThrottledLog
 * Small utility class implementation for ThrottledLogger
 */


var ThrottledLog = function () {
  function ThrottledLog(timeThrottleStarted, throttlingTime) {
    _classCallCheck(this, ThrottledLog);

    this.logThrottleStartTime = timeThrottleStarted;
    this.logthrottleTimeMs = throttlingTime;
  }

  _createClass(ThrottledLog, [{
    key: 'getStartTime',
    value: function getStartTime() {
      return this.logThrottleStartTime;
    }
  }, {
    key: 'getThrottleTime',
    value: function getThrottleTime() {
      return this.logthrottleTimeMs;
    }
  }]);

  return ThrottledLog;
}();

// Utility function to help hash messages when we throttle them.


function hashMessage(msg) {
  var sha1 = crypto.createHash('sha1');
  sha1.update(msg);
  return sha1.digest('hex');
}

//-----------------------------------------------------------------------

module.exports = Logger;