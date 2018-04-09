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

var bunyan = require('bunyan');
var Logger = require('../utils/log/Logger.js');
var RosLogStream = require('../utils/log/RosLogStream.js');
var ConsoleLogStream = require('../utils/log/ConsoleLogStream.js');
var LogFormatter = require('../utils/log/LogFormatter.js');

//-----------------------------------------------------------------------

var DEFAULT_LOGGER_NAME = 'ros';
var LOG_CLEANUP_INTERVAL_MS = 30000; // 30 seconds

// TODO: put this in a config file somewhere
var KNOWN_LOGS = [{
  name: DEFAULT_LOGGER_NAME + '.superdebug',
  level: 'fatal'
}, {
  name: DEFAULT_LOGGER_NAME + '.rosnodejs',
  level: 'warn'
}, {
  name: DEFAULT_LOGGER_NAME + '.masterapi',
  level: 'warn'
}, {
  name: DEFAULT_LOGGER_NAME + '.params',
  level: 'warn'
}, {
  name: DEFAULT_LOGGER_NAME + '.spinner',
  level: 'error'
}];

//-----------------------------------------------------------------------

var LoggingManager = function () {
  function LoggingManager() {
    var _this = this;

    _classCallCheck(this, LoggingManager);

    this.loggerMap = {};

    // initialize the root logger with a console stream
    var rootLoggerOptions = {
      name: DEFAULT_LOGGER_NAME,
      streams: [{
        type: 'raw',
        name: 'ConsoleLogStream',
        stream: new ConsoleLogStream({ formatter: LogFormatter }),
        level: 'info'
      }],
      level: 'info'
    };
    this.rootLogger = new Logger(rootLoggerOptions);

    this._bindNodeLoggerMethods(this.rootLogger);
    this._cleanLoggersInterval = null;

    this.nameFromLevel = bunyan.nameFromLevel;
    this.levelFromName = bunyan.levelFromName;
    this.DEFAULT_LOGGER_NAME = DEFAULT_LOGGER_NAME;

    // in case the node we're running has it's own logging system, we'll
    // allow users to pass in callbacks for getting and setting loggers
    // through the logging services (_handleGetLoggers, _handleSetLoggerLevel)
    this._externalLog = {
      getLoggers: null,
      setLoggerLevel: null
    };

    KNOWN_LOGS.forEach(function (log) {
      _this.generateLogger(log);
    });
  }

  _createClass(LoggingManager, [{
    key: 'initializeNodeLogger',
    value: function initializeNodeLogger(nodeName) {
      var _this2 = this;

      var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};


      // setup desired streams
      if (options.hasOwnProperty('streams')) {
        options.streams.forEach(function (stream) {
          _this2.addStream(stream);
        });
      }
      // set desired log level
      if (options.hasOwnProperty('level')) {
        this.setLevel(options.level);
      }

      // automatically clear out expired throttled logs every so often unless specified otherwise
      if (!options.hasOwnProperty('overrideLoggerCleanup')) {
        this._cleanLoggersInterval = setInterval(this.clearThrottledLogs.bind(this), LOG_CLEANUP_INTERVAL_MS);
      }

      if (typeof options.getLoggers === 'function') {
        this._externalLog.getLoggers = options.getLoggers;
      }

      if (typeof options.setLoggerLevel === 'function') {
        this._externalLog.setLoggerLevel = options.setLoggerLevel;
      }
    }
  }, {
    key: 'initializeRosOptions',
    value: function initializeRosOptions(rosnodejs) {
      var _this3 = this;

      var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};

      if (options.skipRosLogging) {
        return Promise.resolve();
      }

      var nh = rosnodejs.nh;
      var rosLogStream = void 0;
      try {
        var rosgraphMsgs = rosnodejs.require('rosgraph_msgs');
        rosLogStream = new RosLogStream(nh, rosgraphMsgs.msg.Log);
        this.addStream({
          type: 'raw',
          name: 'RosLogStream',
          stream: rosLogStream
        });
      } catch (err) {
        this.rootLogger.warn('Unable to setup ros logging stream');
      }

      // try to set up logging services
      try {
        var roscpp = rosnodejs.require('roscpp');
        var getLoggerSrv = nh.getNodeName() + '/get_loggers';
        var setLoggerSrv = nh.getNodeName() + '/set_logger_level';
        nh.advertiseService(getLoggerSrv, roscpp.srv.GetLoggers, this._handleGetLoggers.bind(this));
        nh.advertiseService(setLoggerSrv, roscpp.srv.SetLoggerLevel, this._handleSetLoggerLevel.bind(this));
      } catch (err) {
        this.rootLogger.warn('Unable to setup ros logging services');
      }

      if (rosLogStream && options.waitOnRosOut !== undefined && options.waitOnRosOut) {
        this.rootLogger.debug('Waiting for /rosout connection before resolving node initialization...');
        return new Promise(function (resolve, reject) {
          rosLogStream.getPub().on('connection', function () {
            _this3.rootLogger.debug('Got connection to /rosout !');
            resolve();
          });
        });
      }
      return Promise.resolve();
    }
  }, {
    key: 'generateLogger',
    value: function generateLogger(options) {
      if (!options.hasOwnProperty('name')) {
        throw new Error('Unable to generate logger without name');
      }
      var loggerName = options.name;

      // don't regenerate the logger if it exists
      if (this.loggerMap.hasOwnProperty(loggerName)) {
        return this.loggerMap[loggerName];
      }
      // else
      // generate a child logger from root
      var newLogger = this._createChildLogger(loggerName, this.rootLogger, options);

      // stash the logger and return it
      this.loggerMap[loggerName] = newLogger;
      return newLogger;
    }
  }, {
    key: 'getLogger',
    value: function getLogger(loggerName, options) {
      if (!loggerName || loggerName === this.rootLogger.getName()) {
        return this.rootLogger;
      } else if (!this.hasLogger(loggerName)) {
        options = options || {};
        options.name = loggerName;
        return this.generateLogger(options);
      }
      // else
      return this.loggerMap[loggerName];
    }
  }, {
    key: 'hasLogger',
    value: function hasLogger(loggerName) {
      return this.loggerMap.hasOwnProperty(loggerName);
    }
  }, {
    key: 'removeLogger',
    value: function removeLogger(loggerName) {
      if (loggerName !== DEFAULT_LOGGER_NAME) {
        delete this.loggerMap[loggerName];
      }
    }
  }, {
    key: 'getLoggers',
    value: function getLoggers() {
      var loggerNames = Object.keys(this.loggerMap);
      loggerNames.push(this.rootLogger.getName());
      return loggerNames;
    }
  }, {
    key: 'getStreams',
    value: function getStreams() {
      return this.rootLogger.getStreams();
    }
  }, {
    key: 'getStream',
    value: function getStream(streamName) {
      var streams = this.getStreams();
      for (var i = 0; i < streams.length; ++i) {
        var stream = streams[i];
        if (stream.name === streamName) {
          return stream;
        }
      }
    }
  }, {
    key: 'setLevel',
    value: function setLevel(level) {
      this._forEachLogger(function (logger) {
        return logger.setLevel(level);
      }, true);
    }
  }, {
    key: 'addStream',
    value: function addStream(stream) {
      this._forEachLogger(function (logger) {
        return logger.addStream(stream);
      }, true);
    }
  }, {
    key: 'clearStreams',
    value: function clearStreams() {
      this._forEachLogger(function (logger) {
        return logger.clearStreams();
      }, true);
    }
  }, {
    key: 'clearThrottledLogs',
    value: function clearThrottledLogs() {
      this._forEachLogger(function (logger) {
        return logger.clearExpiredThrottledLogs();
      }, true);
    }
  }, {
    key: 'stopLogCleanup',
    value: function stopLogCleanup() {
      clearInterval(this._cleanLoggersInterval);
    }
  }, {
    key: '_handleGetLoggers',
    value: function _handleGetLoggers(req, resp) {
      if (this._externalLog.getLoggers !== null) {
        this._externalLog.getLoggers(req, resp);
      }

      this._forEachLogger(function (logger) {
        resp.loggers.push({
          name: logger.getName(),
          level: bunyan.nameFromLevel[logger.getLevel()]
        });
      }, true);

      return true;
    }
  }, {
    key: '_handleSetLoggerLevel',
    value: function _handleSetLoggerLevel(req, resp) {
      var handled = false;
      if (this._externalLog.setLoggerLevel !== null) {
        handled = this._externalLog.setLoggerLevel(req, resp);
      }

      if (!handled) {
        var logger = this.getLogger(req.logger);
        if (!logger) {
          return false;
        }
        // else
        logger.setLevel(req.level);
      }

      return true;
    }
  }, {
    key: '_bindNodeLoggerMethods',
    value: function _bindNodeLoggerMethods(logger) {
      var _this4 = this;

      var rawMethods = ['trace', 'debug', 'info', 'warn', 'error', 'fatal'];
      var methods = [];
      rawMethods.forEach(function (method) {
        return methods.push(method);
      });
      rawMethods.forEach(function (method) {
        return methods.push(method + 'Throttle');
      });
      rawMethods.forEach(function (method) {
        return methods.push(method + 'Once');
      });
      methods.forEach(function (method) {
        _this4[method] = logger[method].bind(logger);
      });
    }
  }, {
    key: '_forEachLogger',
    value: function _forEachLogger(perLoggerCallback, includeRoot) {
      var _this5 = this;

      if (includeRoot) {
        perLoggerCallback(this.rootLogger);
      }
      Object.keys(this.loggerMap).forEach(function (loggerName) {
        perLoggerCallback(_this5.loggerMap[loggerName]);
      });
    }
  }, {
    key: '_createChildLogger',
    value: function _createChildLogger(childLoggerName, parentLogger, options) {
      // setup options
      options = options || {};
      options.scope = childLoggerName;

      // create logger
      var childLogger = parentLogger.child(options);

      // cache in map
      this.loggerMap[childLoggerName] = childLogger;
      return childLogger;
    }
  }]);

  return LoggingManager;
}();

module.exports = new LoggingManager();