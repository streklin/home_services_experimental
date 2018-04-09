'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var EventEmitter = require('events');
var xmlrpc = require('xmlrpc');

var CONNECTION_REFUSED = 'ECONNREFUSED';
var TRY_AGAIN_LIST = [1, 2, 2, 4, 4, 4, 4, 8, 8, 8, 8, 16, 16, 32, 64, 128, 256, 512, 1024, 2048];

var XmlrpcCall = function () {
  function XmlrpcCall(method, data, resolve, reject) {
    var options = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : {};

    _classCallCheck(this, XmlrpcCall);

    this.method = method;
    this.data = data;
    this.resolve = resolve;
    this.reject = reject;

    this.maxAttempts = options.maxAttempts || Infinity;
  }

  _createClass(XmlrpcCall, [{
    key: 'call',
    value: function call(client) {
      var _this = this;

      return new Promise(function (resolve, reject) {
        client.methodCall(_this.method, _this.data, function (err, resp) {
          if (err) {
            reject(err);
          } else if (resp[0] !== 1) {
            var error = new Error('ROS XMLRPC Error');
            error.code = 'EROSAPIERROR';
            error.statusCode = resp[0];
            error.statusMessage = resp[1];
            error.value = resp[2];
            reject(error);
          } else {
            resolve(resp);
          }
        });
      });
    }
  }]);

  return XmlrpcCall;
}();

var XmlrpcClient = function (_EventEmitter) {
  _inherits(XmlrpcClient, _EventEmitter);

  function XmlrpcClient(clientAddressInfo, log) {
    _classCallCheck(this, XmlrpcClient);

    var _this2 = _possibleConstructorReturn(this, (XmlrpcClient.__proto__ || Object.getPrototypeOf(XmlrpcClient)).call(this));

    _this2._xmlrpcClient = xmlrpc.createClient(clientAddressInfo);

    _this2._log = log;

    _this2._callQueue = [];

    _this2._timeout = 0;
    _this2._timeoutId = null;

    _this2._failedAttempts = 0;
    return _this2;
  }

  _createClass(XmlrpcClient, [{
    key: 'getClient',
    value: function getClient() {
      return this._xmlrpcClient;
    }
  }, {
    key: 'call',
    value: function call(method, data, resolve, reject, options) {
      var newCall = new XmlrpcCall(method, data, resolve, reject, options);
      var numCalls = this._callQueue.length;
      this._callQueue.push(newCall);
      // if nothing else was on the queue, try executing the call now
      if (numCalls === 0) {
        this._tryExecuteCall();
      }
    }
  }, {
    key: 'clear',
    value: function clear() {
      this._log.info('Clearing xmlrpc client queue...');
      if (this._callQueue.length !== 0) {
        this._callQueue[0].reject(new Error('Clearing call queue - probably shutting down...'));
      }
      clearTimeout(this._timeoutId);
      this._callQueue = [];
    }
  }, {
    key: '_tryExecuteCall',
    value: function _tryExecuteCall() {
      var _this3 = this;

      if (this._callQueue.length === 0) {
        this._log.warn('Tried executing xmlprc call on empty queue');
        return;
      }
      // else
      var call = this._callQueue[0];
      this._log.info('Try execute call %s: %j', call.method, call.data);
      call.call(this._xmlrpcClient).then(function (resp) {
        // call succeeded, clean up and call its handler
        _this3._log.info('Call %s %j succeeded! %j', call.method, call.data, resp);
        _this3._shiftQueue();
        _this3._resetTimeout();
        call.resolve(resp);
      }).catch(function (err) {
        ++_this3._failedAttempts;
        _this3._log.info('Call %s %j failed! %s', call.method, call.data, err);
        if (err instanceof Error && err.code === CONNECTION_REFUSED && _this3._failedAttempts < call.maxAttempts) {
          // Call failed to connect - try to connect again.
          // All future calls would have same error since they're
          // directed at the same xmlrpc server.
          _this3._log.info('Trying call again on attempt %d of %d', _this3._failedAttempts, call.maxAttempts);
          _this3._scheduleTryAgain();
          _this3.emit(CONNECTION_REFUSED, err, _this3._failedAttempts);
        } else {
          // call failed - move on.
          _this3._shiftQueue();
          _this3._resetTimeout();
          call.reject(err);
        }
      }).then(function () {
        if (_this3._timeoutId === null && _this3._callQueue.length > 0) {
          _this3._tryExecuteCall();
        }
      });
    }
  }, {
    key: '_shiftQueue',
    value: function _shiftQueue() {
      this._callQueue.shift();
    }
  }, {
    key: '_resetTimeout',
    value: function _resetTimeout() {
      this._timeout = 0;
      this._timeoutId = null;
      this._failedAttempts = 0;
    }
  }, {
    key: '_scheduleTryAgain',
    value: function _scheduleTryAgain() {
      var timeout = TRY_AGAIN_LIST[this._timeout];
      if (this._timeout + 1 < TRY_AGAIN_LIST.length) {
        ++this._timeout;
      }
      this._log.info('Scheduling call again in %dms', timeout);
      this._timeoutId = setTimeout(this._tryExecuteCall.bind(this), timeout);
    }
  }]);

  return XmlrpcClient;
}(EventEmitter);

module.exports = XmlrpcClient;