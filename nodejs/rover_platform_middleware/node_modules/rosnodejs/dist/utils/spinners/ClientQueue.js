'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

var EventEmitter = require('events');

/**
 * @class ClientQueue
 * Queue of messages to handle for an individual client (subscriber or publisher)
 */

var ClientQueue = function (_EventEmitter) {
  _inherits(ClientQueue, _EventEmitter);

  function ClientQueue(client, queueSize, throttleMs) {
    _classCallCheck(this, ClientQueue);

    var _this = _possibleConstructorReturn(this, (ClientQueue.__proto__ || Object.getPrototypeOf(ClientQueue)).call(this));

    if (queueSize < 1) {
      queueSize = Number.POSITIVE_INFINITY;
    }

    _this._client = client;

    _this._queue = [];
    _this._queueSize = queueSize;

    _this.throttleMs = throttleMs;
    _this._handleTime = null;
    return _this;
  }

  _createClass(ClientQueue, [{
    key: 'destroy',
    value: function destroy() {
      this._queue = [];
      this._client = null;
      this._handleTime = null;
    }
  }, {
    key: 'push',
    value: function push(item) {
      this._queue.push(item);
      if (this.length > this._queueSize) {
        this._queue.shift();
      }
    }
  }, {
    key: 'handleClientMessages',
    value: function handleClientMessages(time) {
      if (this._handleTime === null || time - this._handleTime >= this.throttleMs) {
        this._handleTime = time;
        this._client._handleMsgQueue(this._queue);
        this._queue = [];
        return true;
      }
      // else
      return false;
    }
  }, {
    key: 'length',
    get: function get() {
      return this._queue.length;
    }
  }]);

  return ClientQueue;
}(EventEmitter);

module.exports = ClientQueue;