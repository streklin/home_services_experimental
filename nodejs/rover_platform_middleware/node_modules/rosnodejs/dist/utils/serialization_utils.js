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

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

var util = require('util');
var Transform = require('stream').Transform;

//-----------------------------------------------------------------------

/**
 * DeserializeStream handles parsing of message chunks for TCPROS
 * encoded messages. When a full message has been received, it
 * emits 'message' with the data for that message. All socket
 * communications should be piped through this.
 */

var DeserializeStream = function (_Transform) {
  _inherits(DeserializeStream, _Transform);

  function DeserializeStream(options) {
    _classCallCheck(this, DeserializeStream);

    // Transform.call(this, options);
    // true once we've pulled off the message length
    // for the next message we'll need to deserialize
    var _this = _possibleConstructorReturn(this, (DeserializeStream.__proto__ || Object.getPrototypeOf(DeserializeStream)).call(this, options));

    _this._inBody = false;

    // track how many bytes of this message we've received so far
    _this._messageConsumed = 0;

    // how long this message will be
    _this._messageLen = -1;

    // as bytes of this message arrive, store them in this
    // buffer until we have the whole thing
    _this._messageBuffer = [];

    // TODO: These are specific to parsing a service response...
    //   don't use them everywhere
    // the first byte in a service response is true/false service success/fail
    _this._deserializeServiceResp = false;

    _this._serviceRespSuccess = null;
    return _this;
  }

  _createClass(DeserializeStream, [{
    key: '_transform',
    value: function _transform(chunk, encoding, done) {
      var pos = 0;
      var chunkLen = chunk.length;

      while (pos < chunkLen) {
        if (this._inBody) {
          var messageRemaining = this._messageLen - this._messageConsumed;

          // if the chunk is longer than the amount of the message we have left
          // just pull off what we need
          if (chunkLen >= messageRemaining + pos) {
            var slice = chunk.slice(pos, pos + messageRemaining);
            this._messageBuffer.push(slice);
            var concatBuf = Buffer.concat(this._messageBuffer, this._messageLen);
            this.emitMessage(concatBuf);

            // message finished, reset
            this._messageBuffer = [];
            pos += messageRemaining;
            this._inBody = false;
            this._messageConsumed = 0;
          } else {
            // rest of the chunk does not complete the message
            // cache it and move on
            this._messageBuffer.push(chunk.slice(pos));
            this._messageConsumed += chunkLen - pos;
            pos = chunkLen;
          }
        } else {
          // if we're deserializing a service response, first byte is 'success'
          if (this._deserializeServiceResp && this._serviceRespSuccess === null) {
            this._serviceRespSuccess = chunk.readUInt8(pos, true);
            ++pos;
          }

          var bufLen = 0;
          this._messageBuffer.forEach(function (bufferEntry) {
            bufLen += bufferEntry.length;
          });

          // first 4 bytes of the message are a uint32 length field
          if (chunkLen - pos >= 4 - bufLen) {
            this._messageBuffer.push(chunk.slice(pos, pos + 4 - bufLen));
            var buffer = Buffer.concat(this._messageBuffer, 4);
            this._messageLen = buffer.readUInt32LE(0);
            pos += 4 - bufLen;

            this._messageBuffer = [];
            // if its an empty message, there won't be any bytes left and message
            // will never be emitted -- handle that case here
            if (this._messageLen === 0 && pos === chunkLen) {
              this.emitMessage(new Buffer([]));
            } else {
              this._inBody = true;
            }
          } else {
            // the length field is split on a chunk
            this._messageBuffer.push(chunk.slice(pos));
            pos = chunkLen;
          }
        }
      }
      done();
    }
  }, {
    key: 'emitMessage',
    value: function emitMessage(buffer) {
      if (this._deserializeServiceResp) {
        this.emit('message', buffer, this._serviceRespSuccess);
        this._serviceRespSuccess = null;
      } else {
        this.emit('message', buffer);
      }
    }
  }, {
    key: 'setServiceRespDeserialize',
    value: function setServiceRespDeserialize() {
      this._deserializeServiceResp = true;
    }
  }]);

  return DeserializeStream;
}(Transform);

;

//-----------------------------------------------------------------------

function PrependLength(buffer, len) {
  var lenBuf = new Buffer(4);
  lenBuf.writeUInt32LE(len, 0);
  return Buffer.concat([lenBuf, buffer], buffer.length + 4);
}

//-----------------------------------------------------------------------

var SerializationUtils = {
  DeserializeStream: DeserializeStream,

  PrependLength: PrependLength,

  Serialize: function Serialize(buffer) {
    return PrependLength(buffer, buffer.length);
  },
  Deserialize: function Deserialize(buffer) {
    var len = buffer.readUInt32LE(0, true);
    buffer = buffer.slice(4);
    return len;
  }
};

//-----------------------------------------------------------------------

module.exports = SerializationUtils;