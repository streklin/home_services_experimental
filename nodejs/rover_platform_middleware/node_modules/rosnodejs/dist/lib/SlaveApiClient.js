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

var xmlrpc = require('xmlrpc');

//-----------------------------------------------------------------------

var SlaveApiClient = function () {
  function SlaveApiClient(host, port) {
    _classCallCheck(this, SlaveApiClient);

    this._xmlrpcClient = xmlrpc.createClient({ host: host, port: port });
  }

  _createClass(SlaveApiClient, [{
    key: 'requestTopic',
    value: function requestTopic(callerId, topic, protocols) {
      var _this = this;

      var data = [callerId, topic, protocols];
      return new Promise(function (resolve, reject) {
        _this._xmlrpcClient.methodCall('requestTopic', data, function (err, resp) {
          if (err || resp[0] !== 1) {
            reject(err, resp);
          } else {
            resolve(resp);
          }
        });
      });
    }
  }]);

  return SlaveApiClient;
}();

;

//-----------------------------------------------------------------------

module.exports = SlaveApiClient;