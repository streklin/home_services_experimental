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

var Logging = require('./Logging.js');
var XmlrpcClient = require('../utils/XmlrpcClient.js');

//-----------------------------------------------------------------------

var ParamServerApiClient = function () {
  function ParamServerApiClient(xmlrpcClient) {
    _classCallCheck(this, ParamServerApiClient);

    this._log = Logging.getLogger(Logging.DEFAULT_LOGGER_NAME + '.params');

    this._xmlrpcClient = xmlrpcClient;
  }

  _createClass(ParamServerApiClient, [{
    key: '_call',
    value: function _call(method, data, resolve, reject) {
      this._xmlrpcClient.call(method, data, resolve, reject);
    }
  }, {
    key: 'deleteParam',
    value: function deleteParam(callerId, key) {
      var _this = this;

      var data = [callerId, key];

      return new Promise(function (resolve, reject) {
        _this._call('deleteParam', data, resolve, reject);
      });
    }
  }, {
    key: 'setParam',
    value: function setParam(callerId, key, value) {
      var _this2 = this;

      var data = [callerId, key, value];

      return new Promise(function (resolve, reject) {
        _this2._call('setParam', data, resolve, reject);
      });
    }
  }, {
    key: 'getParam',
    value: function getParam(callerId, key) {
      var _this3 = this;

      var data = [callerId, key];

      return new Promise(function (resolve, reject) {
        _this3._call('getParam', data, function (resp) {
          // resp[2] is the actual parameter value, and presumably all anyone cares about
          resolve(resp[2]);
        }, reject);
      });
    }
  }, {
    key: 'searchParam',
    value: function searchParam(callerId, key) {
      throw new Error('NOT IMPLEMENTED');
    }
  }, {
    key: 'subscribeParam',
    value: function subscribeParam(callerId, key) {
      throw new Error('NOT IMPLEMENTED');
    }
  }, {
    key: 'unsubscribeParam',
    value: function unsubscribeParam(callerId, key) {
      throw new Error('NOT IMPLEMENTED');
    }
  }, {
    key: 'hasParam',
    value: function hasParam(callerId, key) {
      var _this4 = this;

      var data = [callerId, key];

      return new Promise(function (resolve, reject) {
        _this4._xmlrpcClient.methodCall('hasParam', data, function (err, resp) {
          if (err || resp[0] !== 1) {
            reject(err, resp);
          } else {
            // resp[2] is whether it actually has param and presumably all anyone  cares about
            resolve(resp[2]);
          }
        });
      });
    }
  }, {
    key: 'getParamNames',
    value: function getParamNames(callerId) {
      var _this5 = this;

      var data = [callerId];

      return new Promise(function (resolve, reject) {
        _this5._xmlrpcClient.methodCall('getParamNames', data, function (err, resp) {
          if (err || resp[0] !== 1) {
            reject(err, resp);
          } else {
            // resp[2] is parameter name list and presumably all anyone cares about
            resolve(resp[2]);
          }
        });
      });
    }
  }]);

  return ParamServerApiClient;
}();

//-----------------------------------------------------------------------

module.exports = ParamServerApiClient;