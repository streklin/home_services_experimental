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

var timeUtils = require('../utils/time_utils.js');
var msgUtils = require('../utils/message_utils.js');
var EventEmitter = require('events');
var GoalID = null;
var Header = null;

var ActionClient = function (_EventEmitter) {
  _inherits(ActionClient, _EventEmitter);

  function ActionClient(options) {
    _classCallCheck(this, ActionClient);

    var _this = _possibleConstructorReturn(this, (ActionClient.__proto__ || Object.getPrototypeOf(ActionClient)).call(this));

    if (GoalID === null) {
      GoalID = msgUtils.requireMsgPackage('actionlib_msgs').msg.GoalID;
    }

    if (Header === null) {
      Header = msgUtils.requireMsgPackage('std_msgs').msg.Header;
    }

    _this._actionType = options.type;

    _this._actionServer = options.actionServer;

    var nh = options.nh;

    var goalOptions = Object.assign({ queueSize: 10, latching: true }, options.goal);
    _this._goalPub = nh.advertise(_this._actionServer + '/goal', _this._actionType + 'Goal', goalOptions);

    var cancelOptions = Object.assign({ queueSize: 10, latching: true }, options.cancel);
    _this._cancelPub = nh.advertise(_this._actionServer + '/cancel', 'actionlib_msgs/GoalID', cancelOptions);

    var statusOptions = Object.assign({ queueSize: 1 }, options.status);
    _this._statusSub = nh.subscribe(_this._actionServer + '/status', 'actionlib_msgs/GoalStatusArray', function (msg) {
      _this._handleStatus(msg);
    }, statusOptions);

    var feedbackOptions = Object.assign({ queueSize: 1 }, options.feedback);
    _this._feedbackSub = nh.subscribe(_this._actionServer + '/feedback', _this._actionType + 'Feedback', function (msg) {
      _this._handleFeedback(msg);
    }, feedbackOptions);

    var resultOptions = Object.assign({ queueSize: 1 }, options.result);
    _this._resultSub = nh.subscribe(_this._actionServer + '/result', _this._actionType + 'Result', function (msg) {
      _this._handleResult(msg);
    }, resultOptions);

    _this._goals = {};
    _this._goalCallbacks = {};
    _this._goalSeqNum = 0;
    return _this;
  }

  _createClass(ActionClient, [{
    key: '_handleStatus',
    value: function _handleStatus(msg) {
      this.emit('status', msg);
    }
  }, {
    key: '_handleFeedback',
    value: function _handleFeedback(msg) {
      var goalId = msg.status.goal_id.id;
      if (this._goals.hasOwnProperty(goalId)) {
        this.emit('feedback', msg);
      }
    }
  }, {
    key: '_handleResult',
    value: function _handleResult(msg) {
      var goalId = msg.status.goal_id.id;
      if (this._goals.hasOwnProperty(goalId)) {
        delete this._goals[goalId];
        this.emit('result', msg);
      }
    }

    /**
     * Cancel the given goal. If none is given, send an empty goal message,
     * i.e. cancel all goals. See
     * http://wiki.ros.org/actionlib/DetailedDescription#The_Messages
     * @param [goalId] {string} id of the goal to cancel
     */

  }, {
    key: 'cancel',
    value: function cancel(goalId) {
      var cancelGoal = new GoalID({ stamp: timeUtils.now() });
      if (!goalId) {
        this._cancelPub.publish(cancelGoal);
      } else if (this._goals.hasOwnProperty(goalId)) {
        cancelGoal.id = goalId;
        this._cancelPub.publish(cancelGoal);
      }
    }
  }, {
    key: 'sendGoal',
    value: function sendGoal(goal) {
      if (!goal.goal_id) {
        goal.goal_id = new GoalID({
          stamp: timeUtils.now(),
          id: this.generateGoalId()
        });
      }
      if (!goal.header) {
        goal.header = new Header({
          seq: this._goalSeqNum++,
          stamp: goal.goal_id.stamp,
          frame_id: 'auto-generated'
        });
      }
      var goalId = goal.goal_id.id;
      this._goals[goalId] = goal;

      this._goalPub.publish(goal);
      return goal;
    }
  }, {
    key: 'generateGoalId',
    value: function generateGoalId() {
      var id = this._actionType + '.';
      id += 'xxxxxxxx'.replace(/[x]/g, function (c) {
        return (Math.random() * 16).toString(16);
      });
      return id;
    }
  }]);

  return ActionClient;
}(EventEmitter);

module.exports = ActionClient;