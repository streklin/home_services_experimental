'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var fs = require('fs');
var util = require('util');
var path = require('path');
var md5 = require('md5');
var fieldsUtil = require('./fields.js');
var IndentedWriter = require('./IndentedWriter.js');
var MessageWriter = require('./MessageWriter.js');

var MSG_DIVIDER = '---';

var MSG_TYPE = 'msg';
var SRV_TYPE = 'srv';
var SRV_REQUEST_TYPE = 'srvRequest';
var SRV_RESPONSE_TYPE = 'srvResponse';
var ACTION_TYPE = 'action';
var ACTION_GOAL_TYPE = 'actionGoal';
var ACTION_FEEDBACK_TYPE = 'actionFeedback';
var ACTION_RESULT_TYPE = 'actionResult';
var ACTION_ACTION_GOAL_TYPE = 'actionGoal';
var ACTION_ACTION_FEEDBACK_TYPE = 'actionActionFeedback';
var ACTION_ACTION_RESULT_TYPE = 'actionActionResult';
var ACTION_ACTION_TYPE = 'actionAction';

function _getFullMessageName(packageName, messageName) {
  return packageName + '/' + messageName;
}

function getPackageNameFromMessageType(messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[0] : '';
}

var isArrayRegex = /.*\[*\]$/;
function isArray(fieldType) {
  return fieldType.match(isArrayRegex) !== null;
}

function getLengthOfArray(arrayType) {
  var match = arrayType.match(/.*\[(\d*)\]$/);
  if (match[1] === '') {
    return null;
  }
  return parseInt(match[1]);
}

function parseType(msgType) {
  if (!msgType) {
    throw new Error('Invalid empty type ' + JSON.stringify(field));
  }
  // else
  var field = {};
  if (isArray(msgType)) {
    field.isArray = true;
    var variableLength = msgType.endsWith('[]');
    var splits = msgType.split('[');
    if (splits.length > 2) {
      throw new Error('Only support 1-dimensional array types: ' + msgType);
    }
    field.baseType = splits[0];
    if (!variableLength) {
      field.arrayLen = getLengthOfArray(msgType);
    } else {
      field.arrayLen = null;
    }
  } else {
    field.baseType = msgType;
    field.isArray = false;
    field.arrayLen = null;
  }
  return field;
}

function isHeader(type) {
  return ['Header', 'std_msgs/Header', 'roslib/Header'].indexOf(type) >= 0;
}

var Field = function () {
  function Field(name, type) {
    _classCallCheck(this, Field);

    this.name = name;
    this.type = type;
    Object.assign(this, parseType(type));
    this.isHeader = isHeader(type);
    this.isBuiltin = fieldsUtil.isPrimitive(this.baseType);
  }

  _createClass(Field, [{
    key: 'getPackage',
    value: function getPackage() {
      if (this.isBuiltin) {
        return null;
      }
      return this.baseType.split('/')[0];
    }
  }, {
    key: 'getMessage',
    value: function getMessage() {
      if (this.isBuiltin) {
        return null;
      }
      return this.baseType.split('/')[1];
    }
  }]);

  return Field;
}();

/**
 * @class RosMsgSpec
 * Base class for message spec. Provides useful functionality on its own that is extended
 * by subclasses.
 */


var RosMsgSpec = function () {
  /**
   * Constructor for base class
   * @param msgCache {MessageManager}
   * @param packageName {string} name of package
   * @param messageName {string} name of message
   * @param type {string} type of message (see MSG_TYPE, SRV_TYPE, ... above)
   * @param filePath {string|null} path to message file
   * @returns {RosMsgSpec}
   */
  function RosMsgSpec(msgCache, packageName, messageName, type) {
    var filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    _classCallCheck(this, RosMsgSpec);

    this.msgCache = msgCache;
    this.messageName = messageName;
    this.packageName = packageName;
    this.type = type;
    this.fileContents = null;
  }

  /**
   * Given a type of message, returns the correct subclass of RosMsgSpec
   * @param msgCache {MessageManager}
   * @param packageName {string} name of package
   * @param messageName {string} name of message
   * @param type {string} type of message (see MSG_TYPE, SRV_TYPE, ... above)
   * @param filePath {string|null} path to message file
   * @returns {SrvSpec|MsgSpec|ActionSpec}
   */


  _createClass(RosMsgSpec, [{
    key: 'getMsgSpecForType',


    /**
     * Query the cache for another message spec
     * @param type {string} full type of message to search for (e.g. sensor_msgs/Image)
     * @returns {RosMsgSpec}
     */
    value: function getMsgSpecForType(type) {
      return this.msgCache.getMessageSpec(type);
    }

    /**
     * Tries to load and parse message file
     * @param [filePath] {string} path to file - will load file from here if provided
     * @param [fileContents] {string} file contents - will parse into desired fields
     */

  }, {
    key: 'loadFile',
    value: function loadFile() {
      var filePath = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;
      var fileContents = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;


      if (filePath !== null) {
        fileContents = this._loadMessageFile(filePath);
      }
      if (fileContents !== null) {
        this.fileContents = fileContents;

        this._parseMessage(fileContents);
      }
    }
  }, {
    key: '_parseMessage',
    value: function _parseMessage() {
      throw new Error('Unable to parse message file for base class RosMsgSpec');
    }

    /**
     * Generates the file data for this class
     */

  }, {
    key: 'writeMessageClassFile',
    value: function writeMessageClassFile() {
      throw new Error('Unable to write message class file for base class RosMsgSpec');
    }

    /**
     * Get full message name for this spec (e.g. sensor_msgs/String)
     * @returns {string}
     */

  }, {
    key: 'getFullMessageName',
    value: function getFullMessageName() {
      return _getFullMessageName(this.packageName, this.messageName);
    }

    /**
     * Get a unique list of other packages this spec depends on
     * @param [deps] {Set} dependencies will be added to this set if provided
     * @returns {Set} list of dependencies
     */

  }, {
    key: 'getMessageDependencies',
    value: function getMessageDependencies() {
      var deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();
      var packageName = this.packageName;

      this.fields.forEach(function (field) {
        var fieldPackage = getPackageNameFromMessageType(field.baseType);
        if (!field.isBuiltin && fieldPackage !== packageName) {
          if (field.isHeader) {
            deps.add('std_msgs');
          } else {
            deps.add(fieldPackage);
          }
        }
      });
      return deps;
    }

    /**
     * Reads file at specified location and returns its contents
     * @param fileName {string}
     * @returns fileContents {string}
     * @private
     */

  }, {
    key: '_loadMessageFile',
    value: function _loadMessageFile(fileName) {
      return fs.readFileSync(fileName, 'utf8');
    }

    /**
     * For this message spec, generates the text used to calculate the message's md5 sum
     * @returns {string}
     */

  }, {
    key: 'getMd5text',
    value: function getMd5text() {
      return '';
    }

    /**
     * Get the md5 sum of this message
     * @returns {string}
     */

  }, {
    key: 'getMd5sum',
    value: function getMd5sum() {
      return md5(this.getMd5text());
    }

    /**
     * Generates a depth-first list of all dependencies of this message in field order.
     * @param [deps] {Array}
     * @returns {Array}
     */

  }, {
    key: 'getFullDependencies',
    value: function getFullDependencies() {
      var deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : [];

      return [];
    }

    /**
     * Computes the full text of a message/service.
     * Necessary for rosbags.
     * Mirrors gentools.
     * See compute_full_text() in
     *   https://github.com/ros/ros/blob/kinetic-devel/core/roslib/src/roslib/gentools.py
     */

  }, {
    key: 'computeFullText',
    value: function computeFullText() {
      var w = new IndentedWriter();

      var deps = this.getFullDependencies();
      var sep = '='.repeat(80);
      w.write(this.fileContents.trim()).newline();

      deps.forEach(function (dep) {
        w.write(sep).write('MSG: ' + dep.getFullMessageName()).write(dep.fileContents.trim()).newline();
      });

      return w.get().trim();
    }
  }], [{
    key: 'create',
    value: function create(msgCache, packageName, messageName, type) {
      var filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

      switch (type) {
        case SRV_TYPE:
          return new SrvSpec(msgCache, packageName, messageName, type, filePath);
        case MSG_TYPE:
          return new MsgSpec(msgCache, packageName, messageName, type, filePath);
        case ACTION_TYPE:
          return new ActionSpec(msgCache, packageName, messageName, type, filePath);
        default:
          throw new Error('Unable to create message spec for type [' + type + ']');
      }
    }
  }]);

  return RosMsgSpec;
}();

/**
 * Subclass of RosMsgSpec
 * Implements logic for individual ros messages as well as separated parts of services and actions
 * (e.g. Request, Response, Goal, ActionResult, ...)
 * @class MsgSpec
 */


var MsgSpec = function (_RosMsgSpec) {
  _inherits(MsgSpec, _RosMsgSpec);

  function MsgSpec(msgCache, packageName, messageName, type) {
    var filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;
    var fileContents = arguments.length > 5 && arguments[5] !== undefined ? arguments[5] : null;

    _classCallCheck(this, MsgSpec);

    var _this = _possibleConstructorReturn(this, (MsgSpec.__proto__ || Object.getPrototypeOf(MsgSpec)).call(this, msgCache, packageName, messageName, type, filePath));

    _this.constants = [];
    _this.fields = [];

    _this.loadFile(filePath, fileContents);
    return _this;
  }

  /**
   * Parses through message definition for fields and constants
   * @param content {string} relevant portion of message definition
   * @private
   */


  _createClass(MsgSpec, [{
    key: '_parseMessage',
    value: function _parseMessage(content) {
      var lines = content.split('\n').map(function (line) {
        return line.trim();
      });

      try {
        lines.forEach(this._parseLine.bind(this));
      } catch (err) {
        console.error('Error while parsing message %s: %s', this.getFullMessageName(), err);
        throw err;
      }
    }

    /**
     * Given a line from the message file, parse it for useful contents
     * @param line {string}
     * @private
     */

  }, {
    key: '_parseLine',
    value: function _parseLine(line) {
      line = line.trim();

      var lineEqualIndex = line.indexOf('=');
      var lineCommentIndex = line.indexOf('#');

      // clear out comments if this line is not a constant
      // string constants include EVERYTHING after the equals
      if (lineEqualIndex === -1 && lineCommentIndex !== -1 || lineEqualIndex > lineCommentIndex) {
        line = line.replace(/#.*/, '');
      }

      if (line !== '') {

        var firstSpace = line.indexOf(' '),
            fieldType = line.substring(0, firstSpace).trim(),
            field = line.substring(firstSpace + 1).trim(),
            equalIndex = field.indexOf('='),
            fieldName = field.trim();

        if (equalIndex !== -1) {
          fieldName = field.substring(0, equalIndex).trim();
          if (fieldType !== 'string') {
            var commentIndex = field.indexOf('#');
            if (commentIndex !== -1) {
              field = field.substring(0, commentIndex).trim();
            }
          }
          var constant = field.substring(equalIndex + 1, field.length).trim();
          var parsedConstant = fieldsUtil.parsePrimitive(fieldType, constant);

          this.constants.push({
            name: fieldName,
            type: fieldType,
            value: parsedConstant,
            stringValue: constant // include the string value for md5 text
            , index: this.constants.length,
            messageType: null
          });
        } else {
          // ROS lets you not include the package name if it's in the same package
          // e.g. in tutorial_msgs/MyMsg
          //    ComplexType fieldName  # this is assumed to be in tutorial_msgs
          // TODO: would ROS automatically search for fields in other packages if possible??
          //       we may need to support this...
          var _parseType = parseType(fieldType),
              baseType = _parseType.baseType;
          // if it's a header and isn't explicit, be explicit


          if (isHeader(baseType) && !getPackageNameFromMessageType(baseType)) {
            fieldType = 'std_msgs/' + fieldType;
          } else if (!fieldsUtil.isPrimitive(baseType) && !getPackageNameFromMessageType(baseType)) {
            fieldType = this.packageName + '/' + fieldType;
          }
          var f = new Field(fieldName, fieldType);
          this.fields.push(f);
        }
      }
    }

    /**
     * Check if this message will have a fixed size regardless of its contents
     * @returns {boolean}
     */

  }, {
    key: 'isMessageFixedSize',
    value: function isMessageFixedSize() {
      var _this2 = this;

      // Check if a particular message specification has a constant size in bytes
      var fields = this.fields;
      var types = fields.map(function (field) {
        return field.baseType;
      });
      var variableLengthArrays = fields.map(function (field) {
        return field.isArray && field.arrayLen === null;
      });
      var isBuiltin = fields.map(function (field) {
        return field.isBuiltin;
      });
      if (types.indexOf('string') !== -1) {
        return false;
      } else if (variableLengthArrays.indexOf(true) !== -1) {
        return false;
      } else if (isBuiltin.indexOf(false) === -1) {
        return true;
      } else {
        var nonBuiltins = fields.filter(function (field) {
          return !field.isBuiltin;
        });
        return nonBuiltins.every(function (field) {
          var msgSpec = _this2.getMsgSpecForType(field.baseType);
          if (!msgSpec) {
            throw new Error('Unable to load spec for field [' + field.baseType + ']');
          }
          return msgSpec.isMessageFixedSize();
        });
      }
    }

    /**
     * Calculates the fixed size of this message if it has a fixed size
     * @returns {number|null} size if message has fixed size else null
     */

  }, {
    key: 'getMessageFixedSize',
    value: function getMessageFixedSize() {
      var _this3 = this;

      // Return the size of the message.
      // If the message does not have a fixed size, returns null
      if (!this.isMessageFixedSize()) {
        return null;
      }
      // else
      var length = 0;
      this.fields.forEach(function (field) {
        if (field.isBuiltin) {
          var typeSize = fieldsUtil.getPrimitiveSize(field.baseType);
          if (typeSize === 0) {
            throw new Error('Field ' + field.baseType + ' in message ' + _this3.getFullMessageName() + ' has a non-constant size');
          }
          if (!field.isArray) {
            length += typeSize;
          } else if (field.arrayLen === null) {
            throw new Error('Array field ' + field.baseType + ' in message ' + _this3.getFullMessageName() + ' has a variable length');
          } else {
            length += field.arrayLen * typeSize;
          }
        } else {
          var msgSpec = _this3.getMsgSpecForType(field.baseType);
          if (!msgSpec) {
            throw new Error('Unable to load spec for field [' + field.baseType + '] in message ' + _this3.getFullMessageName());
          }
          var fieldSize = msgSpec.getMessageFixedSize();
          if (fieldSize === null) {
            throw new Error('Field ' + field.baseType + ' in message ' + _this3.getFullMessageName() + ' has a non-constant size');
          }
          length += fieldSize;
        }
      });
      return length;
    }

    /**
     * Generates the text used to calculate this message's md5 sum
     * @returns {string}
     */

  }, {
    key: 'getMd5text',
    value: function getMd5text() {
      var _this4 = this;

      var text = '';
      var constants = this.constants.map(function (constant) {
        // NOTE: use the string value of the constant from when we parsed it so that JS doesn't drop decimal precision
        // e.g. message has constant "float32 A_CONSTANT=1.0"
        //  here would turn into "float32 A_CONSTANT=1" if we used its parsed value
        return constant.type + ' ' + constant.name + '=' + constant.stringValue;
      }).join('\n');

      var fields = this.fields.map(function (field) {
        if (field.isBuiltin) {
          return field.type + ' ' + field.name;
        } else {
          var spec = _this4.getMsgSpecForType(field.baseType);
          return spec.getMd5sum() + ' ' + field.name;
        }
      }).join('\n');

      text += constants;
      if (text.length > 0 && fields.length > 0) {
        text += "\n";
      }
      text += fields;
      return text;
    }

    /**
     * Generates text for message class file
     * @returns {string}
     */

  }, {
    key: 'generateMessageClassFile',
    value: function generateMessageClassFile() {
      return MessageWriter.createMessageClass(this);
    }

    /**
     * Generates a depth-first list of all dependencies of this message in field order.
     * @param [deps] {Array}
     * @returns {Array}
     */

  }, {
    key: 'getFullDependencies',
    value: function getFullDependencies() {
      var _this5 = this;

      var deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : [];

      this.fields.forEach(function (field) {
        if (!field.isBuiltin) {
          var fieldSpec = _this5.getMsgSpecForType(field.baseType);
          if (deps.indexOf(fieldSpec) === -1) {
            deps.push(fieldSpec);
          }
          fieldSpec.getFullDependencies(deps);
        }
      });

      return deps;
    }
  }]);

  return MsgSpec;
}(RosMsgSpec);

/**
 * Subclass of RosMsgSpec
 * Implements logic for ros services. Creates MsgSpecs for request and response
 * @class SrvSpec
 */


var SrvSpec = function (_RosMsgSpec2) {
  _inherits(SrvSpec, _RosMsgSpec2);

  function SrvSpec(msgCache, packageName, messageName, type) {
    var filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    _classCallCheck(this, SrvSpec);

    var _this6 = _possibleConstructorReturn(this, (SrvSpec.__proto__ || Object.getPrototypeOf(SrvSpec)).call(this, msgCache, packageName, messageName, type, filePath));

    _this6.fileContents = _this6._loadMessageFile(filePath);

    var _this6$_extractMessag = _this6._extractMessageSections(_this6.fileContents),
        req = _this6$_extractMessag.req,
        resp = _this6$_extractMessag.resp;

    _this6.request = new MsgSpec(msgCache, packageName, messageName + 'Request', SRV_REQUEST_TYPE, null, req);
    _this6.response = new MsgSpec(msgCache, packageName, messageName + 'Response', SRV_RESPONSE_TYPE, null, resp);
    return _this6;
  }

  /**
   * Takes a full service definition and pulls out the request and response sections
   * @param fileContents {string}
   * @returns {object}
   * @private
   */


  _createClass(SrvSpec, [{
    key: '_extractMessageSections',
    value: function _extractMessageSections(fileContents) {
      var lines = fileContents.split('\n').map(function (line) {
        return line.trim();
      });

      var sections = {
        req: '',
        resp: ''
      };

      var currentSection = 'req';

      lines.forEach(function (line) {
        if (line.startsWith(MSG_DIVIDER)) {
          currentSection = 'resp';
        } else {
          sections[currentSection] += '\n' + line;
        }
      });

      return sections;
    }
  }, {
    key: 'getMd5text',
    value: function getMd5text() {
      return this.request.getMd5text() + this.response.getMd5text();
    }
  }, {
    key: 'getMessageDependencies',
    value: function getMessageDependencies() {
      var deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();

      this.request.getMessageDependencies(deps);
      this.response.getMessageDependencies(deps);
      return deps;
    }

    /**
     * Generates text for service class file
     * @returns {string}
     */

  }, {
    key: 'generateMessageClassFile',
    value: function generateMessageClassFile() {
      return MessageWriter.createServiceClass(this);
    }
  }]);

  return SrvSpec;
}(RosMsgSpec);

/**
 * Subclass of RosMsgSpec
 * Implements logic for ROS actions which generate 7 messages from their definition.
 * Creates MsgSpecs for goal, result, feedback, action goal, action result, action feedback, and action
 * @class ActionSpec
 */


var ActionSpec = function (_RosMsgSpec3) {
  _inherits(ActionSpec, _RosMsgSpec3);

  function ActionSpec(msgCache, packageName, messageName, type) {
    var filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    _classCallCheck(this, ActionSpec);

    var _this7 = _possibleConstructorReturn(this, (ActionSpec.__proto__ || Object.getPrototypeOf(ActionSpec)).call(this, msgCache, packageName, messageName, type, filePath));

    _this7.fileContents = _this7._loadMessageFile(filePath);

    var _this7$_extractMessag = _this7._extractMessageSections(_this7.fileContents),
        goal = _this7$_extractMessag.goal,
        result = _this7$_extractMessag.result,
        feedback = _this7$_extractMessag.feedback;

    // Parse the action definition into its 3 respective parts


    _this7.goal = new MsgSpec(msgCache, packageName, messageName + 'Goal', ACTION_GOAL_TYPE, null, goal);
    _this7.result = new MsgSpec(msgCache, packageName, messageName + 'Result', ACTION_RESULT_TYPE, null, result);
    _this7.feedback = new MsgSpec(msgCache, packageName, messageName + 'Feedback', ACTION_FEEDBACK_TYPE, null, feedback);
    _this7.generateActionMessages();
    return _this7;
  }

  /**
   * Takes a full service definition and pulls out the request and response sections
   * @param fileContents {string}
   * @returns {object}
   * @private
   */


  _createClass(ActionSpec, [{
    key: '_extractMessageSections',
    value: function _extractMessageSections(fileContents) {
      var lines = fileContents.split('\n').map(function (line) {
        return line.trim();
      });

      var sections = {
        goal: '',
        result: '',
        feedback: ''
      };

      var currentSection = 'goal';

      lines.forEach(function (line) {
        if (line.startsWith(MSG_DIVIDER)) {
          currentSection = {
            goal: 'result',
            result: 'feedback'
          }[currentSection];
        } else {
          sections[currentSection] += '\n' + line;
        }
      });

      return sections;
    }

    /**
     * Get a list of all the message specs created by this ros action
     * @returns {MsgSpec[]}
     */

  }, {
    key: 'getMessages',
    value: function getMessages() {
      return [this.goal, this.result, this.feedback, this.actionGoal, this.actionResult, this.actionFeedback, this.action];
    }

    /**
     * Creates the remaining 4 action messages
     */

  }, {
    key: 'generateActionMessages',
    value: function generateActionMessages() {
      this.generateActionGoalMessage();
      this.generateActionResultMessage();
      this.generateActionFeedbackMessage();
      this.generateActionMessage();
    }
  }, {
    key: 'generateActionGoalMessage',
    value: function generateActionGoalMessage() {
      var goalMessage = MessageWriter.generateActionGoalMessage(this.getFullMessageName());

      this.actionGoal = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionGoal', ACTION_ACTION_GOAL_TYPE, null, goalMessage);
    }
  }, {
    key: 'generateActionResultMessage',
    value: function generateActionResultMessage() {
      var resultMessage = MessageWriter.generateActionResultMessage(this.getFullMessageName());

      this.actionResult = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionResult', ACTION_ACTION_RESULT_TYPE, null, resultMessage);
    }
  }, {
    key: 'generateActionFeedbackMessage',
    value: function generateActionFeedbackMessage() {
      var feedbackMessage = MessageWriter.generateActionFeedbackMessage(this.getFullMessageName());

      this.actionFeedback = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionFeedback', ACTION_ACTION_FEEDBACK_TYPE, null, feedbackMessage);
    }
  }, {
    key: 'generateActionMessage',
    value: function generateActionMessage() {
      var actionMessage = MessageWriter.generateActionMessage(this.getFullMessageName());

      this.action = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'Action', ACTION_ACTION_TYPE, null, actionMessage);
    }
  }, {
    key: 'getMessageDependencies',
    value: function getMessageDependencies() {
      var deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();

      this.goal.getMessageDependencies(deps);
      this.result.getMessageDependencies(deps);
      this.feedback.getMessageDependencies(deps);
      this.actionGoal.getMessageDependencies(deps);
      this.actionResult.getMessageDependencies(deps);
      this.actionFeedback.getMessageDependencies(deps);
      this.action.getMessageDependencies(deps);
      return deps;
    }
  }]);

  return ActionSpec;
}(RosMsgSpec);

RosMsgSpec.MSG_TYPE = MSG_TYPE;
RosMsgSpec.SRV_TYPE = SRV_TYPE;
RosMsgSpec.SRV_REQUEST_TYPE = SRV_REQUEST_TYPE;
RosMsgSpec.SRV_RESPONSE_TYPE = SRV_RESPONSE_TYPE;
RosMsgSpec.ACTION_TYPE = ACTION_TYPE;
RosMsgSpec.ACTION_GOAL_TYPE = ACTION_GOAL_TYPE;
RosMsgSpec.ACTION_FEEDBACK_TYPE = ACTION_FEEDBACK_TYPE;
RosMsgSpec.ACTION_RESULT_TYPE = ACTION_RESULT_TYPE;
RosMsgSpec.ACTION_ACTION_GOAL_TYPE = ACTION_ACTION_GOAL_TYPE;
RosMsgSpec.ACTION_ACTION_FEEDBACK_TYPE = ACTION_ACTION_FEEDBACK_TYPE;
RosMsgSpec.ACTION_ACTION_RESULT_TYPE = ACTION_ACTION_RESULT_TYPE;
RosMsgSpec.ACTION_ACTION_TYPE = ACTION_ACTION_TYPE;

module.exports = RosMsgSpec;