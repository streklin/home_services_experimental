'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var util = require('util');

var IndentedWriter = function () {
  function IndentedWriter() {
    _classCallCheck(this, IndentedWriter);

    this._str = '';
    this._indentation = 0;
  }

  _createClass(IndentedWriter, [{
    key: 'write',
    value: function write(args) {
      var formattedStr = util.format.apply(this, arguments);
      if (this.isIndented()) {
        for (var i = 0; i < this._indentation; ++i) {
          this._str += '  ';
        }
      }
      this._str += formattedStr;
      return this.newline();
    }
  }, {
    key: 'newline',
    value: function newline() {
      var indentDir = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : undefined;

      this._str += '\n';
      if (indentDir === undefined) {
        return this;
      } else if (indentDir > 0) {
        return this.indent();
      } else if (indentDir < 0) {
        return this.dedent();
      }
      // else
      return this;
    }
  }, {
    key: 'indent',
    value: function indent() {
      ++this._indentation;
      if (arguments.length > 0) {
        return this.write.apply(this, arguments);
      }
      // else
      return this;
    }
  }, {
    key: 'isIndented',
    value: function isIndented() {
      return this._indentation > 0;
    }
  }, {
    key: 'dedent',
    value: function dedent() {
      --this._indentation;
      if (this._indentation < 0) {
        this.resetIndent();
      }
      if (arguments.length > 0) {
        return this.write.apply(this, arguments);
      }
      // else
      return this;
    }
  }, {
    key: 'resetIndent',
    value: function resetIndent() {
      this._indentation = 0;
      return this;
    }
  }, {
    key: 'dividingLine',
    value: function dividingLine() {
      return this.write('//-----------------------------------------------------------');
    }
  }, {
    key: 'get',
    value: function get() {
      return this._str;
    }
  }]);

  return IndentedWriter;
}();

module.exports = IndentedWriter;