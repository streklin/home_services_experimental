'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var fs = require('fs');
var path = require('path');
var util = require('util');
var md5 = require('md5');
var async = require('async');

var packages = require('./packages');
var fieldsUtil = require('./fields');
var IndentedWriter = require('./IndentedWriter.js');
var MsgSpec = require('./MessageSpec.js');

var Field = fieldsUtil.Field;

var packageCache = null;

var PKG_LOADING = 'loading';
var PKG_LOADED = 'loaded';

function createDirectory(directory) {
  var curPath = '/';
  var paths = directory.split(path.sep);

  function createLocal(dirPath) {
    return new Promise(function (resolve, reject) {
      fs.mkdir(dirPath, function (err) {
        if (err && err.code !== 'EEXIST') {
          reject(err);
        }
        resolve();
      });
    });
  }

  return paths.reduce(function (prev, cur, index, array) {
    curPath = path.join(curPath, array[index]);
    return prev.then(createLocal.bind(null, curPath));
  }, Promise.resolve());
}

function writeFile(filepath, data) {
  return new Promise(function (resolve, reject) {
    fs.writeFile(filepath, data, function (err) {
      if (err) {
        reject(err);
      } else {
        resolve();
      }
    });
  });
}

var MessageManager = function () {
  function MessageManager() {
    var verbose = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : false;

    _classCallCheck(this, MessageManager);

    this._packageChain = [];
    this._loadingPkgs = new Map();

    this._verbose = verbose;
  }

  _createClass(MessageManager, [{
    key: 'log',
    value: function log() {
      if (this._verbose) {
        var _console;

        (_console = console).log.apply(_console, arguments);
      }
    }
  }, {
    key: 'getCache',
    value: function getCache() {
      return packageCache;
    }
  }, {
    key: 'getMessageSpec',
    value: function getMessageSpec(msgType) {
      var type = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : MsgSpec.MSG_TYPE;

      var messageName = fieldsUtil.getMessageNameFromMessageType(msgType);
      var pkg = fieldsUtil.getPackageNameFromMessageType(msgType);
      if (packageCache.hasOwnProperty(pkg)) {
        var pkgCache = void 0;
        switch (type) {
          case MsgSpec.MSG_TYPE:
            pkgCache = packageCache[pkg].messages;
            break;
          case MsgSpec.SRV_TYPE:
            pkgCache = packageCache[pkg].services;
            break;
        }
        if (pkgCache) {
          // be case insensitive here...
          if (pkgCache.hasOwnProperty(messageName)) {
            return pkgCache[messageName].msgSpec;
          }
          var lcName = messageName.toLowerCase();
          if (pkgCache.hasOwnProperty(lcName)) {
            return pkgCache[lcName].msgSpec;
          }
        }
      }
      // fall through
      return null;
    }
  }, {
    key: 'buildPackageTree',
    value: function buildPackageTree(outputDirectory) {
      var _this = this;

      var writeFiles = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : true;

      return this.initTree().then(function () {
        _this._packageChain = _this._buildMessageDependencyChain();

        // none of the loading here depends on message dependencies
        // so don't worry about doing it in order, just do it all...
        return Promise.all(_this._packageChain.map(function (pkgName) {
          return _this.loadPackage(pkgName, outputDirectory, false, writeFiles);
        }));
      }).catch(function (err) {
        console.error(err.stack);
        throw err;
      });
    }
  }, {
    key: 'buildPackage',
    value: function buildPackage(packageName, outputDirectory) {
      var _this2 = this;

      var deps = new Set();
      return this.initTree().then(function () {
        _this2.loadPackage(packageName, outputDirectory, true, true, function (depName) {
          if (!deps.has(depName)) {
            deps.add(depName);
            return true;
          }
          return false;
        });
      });
    }
  }, {
    key: 'initTree',
    value: function initTree() {
      var _this3 = this;

      var p = void 0;
      if (packageCache === null) {
        this.log('Traversing ROS_PACKAGE_PATH...');
        p = packages.findMessagePackages();
      } else {
        p = Promise.resolve();
      }
      return p.then(function () {
        packageCache = packages.getPackageCache();

        // load all the messages
        // TODO: only load messages we need
        _this3._loadMessagesInCache();
      });
    }
  }, {
    key: 'loadPackage',
    value: function loadPackage(packageName, outputDirectory) {
      var loadDeps = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : true;

      var _this4 = this;

      var writeFiles = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : true;
      var filterDepFunc = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

      if (this._loadingPkgs.has(packageName)) {
        return Promise.resolve();
      }
      // else
      this.log('Loading package %s', packageName);
      this._loadingPkgs.set(packageName, PKG_LOADING);

      if (loadDeps) {
        // get an ordered list of dependencies for this message package
        var dependencies = this._buildMessageDependencyChain(this._getFullDependencyChain(packageName));

        // filter out any packages that have already been loaded or are loading
        var depsToLoad = dependencies;
        if (filterDepFunc && typeof filterDepFunc === 'function') {
          depsToLoad = dependencies.filter(filterDepFunc);
        }

        depsToLoad.forEach(function (depName) {
          _this4.loadPackage(depName, outputDirectory, loadDeps, filterDepFunc);
        });
      }

      // actions get parsed and are then cached with the rest of the messages
      // which is why there isn't a loadPackageActions
      if (writeFiles) {
        return this.initPackageWrite(packageName, outputDirectory).then(this.writePackageMessages.bind(this, packageName, outputDirectory)).then(this.writePackageServices.bind(this, packageName, outputDirectory)).then(function () {
          _this4._loadingPkgs.set(packageName, PKG_LOADED);
          console.log('Finished building package %s', packageName);
        });
      }
      // else
      return Promise.resolve();
    }
  }, {
    key: 'initPackageWrite',
    value: function initPackageWrite(packageName, jsMsgDir) {
      var _this5 = this;

      var packageDir = path.join(jsMsgDir, packageName);
      packageCache[packageName].directory = packageDir;

      return createDirectory(packageDir).then(function () {
        if (_this5.packageHasMessages(packageName) || _this5.packageHasActions(packageName)) {
          var msgDir = path.join(packageDir, 'msg');
          return createDirectory(msgDir).then(_this5.createMessageIndex.bind(_this5, packageName, msgDir));
        }
      }).then(function () {
        if (_this5.packageHasServices(packageName)) {
          var srvDir = path.join(packageDir, 'srv');
          return createDirectory(srvDir).then(_this5.createServiceIndex.bind(_this5, packageName, srvDir));
        }
      }).then(this.createPackageIndex.bind(this, packageName, packageDir));
    }
  }, {
    key: 'createPackageIndex',
    value: function createPackageIndex(packageName, directory) {
      var w = new IndentedWriter();
      w.write('module.exports = {').indent();

      var hasMessages = this.packageHasMessages(packageName) || this.packageHasActions(packageName);
      var hasServices = this.packageHasServices(packageName);
      if (hasMessages) {
        w.write('msg: require(\'./msg/_index.js\'),');
      }
      if (hasServices) {
        w.write('srv: require(\'./srv/_index.js\')');
      }
      w.dedent().write('};');

      return writeFile(path.join(directory, '_index.js'), w.get());
    }
  }, {
    key: 'createIndex',
    value: function createIndex(packageName, directory, msgKey) {
      var messages = Object.keys(packageCache[packageName][msgKey]);
      var w = new IndentedWriter();
      w.write('module.exports = {').indent();

      messages.forEach(function (message) {
        w.write('%s: require(\'./%s.js\'),', message, message);
      });

      w.dedent().write('};');

      return writeFile(path.join(directory, '_index.js'), w.get());
    }
  }, {
    key: 'createMessageIndex',
    value: function createMessageIndex(packageName, directory) {
      return this.createIndex(packageName, directory, 'messages');
    }
  }, {
    key: 'createServiceIndex',
    value: function createServiceIndex(packageName, directory) {
      return this.createIndex(packageName, directory, 'services');
    }
  }, {
    key: 'packageHasMessages',
    value: function packageHasMessages(packageName) {
      return Object.keys(packageCache[packageName].messages).length > 0;
    }
  }, {
    key: 'packageHasServices',
    value: function packageHasServices(packageName) {
      return Object.keys(packageCache[packageName].services).length > 0;
    }
  }, {
    key: 'packageHasActions',
    value: function packageHasActions(packageName) {
      return Object.keys(packageCache[packageName].actions).length > 0;
    }
  }, {
    key: 'writePackageMessages',
    value: function writePackageMessages(packageName, jsMsgDir) {
      var _this6 = this;

      var msgDir = path.join(jsMsgDir, packageName, 'msg');

      var packageMsgs = packageCache[packageName].messages;
      var numMsgs = Object.keys(packageMsgs).length;
      if (numMsgs > 0) {
        this.log('Building %d messages from %s', numMsgs, packageName);
        var promises = [];
        Object.keys(packageMsgs).forEach(function (msgName) {
          var spec = packageMsgs[msgName].msgSpec;
          _this6.log('Building message ' + spec.packageName + '/' + spec.messageName);
          promises.push(writeFile(path.join(msgDir, msgName + '.js'), spec.generateMessageClassFile()));
        });

        return Promise.all(promises);
      }
      // else
      return Promise.resolve();
    }
  }, {
    key: 'writePackageServices',
    value: function writePackageServices(packageName, jsMsgDir) {
      var _this7 = this;

      var msgDir = path.join(jsMsgDir, packageName, 'srv');

      var packageSrvs = packageCache[packageName].services;
      var numSrvs = Object.keys(packageSrvs).length;
      if (numSrvs > 0) {
        this.log('Building %d services from %s', numSrvs, packageName);
        var promises = [];
        Object.keys(packageSrvs).forEach(function (srvName) {
          var spec = packageSrvs[srvName].msgSpec;
          _this7.log('Building service ' + spec.packageName + '/' + spec.messageName);
          promises.push(writeFile(path.join(msgDir, srvName + '.js'), spec.generateMessageClassFile()));
        });

        return Promise.all(promises);
      }
      // else
      return Promise.resolve();
    }
  }, {
    key: '_loadMessagesInCache',
    value: function _loadMessagesInCache() {
      var _this8 = this;

      this.log('Loading messages...');
      Object.keys(packageCache).forEach(function (packageName) {

        var packageInfo = packageCache[packageName];
        var packageDeps = new Set();

        packageInfo.forEach = function (item, func) {
          var itemInfo = packageInfo[item];
          Object.keys(itemInfo).forEach(function (item) {
            var ret = func(item, itemInfo[item]);
            if (ret) {
              itemInfo[item][ret.key] = ret.val;
            }
          });
        };

        packageInfo.forEach('messages', function (message, _ref) {
          var file = _ref.file;

          _this8.log('Loading message %s from %s', message, file);
          var msgSpec = MsgSpec.create(_this8, packageName, message, MsgSpec.MSG_TYPE, file);

          msgSpec.getMessageDependencies(packageDeps);

          return {
            key: 'msgSpec',
            val: msgSpec
          };
        });

        packageInfo.forEach('services', function (message, _ref2) {
          var file = _ref2.file;

          _this8.log('Loading service %s from %s', message, file);
          var msgSpec = MsgSpec.create(_this8, packageName, message, MsgSpec.SRV_TYPE, file);

          msgSpec.getMessageDependencies(packageDeps);

          return {
            key: 'msgSpec',
            val: msgSpec
          };
        });

        packageInfo.forEach('actions', function (message, _ref3) {
          var file = _ref3.file;

          _this8.log('Loading action %s from %s', message, file);
          var msgSpec = MsgSpec.create(_this8, packageName, message, MsgSpec.ACTION_TYPE, file);

          // cache the individual messages for later lookup (needed when writing files)
          var packageMsgs = packageInfo.messages;
          msgSpec.getMessages().forEach(function (spec) {
            // only write this action if it doesn't exist yet - this should be expected if people
            // have already run catkin_make, as it will generate action message definitions that
            // will just get loaded as regular messages
            if (!packageMsgs.hasOwnProperty(spec.messageName)) {
              packageMsgs[spec.messageName] = { file: null, msgSpec: spec };
            }
          });

          msgSpec.getMessageDependencies(packageDeps);

          return {
            key: 'msgSpec',
            val: msgSpec
          };
        });

        packageInfo.dependencies = packageDeps;
      });
    }
  }, {
    key: '_getFullDependencyChain',
    value: function _getFullDependencyChain(msgPackage) {
      var _this9 = this;

      var originalPackage = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;
      var dependencyList = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : null;

      if (dependencyList === null) {
        dependencyList = packageCache[msgPackage].dependencies;
      }
      if (originalPackage === null) {
        originalPackage = msgPackage;
      }
      var localDeps = packageCache[msgPackage].dependencies;
      localDeps.forEach(function (dep) {
        if (dep === originalPackage) {
          throw new Error('Found circular dependency while building chain');
        }
        dependencyList.add(dep);
        _this9._getFullDependencyChain(dep, originalPackage, dependencyList);
      });

      return dependencyList;
    }
  }, {
    key: '_recurseDependencyChain',
    value: function _recurseDependencyChain(dependencyChain, packageName) {
      var _this10 = this;

      var packageDeps = packageCache[packageName].dependencies;
      var maxInsertionIndex = -1;
      packageDeps.forEach(function (depName) {
        var depIndex = dependencyChain.indexOf(depName);
        if (depIndex === -1) {
          // this dependency is not yet in the list anywhere
          var insertionIndex = _this10._recurseDependencyChain(dependencyChain, depName);
          if (insertionIndex > maxInsertionIndex) {
            maxInsertionIndex = insertionIndex;
          }
        } else {
          maxInsertionIndex = depIndex;
        }
      });

      if (maxInsertionIndex < 0) {
        dependencyChain.unshift(packageName);
        return 0;
      } else {
        dependencyChain.splice(maxInsertionIndex + 1, 0, packageName);
        return maxInsertionIndex + 1;
      }
    }
  }, {
    key: '_buildMessageDependencyChain',
    value: function _buildMessageDependencyChain() {
      var packageList = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;

      if (packageList === null) {
        packageList = Object.keys(packageCache);
      }
      var dependencyChain = [];
      packageList.forEach(this._recurseDependencyChain.bind(this, dependencyChain));
      return dependencyChain;
    }
  }]);

  return MessageManager;
}();

module.exports = MessageManager;