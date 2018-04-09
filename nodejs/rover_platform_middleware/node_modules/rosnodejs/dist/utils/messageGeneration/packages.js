'use strict';

var fs = require('fs'),
    path = require('path'),
    walker = require('walker'),
    async = require('async');

var packageCache = {};
var cache = {};

function packageWalk(directory, symlinks) {
  var noSubDirs = new Set();
  var stopped = false;
  symlinks = symlinks || [];

  return walker(directory).filterDir(function (dir, stat) {
    // return true to explore this directory...

    // Exclude any subdirectory to an excluded directory
    var ignoreFile = path.join(dir, 'CATKIN_IGNORE');

    function fileExists(file) {
      try {
        fs.statSync(file);
      } catch (err) {
        return false;
      }
      return true;
    }

    // if CATKIN_IGNORE exists, just don't even look
    if (fileExists(ignoreFile)) {
      return false;
    }

    // Check if this is a package - if it is, we need to add it to noSubDirs now
    // Otherwise it could start exploring subdirs before discovering its a package.
    var packageFile = path.join(dir, 'package.xml');
    var manifestFile = path.join(dir, 'manifest.xml');

    if (fileExists(packageFile) || fileExists(manifestFile)) {
      noSubDirs.add(dir);
    }

    // don't explore the directory if this dir's parent is
    // in noSubDirs or we have stopped
    var parent = path.dirname(dir);
    return !(noSubDirs.has(parent) || stopped);
  }).on('file', function (file, stat) {
    var shortname = path.basename(file);
    var dir = path.dirname(file);

    if (shortname === 'manifest.xml' || shortname === 'package.xml') {
      this.emit('package', path.basename(dir), dir, file);
    } else if (shortname === 'rospack_nosubdirs') {
      // Explicitly asked to not go into subdirectories
      noSubDirs.add(dir);
    }
  }).on('symlink', function (symlink, stat) {
    var walker = this;
    fs.readlink(symlink, function (error, link) {
      if (error) {
        return;
      }

      var destination = path.resolve(path.dirname(symlink), link);

      // Stores symlinks to avoid circular references
      if (~symlinks.indexOf(destination)) {
        return;
      } else {
        symlinks.concat(destination);
      }

      fs.stat(destination, function (error, stat) {
        if (error) {
          return;
        } else if (stat.isDirectory()) {
          walker.emit('dir', destination, stat);
          return walker.go(destination);
        }
      });
    });
  }).on('error', function (err) {
    console.error('Error while walking directory %s: %s', directory, err.message);
  }).on('end', function () {
    stopped = true;
    // Quit emitting
    this.emit = function () {};
  });
}

function messageWalk(directory, symlinks) {
  var stopped = false;
  symlinks = symlinks || [];

  return walker(directory).filterDir(function (dir, stat) {
    // Exclude any subdirectory to an excluded directory
    var ignoreFile = path.join(dir, 'CATKIN_IGNORE');

    function fileExists(file) {
      try {
        fs.statSync(file);
      } catch (err) {
        return false;
      }
      return true;
    }

    // if CATKIN_IGNORE exists, just don't even look
    if (fileExists(ignoreFile)) {
      return false;
    }
    // else
    return true;
  }).on('file', function (file) {
    var extension = path.extname(file);
    var dir = path.dirname(file);
    var name = path.basename(file, extension);

    if (extension === '.msg') {
      this.emit('message', name, file);
    } else if (extension === '.srv') {
      this.emit('service', name, file);
    } else if (extension === '.action') {
      this.emit('action', name, file);
    }
  }).on('symlink', function (symlink, stat) {
    var walker = this;
    fs.readlink(symlink, function (error, link) {
      if (error) {
        return;
      }

      var destination = path.resolve(path.dirname(symlink), link);

      // Stores symlinks to avoid circular references
      if (~symlinks.indexOf(destination)) {
        return;
      } else {
        symlinks.concat(destination);
      }

      fs.stat(destination, function (error, stat) {
        if (error) {
          return;
        } else if (stat.isDirectory()) {
          walker.emit('dir', destination, stat);
          return walker.go(destination);
        }
      });
    });
  }).on('error', function (err) {
    console.error('Error while walking directory %s: %s', directory, err.message);
  }).on('end', function () {
    stopped = true;
    // Quit emitting
    this.emit = function () {};
  });
}

function findPackageInDirectory(directory, packageName, callback) {
  var found = false;
  return packageWalk(directory).on('package', function (name, dir) {
    if (name === packageName) {
      this.emit('stop');
      found = true;
      callback(null, dir);
    }
  }).on('end', function () {
    if (!found) {
      var error = new Error('ENOTFOUND - Package ' + packageName + ' not found');
      error.name = 'PackageNotFoundError';
      callback(error);
    }
  });
}

function findPackagesInDirectory(directory) {
  var promises = [];
  promises.push(new Promise(function (resolve) {
    packageWalk(directory).on('package', function (packageName, dir, fileName) {
      packageName = packageName.toLowerCase();
      if (!packageCache.hasOwnProperty(packageName)) {
        var packageEntry = {
          directory: dir,
          messages: {},
          services: {},
          actions: {}
        };
        promises.push(new Promise(function (resolve) {
          messageWalk(dir, null).on('message', function (name, file) {
            packageEntry.messages[name] = { file: file };
          }).on('service', function (name, file) {
            packageEntry.services[name] = { file: file };
          }).on('action', function (name, file) {
            packageEntry.actions[name] = { file: file };
          }).on('end', function () {
            if (Object.keys(packageEntry.messages).length > 0 || Object.keys(packageEntry.services).length > 0 || Object.keys(packageEntry.actions).length > 0) {
              packageCache[packageName] = packageEntry;
            }
            resolve();
          });
        }));
      }
    }).on('end', resolve);
  }));

  return Promise.all(promises);
}

function findPackageInDirectoryChain(directories, packageName, callback) {
  if (directories.length < 1) {
    var error = new Error('ENOTFOUND - Package ' + packageName + ' not found');
    error.name = 'PackageNotFoundError';
    callback(error);
  } else {
    findPackageInDirectory(directories.shift(), packageName, function (error, directory) {
      if (error) {
        if (error.name === 'PackageNotFoundError') {
          // Recursive call, try in next directory
          return findPackageInDirectoryChain(directories, packageName, callback);
        } else {
          callback(error);
        }
      } else {
        callback(null, directory);
      }
    });
  }
}

function findPackagesInDirectoryChain(directories) {
  var funcs = directories.map(function (directory) {
    return findPackagesInDirectory.bind(null, directory);
  });
  return funcs.reduce(function (prev, cur, index) {
    return prev.then(function () {
      return cur();
    });
  }, Promise.resolve());
}

// ---------------------------------------------------------

function getRosEnvVar(envVarName) {
  var envVar = process.env[envVarName];

  if (!envVar) {
    throw new Error('Unable to find required environment variable ' + envVarName);
  }

  return envVar;
}

function getRosPackagePath() {
  return getRosEnvVar('ROS_PACKAGE_PATH');
}

function getRosRoot() {
  return getRosEnvVar('ROS_ROOT');
}

// Implements the same crawling algorithm as rospack find
// See http://ros.org/doc/api/rospkg/html/rospack.html
// packages = {};
exports.findPackage = function (packageName, callback) {
  var directory = cache[packageName.toLowerCase()];
  if (directory) {
    callback(null, directory);
    return;
  }

  var packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  var directories = rosPackagePaths;
  return findPackageInDirectoryChain(directories, packageName, function (err, directory) {
    cache[packageName.toLowerCase()] = directory;
    callback(err, directory);
  });
};

exports.findMessagePackages = function () {
  var packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  return findPackagesInDirectoryChain(rosPackagePaths);
};

exports.getPackageCache = function () {
  return Object.assign({}, packageCache);
};

function forEachPackageInDirectory(directory, list, onEnd) {
  fs.access(directory, fs.R_OK, function (err) {
    if (!err) {
      packageWalk(directory).on('package', function (name, dir) {
        list.push(dir);
      }).on('end', onEnd);
    } else {
      onEnd();
    }
  });
}

/** get list of package directories */
exports.getAllPackages = function (done) {
  var rosRoot = getRosRoot();
  var packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  var directories = [rosRoot].concat(rosPackagePaths);
  async.reduce(directories, [], function (memo, directory, callback) {
    forEachPackageInDirectory(directory, memo, function () {
      callback(null, memo);
    });
  }, function (err, directories) {
    directories.forEach(function (directory) {
      var packageName = path.basename(directory);
      cache[packageName.toLowerCase()] = directory;
    });
    done(err, directories);
  });
};