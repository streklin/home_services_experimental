'use strict';

var path = require('path');
var rosnodejs = require('../index.js');
var ArgumentParser = require('argparse').ArgumentParser;

var parser = new ArgumentParser({
  addHelp: true,
  description: 'Utility script to generate ROS messages'
});

parser.addArgument(['-p', '--pkg'], {
  help: 'Message package to build (e.g. std_msgs). Also builds dependencies'
});
parser.addArgument(['-o', '--output'], {
  help: 'Directory to output message into (e.g. /tmp). Messages are built to devel space by default'
});
parser.addArgument(['-v', '--verbose'], {
  action: 'storeTrue'
});

var args = parser.parseArgs();

if (args.output) {
  args.output = path.resolve(args.output);
}

if (args.pkg !== null) {
  rosnodejs.loadPackage(args.pkg, args.output, args.verbose);
} else {
  rosnodejs.loadAllPackages(args.output, args.verbose).then(function () {
    console.log('Message generation complete!');
    process.exit();
  }).catch(function (err) {
    console.error('Error while generating messages!');
    process.exit(1);
  });
}