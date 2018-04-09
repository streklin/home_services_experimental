'use strict';

var rosnodejs = require('../index.js');
var std_msgs = rosnodejs.require('std_msgs').msg;
var SetBool = rosnodejs.require('std_srvs').srv.SetBool;

rosnodejs.initNode('/test_node').then(function (rosNode) {
  // EXP 1) Service Server
  var service = rosNode.advertiseService('/set_bool', SetBool, function (req, resp) {
    rosnodejs.log.info('Handling request! ' + JSON.stringify(req));
    resp.success = !req.data;
    resp.message = 'Inverted!';
    return true;
  });

  // EXP 2) Service Client
  var serviceClient = rosNode.serviceClient('/set_bool', 'std_srvs/SetBool', { persist: true });
  rosNode.waitForService(serviceClient.getService(), 2000).then(function (available) {
    if (available) {
      var request = new SetBool.Request();
      request.data = true;
      serviceClient.call(request).then(function (resp) {
        rosnodejs.log.info('Service response ' + JSON.stringify(resp));
      }).then(function () {
        request.data = false;
        serviceClient.call(request).then(function (resp) {
          rosnodejs.log.info('Service response 2 ' + JSON.stringify(resp));
        });
      }).then(function () {
        var serviceClient2 = rosNode.serviceClient('/set_bool', 'std_srvs/SetBool');
        serviceClient2.call(request).then(function (resp) {
          rosnodejs.log.info('Non persistent response ' + JSON.stringify(resp));
        });
      });
    }
  });

  // EXP 3) Params
  rosNode.setParam('~junk', { 'hi': 2 }).then(function () {
    rosNode.getParam('~junk').then(function (val) {
      rosnodejs.log.info('Got Param!!! ' + JSON.stringify(val));
    });
  });

  // EXP 4) Publisher
  var pub = rosNode.advertise('/my_topic', std_msgs.String, {
    queueSize: 1,
    latching: true,
    throttleMs: 9
  });

  var msgStart = 'my message ';
  var iter = 0;
  var msg = new std_msgs.String();
  setInterval(function () {
    msg.data = msgStart + iter;
    pub.publish(msg);
    ++iter;
    if (iter > 200) {
      iter = 0;
    }
  }, 5);

  // EXP 5) Subscriber
  var sub = rosNode.subscribe('/my_topic', 'std_msgs/String', function (data) {
    rosnodejs.log.info('SUB DATA ' + data.data);
  }, {
    queueSize: 1,
    throttleMs: 1000
  });
}).catch(function (err) {
  rosnodejs.log.error(err.stack);
});