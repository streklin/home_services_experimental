'use strict';

var rosnodejs = require('../index.js');

rosnodejs.initNode('/my_node', { onTheFly: true }).then(function (rosNode) {

  var std_msgs = rosnodejs.require('std_msgs').msg;
  var msg = new std_msgs.String();

  var SetBool = rosnodejs.require('std_srvs').srv.SetBool;
  var request = new SetBool.Request();

  // EXP 1) Service Server
  var service = rosNode.advertiseService('/set_bool', 'std_srvs/SetBool', function (req, resp) {
    console.log('Handling request! ' + JSON.stringify(req));
    resp.success = !req.data;
    resp.message = 'Inverted!';
    return true;
  });

  // EXP 2) Service Client
  setTimeout(function () {
    var serviceClient = rosNode.serviceClient('/set_bool', 'std_srvs/SetBool');
    rosNode.waitForService(serviceClient.getService(), 2000).then(function (available) {
      if (available) {
        var _request = new SetBool.Request();
        _request.data = true;
        serviceClient.call(_request).then(function (resp) {
          console.log('Service response ' + JSON.stringify(resp));
        });
      } else {
        console.log('Service not available');
      }
    });
  }, 1000); // wait a second before calling our service

  // EXP 3) Params
  rosNode.setParam('~junk', { 'hi': 2 }).then(function () {
    rosNode.getParam('~junk').then(function (val) {
      console.log('Got Param!!! ' + JSON.stringify(val));
    });
  });

  // // EXP 4) Publisher
  var pub = rosNode.advertise('/my_topic', 'std_msgs/String', {
    queueSize: 1,
    latching: true,
    throttleMs: 9
  });

  // EXP 5) Subscriber
  var sub = rosNode.subscribe('/my_topic', 'std_msgs/String', function (data) {
    console.log('SUB DATA ', data, data.data);
  }, { queueSize: 1,
    throttleMs: 1000 });

  var msgStart = 'my message ';
  var iter = 0;
  setInterval(function () {
    msg.data = msgStart + iter;
    pub.publish(msg);
    ++iter;
    if (iter > 200) {
      iter = 0;
    }
  }, 5);
});