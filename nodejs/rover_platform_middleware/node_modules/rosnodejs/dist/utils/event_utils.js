"use strict";

module.exports = {
  rebroadcast: function rebroadcast(evt, emitter, rebroadcaster) {
    emitter.on(evt, rebroadcaster.emit.bind(rebroadcaster, evt));
  }
};