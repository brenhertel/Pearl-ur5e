
"use strict";

let shutdown_request = require('./shutdown_request.js')
let time_shutdown_request = require('./time_shutdown_request.js')

module.exports = {
  shutdown_request: shutdown_request,
  time_shutdown_request: time_shutdown_request,
};
