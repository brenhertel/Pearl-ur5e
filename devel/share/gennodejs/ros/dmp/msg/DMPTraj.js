// Auto-generated. Do not edit!

// (in-package dmp.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DMPPoint = require('./DMPPoint.js');

//-----------------------------------------------------------

class DMPTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.points = null;
      this.times = null;
    }
    else {
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
      if (initObj.hasOwnProperty('times')) {
        this.times = initObj.times
      }
      else {
        this.times = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DMPTraj
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = DMPPoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [times]
    bufferOffset = _arraySerializer.float64(obj.times, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DMPTraj
    let len;
    let data = new DMPTraj(null);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = DMPPoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [times]
    data.times = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.points.forEach((val) => {
      length += DMPPoint.getMessageSize(val);
    });
    length += 8 * object.times.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dmp/DMPTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d088d86ab60cf6a2671bc3c0e99932b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # points and times should be the same length
    DMPPoint[] points
    
    # Times of observations, in seconds, starting at zero
    float64[] times
    
    
    
    ================================================================================
    MSG: dmp/DMPPoint
    # Positions and velocities of DOFs
    #Velocity is only used for movement plans, not for giving demonstrations.
    float64[] positions
    float64[] velocities
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DMPTraj(null);
    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = DMPPoint.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    if (msg.times !== undefined) {
      resolved.times = msg.times;
    }
    else {
      resolved.times = []
    }

    return resolved;
    }
};

module.exports = DMPTraj;
