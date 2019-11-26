// Auto-generated. Do not edit!

// (in-package dmp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DMPTraj = require('../msg/DMPTraj.js');

//-----------------------------------------------------------

let DMPData = require('../msg/DMPData.js');

//-----------------------------------------------------------

class LearnDMPFromDemoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.demo = null;
      this.k_gains = null;
      this.d_gains = null;
      this.num_bases = null;
    }
    else {
      if (initObj.hasOwnProperty('demo')) {
        this.demo = initObj.demo
      }
      else {
        this.demo = new DMPTraj();
      }
      if (initObj.hasOwnProperty('k_gains')) {
        this.k_gains = initObj.k_gains
      }
      else {
        this.k_gains = [];
      }
      if (initObj.hasOwnProperty('d_gains')) {
        this.d_gains = initObj.d_gains
      }
      else {
        this.d_gains = [];
      }
      if (initObj.hasOwnProperty('num_bases')) {
        this.num_bases = initObj.num_bases
      }
      else {
        this.num_bases = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LearnDMPFromDemoRequest
    // Serialize message field [demo]
    bufferOffset = DMPTraj.serialize(obj.demo, buffer, bufferOffset);
    // Serialize message field [k_gains]
    bufferOffset = _arraySerializer.float64(obj.k_gains, buffer, bufferOffset, null);
    // Serialize message field [d_gains]
    bufferOffset = _arraySerializer.float64(obj.d_gains, buffer, bufferOffset, null);
    // Serialize message field [num_bases]
    bufferOffset = _serializer.int32(obj.num_bases, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LearnDMPFromDemoRequest
    let len;
    let data = new LearnDMPFromDemoRequest(null);
    // Deserialize message field [demo]
    data.demo = DMPTraj.deserialize(buffer, bufferOffset);
    // Deserialize message field [k_gains]
    data.k_gains = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [d_gains]
    data.d_gains = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [num_bases]
    data.num_bases = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DMPTraj.getMessageSize(object.demo);
    length += 8 * object.k_gains.length;
    length += 8 * object.d_gains.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/LearnDMPFromDemoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7c68a7f789c246aea188ade43175b30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    DMPTraj demo
    
    
    float64[] k_gains
    float64[] d_gains
    
    
    int32 num_bases
    
    
    ================================================================================
    MSG: dmp/DMPTraj
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
    const resolved = new LearnDMPFromDemoRequest(null);
    if (msg.demo !== undefined) {
      resolved.demo = DMPTraj.Resolve(msg.demo)
    }
    else {
      resolved.demo = new DMPTraj()
    }

    if (msg.k_gains !== undefined) {
      resolved.k_gains = msg.k_gains;
    }
    else {
      resolved.k_gains = []
    }

    if (msg.d_gains !== undefined) {
      resolved.d_gains = msg.d_gains;
    }
    else {
      resolved.d_gains = []
    }

    if (msg.num_bases !== undefined) {
      resolved.num_bases = msg.num_bases;
    }
    else {
      resolved.num_bases = 0
    }

    return resolved;
    }
};

class LearnDMPFromDemoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dmp_list = null;
      this.tau = null;
    }
    else {
      if (initObj.hasOwnProperty('dmp_list')) {
        this.dmp_list = initObj.dmp_list
      }
      else {
        this.dmp_list = [];
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LearnDMPFromDemoResponse
    // Serialize message field [dmp_list]
    // Serialize the length for message field [dmp_list]
    bufferOffset = _serializer.uint32(obj.dmp_list.length, buffer, bufferOffset);
    obj.dmp_list.forEach((val) => {
      bufferOffset = DMPData.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [tau]
    bufferOffset = _serializer.float64(obj.tau, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LearnDMPFromDemoResponse
    let len;
    let data = new LearnDMPFromDemoResponse(null);
    // Deserialize message field [dmp_list]
    // Deserialize array length for message field [dmp_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.dmp_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.dmp_list[i] = DMPData.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [tau]
    data.tau = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.dmp_list.forEach((val) => {
      length += DMPData.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/LearnDMPFromDemoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2dccae00aae58574694dfa33e62fac1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    DMPData[] dmp_list
    
    
    float64 tau
    
    
    ================================================================================
    MSG: dmp/DMPData
    float64 k_gain
    float64 d_gain
    float64[] weights
    float64[] f_domain
    float64[] f_targets
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LearnDMPFromDemoResponse(null);
    if (msg.dmp_list !== undefined) {
      resolved.dmp_list = new Array(msg.dmp_list.length);
      for (let i = 0; i < resolved.dmp_list.length; ++i) {
        resolved.dmp_list[i] = DMPData.Resolve(msg.dmp_list[i]);
      }
    }
    else {
      resolved.dmp_list = []
    }

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: LearnDMPFromDemoRequest,
  Response: LearnDMPFromDemoResponse,
  md5sum() { return '3ba13cfa47585560a2fd9cc202efdbff'; },
  datatype() { return 'dmp/LearnDMPFromDemo'; }
};
