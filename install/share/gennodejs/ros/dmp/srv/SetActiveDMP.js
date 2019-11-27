// Auto-generated. Do not edit!

// (in-package dmp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DMPData = require('../msg/DMPData.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetActiveDMPRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dmp_list = null;
    }
    else {
      if (initObj.hasOwnProperty('dmp_list')) {
        this.dmp_list = initObj.dmp_list
      }
      else {
        this.dmp_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetActiveDMPRequest
    // Serialize message field [dmp_list]
    // Serialize the length for message field [dmp_list]
    bufferOffset = _serializer.uint32(obj.dmp_list.length, buffer, bufferOffset);
    obj.dmp_list.forEach((val) => {
      bufferOffset = DMPData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetActiveDMPRequest
    let len;
    let data = new SetActiveDMPRequest(null);
    // Deserialize message field [dmp_list]
    // Deserialize array length for message field [dmp_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.dmp_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.dmp_list[i] = DMPData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.dmp_list.forEach((val) => {
      length += DMPData.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/SetActiveDMPRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec873fb8fce06b1431bf639280c06e64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    DMPData[] dmp_list
    
    
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
    const resolved = new SetActiveDMPRequest(null);
    if (msg.dmp_list !== undefined) {
      resolved.dmp_list = new Array(msg.dmp_list.length);
      for (let i = 0; i < resolved.dmp_list.length; ++i) {
        resolved.dmp_list[i] = DMPData.Resolve(msg.dmp_list[i]);
      }
    }
    else {
      resolved.dmp_list = []
    }

    return resolved;
    }
};

class SetActiveDMPResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetActiveDMPResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetActiveDMPResponse
    let len;
    let data = new SetActiveDMPResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/SetActiveDMPResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    bool success
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetActiveDMPResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetActiveDMPRequest,
  Response: SetActiveDMPResponse,
  md5sum() { return '10d64adb0a08cbb7afbd425801f828e5'; },
  datatype() { return 'dmp/SetActiveDMP'; }
};
