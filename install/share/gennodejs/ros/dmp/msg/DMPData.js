// Auto-generated. Do not edit!

// (in-package dmp.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DMPData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.k_gain = null;
      this.d_gain = null;
      this.weights = null;
      this.f_domain = null;
      this.f_targets = null;
    }
    else {
      if (initObj.hasOwnProperty('k_gain')) {
        this.k_gain = initObj.k_gain
      }
      else {
        this.k_gain = 0.0;
      }
      if (initObj.hasOwnProperty('d_gain')) {
        this.d_gain = initObj.d_gain
      }
      else {
        this.d_gain = 0.0;
      }
      if (initObj.hasOwnProperty('weights')) {
        this.weights = initObj.weights
      }
      else {
        this.weights = [];
      }
      if (initObj.hasOwnProperty('f_domain')) {
        this.f_domain = initObj.f_domain
      }
      else {
        this.f_domain = [];
      }
      if (initObj.hasOwnProperty('f_targets')) {
        this.f_targets = initObj.f_targets
      }
      else {
        this.f_targets = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DMPData
    // Serialize message field [k_gain]
    bufferOffset = _serializer.float64(obj.k_gain, buffer, bufferOffset);
    // Serialize message field [d_gain]
    bufferOffset = _serializer.float64(obj.d_gain, buffer, bufferOffset);
    // Serialize message field [weights]
    bufferOffset = _arraySerializer.float64(obj.weights, buffer, bufferOffset, null);
    // Serialize message field [f_domain]
    bufferOffset = _arraySerializer.float64(obj.f_domain, buffer, bufferOffset, null);
    // Serialize message field [f_targets]
    bufferOffset = _arraySerializer.float64(obj.f_targets, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DMPData
    let len;
    let data = new DMPData(null);
    // Deserialize message field [k_gain]
    data.k_gain = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_gain]
    data.d_gain = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [weights]
    data.weights = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [f_domain]
    data.f_domain = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [f_targets]
    data.f_targets = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.weights.length;
    length += 8 * object.f_domain.length;
    length += 8 * object.f_targets.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dmp/DMPData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd2a2dd30705ac6b894c0c83b8081221';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DMPData(null);
    if (msg.k_gain !== undefined) {
      resolved.k_gain = msg.k_gain;
    }
    else {
      resolved.k_gain = 0.0
    }

    if (msg.d_gain !== undefined) {
      resolved.d_gain = msg.d_gain;
    }
    else {
      resolved.d_gain = 0.0
    }

    if (msg.weights !== undefined) {
      resolved.weights = msg.weights;
    }
    else {
      resolved.weights = []
    }

    if (msg.f_domain !== undefined) {
      resolved.f_domain = msg.f_domain;
    }
    else {
      resolved.f_domain = []
    }

    if (msg.f_targets !== undefined) {
      resolved.f_targets = msg.f_targets;
    }
    else {
      resolved.f_targets = []
    }

    return resolved;
    }
};

module.exports = DMPData;
