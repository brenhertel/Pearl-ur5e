// Auto-generated. Do not edit!

// (in-package brendan_ur5e.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class shutdown_requestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.a = null;
    }
    else {
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shutdown_requestRequest
    // Serialize message field [a]
    bufferOffset = _serializer.int64(obj.a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shutdown_requestRequest
    let len;
    let data = new shutdown_requestRequest(null);
    // Deserialize message field [a]
    data.a = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'brendan_ur5e/shutdown_requestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '019706110004b728d56d8baaa8e32797';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shutdown_requestRequest(null);
    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0
    }

    return resolved;
    }
};

class shutdown_requestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shutdown_requestResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shutdown_requestResponse
    let len;
    let data = new shutdown_requestResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'brendan_ur5e/shutdown_requestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shutdown_requestResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: shutdown_requestRequest,
  Response: shutdown_requestResponse,
  md5sum() { return '019706110004b728d56d8baaa8e32797'; },
  datatype() { return 'brendan_ur5e/shutdown_request'; }
};
