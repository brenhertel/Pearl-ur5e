// Auto-generated. Do not edit!

// (in-package dmp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let DMPTraj = require('../msg/DMPTraj.js');

//-----------------------------------------------------------

class GetDMPPlanRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_0 = null;
      this.x_dot_0 = null;
      this.t_0 = null;
      this.goal = null;
      this.goal_thresh = null;
      this.seg_length = null;
      this.tau = null;
      this.dt = null;
      this.integrate_iter = null;
    }
    else {
      if (initObj.hasOwnProperty('x_0')) {
        this.x_0 = initObj.x_0
      }
      else {
        this.x_0 = [];
      }
      if (initObj.hasOwnProperty('x_dot_0')) {
        this.x_dot_0 = initObj.x_dot_0
      }
      else {
        this.x_dot_0 = [];
      }
      if (initObj.hasOwnProperty('t_0')) {
        this.t_0 = initObj.t_0
      }
      else {
        this.t_0 = 0.0;
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = [];
      }
      if (initObj.hasOwnProperty('goal_thresh')) {
        this.goal_thresh = initObj.goal_thresh
      }
      else {
        this.goal_thresh = [];
      }
      if (initObj.hasOwnProperty('seg_length')) {
        this.seg_length = initObj.seg_length
      }
      else {
        this.seg_length = 0.0;
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = 0.0;
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = 0.0;
      }
      if (initObj.hasOwnProperty('integrate_iter')) {
        this.integrate_iter = initObj.integrate_iter
      }
      else {
        this.integrate_iter = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDMPPlanRequest
    // Serialize message field [x_0]
    bufferOffset = _arraySerializer.float64(obj.x_0, buffer, bufferOffset, null);
    // Serialize message field [x_dot_0]
    bufferOffset = _arraySerializer.float64(obj.x_dot_0, buffer, bufferOffset, null);
    // Serialize message field [t_0]
    bufferOffset = _serializer.float64(obj.t_0, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = _arraySerializer.float64(obj.goal, buffer, bufferOffset, null);
    // Serialize message field [goal_thresh]
    bufferOffset = _arraySerializer.float64(obj.goal_thresh, buffer, bufferOffset, null);
    // Serialize message field [seg_length]
    bufferOffset = _serializer.float64(obj.seg_length, buffer, bufferOffset);
    // Serialize message field [tau]
    bufferOffset = _serializer.float64(obj.tau, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _serializer.float64(obj.dt, buffer, bufferOffset);
    // Serialize message field [integrate_iter]
    bufferOffset = _serializer.int32(obj.integrate_iter, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDMPPlanRequest
    let len;
    let data = new GetDMPPlanRequest(null);
    // Deserialize message field [x_0]
    data.x_0 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [x_dot_0]
    data.x_dot_0 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [t_0]
    data.t_0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [goal_thresh]
    data.goal_thresh = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [seg_length]
    data.seg_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tau]
    data.tau = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [integrate_iter]
    data.integrate_iter = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.x_0.length;
    length += 8 * object.x_dot_0.length;
    length += 8 * object.goal.length;
    length += 8 * object.goal_thresh.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/GetDMPPlanRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bae6b051e2f7b80225be1fd25b5b705a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float64[] x_0
    
    
    float64[] x_dot_0
    
    
    
    float64 t_0
    
    
    float64[] goal
    
    
    
    
    
    float64[] goal_thresh
    
    
    float64 seg_length
    
    
    float64 tau
    
    
    float64 dt
    
    
    int32 integrate_iter
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDMPPlanRequest(null);
    if (msg.x_0 !== undefined) {
      resolved.x_0 = msg.x_0;
    }
    else {
      resolved.x_0 = []
    }

    if (msg.x_dot_0 !== undefined) {
      resolved.x_dot_0 = msg.x_dot_0;
    }
    else {
      resolved.x_dot_0 = []
    }

    if (msg.t_0 !== undefined) {
      resolved.t_0 = msg.t_0;
    }
    else {
      resolved.t_0 = 0.0
    }

    if (msg.goal !== undefined) {
      resolved.goal = msg.goal;
    }
    else {
      resolved.goal = []
    }

    if (msg.goal_thresh !== undefined) {
      resolved.goal_thresh = msg.goal_thresh;
    }
    else {
      resolved.goal_thresh = []
    }

    if (msg.seg_length !== undefined) {
      resolved.seg_length = msg.seg_length;
    }
    else {
      resolved.seg_length = 0.0
    }

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = 0.0
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = 0.0
    }

    if (msg.integrate_iter !== undefined) {
      resolved.integrate_iter = msg.integrate_iter;
    }
    else {
      resolved.integrate_iter = 0
    }

    return resolved;
    }
};

class GetDMPPlanResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plan = null;
      this.at_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('plan')) {
        this.plan = initObj.plan
      }
      else {
        this.plan = new DMPTraj();
      }
      if (initObj.hasOwnProperty('at_goal')) {
        this.at_goal = initObj.at_goal
      }
      else {
        this.at_goal = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDMPPlanResponse
    // Serialize message field [plan]
    bufferOffset = DMPTraj.serialize(obj.plan, buffer, bufferOffset);
    // Serialize message field [at_goal]
    bufferOffset = _serializer.uint8(obj.at_goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDMPPlanResponse
    let len;
    let data = new GetDMPPlanResponse(null);
    // Deserialize message field [plan]
    data.plan = DMPTraj.deserialize(buffer, bufferOffset);
    // Deserialize message field [at_goal]
    data.at_goal = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DMPTraj.getMessageSize(object.plan);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/GetDMPPlanResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dcf9f84a71b2617cf4bbc301a97c05cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    DMPTraj plan
    
    
    uint8 at_goal
    
    
    
    
    
    
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
    const resolved = new GetDMPPlanResponse(null);
    if (msg.plan !== undefined) {
      resolved.plan = DMPTraj.Resolve(msg.plan)
    }
    else {
      resolved.plan = new DMPTraj()
    }

    if (msg.at_goal !== undefined) {
      resolved.at_goal = msg.at_goal;
    }
    else {
      resolved.at_goal = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetDMPPlanRequest,
  Response: GetDMPPlanResponse,
  md5sum() { return '5cd79fd80676a4f8f062c5472a3190b1'; },
  datatype() { return 'dmp/GetDMPPlan'; }
};
