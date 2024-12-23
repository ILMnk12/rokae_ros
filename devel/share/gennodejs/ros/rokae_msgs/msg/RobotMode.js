// Auto-generated. Do not edit!

// (in-package rokae_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RobotMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.is_robot_connected = null;
      this.is_real_robot_enabled = null;
      this.is_robot_power_on = null;
      this.is_robot_running = null;
      this.is_program_running = null;
      this.is_emergency_stopped = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('is_robot_connected')) {
        this.is_robot_connected = initObj.is_robot_connected
      }
      else {
        this.is_robot_connected = false;
      }
      if (initObj.hasOwnProperty('is_real_robot_enabled')) {
        this.is_real_robot_enabled = initObj.is_real_robot_enabled
      }
      else {
        this.is_real_robot_enabled = false;
      }
      if (initObj.hasOwnProperty('is_robot_power_on')) {
        this.is_robot_power_on = initObj.is_robot_power_on
      }
      else {
        this.is_robot_power_on = false;
      }
      if (initObj.hasOwnProperty('is_robot_running')) {
        this.is_robot_running = initObj.is_robot_running
      }
      else {
        this.is_robot_running = false;
      }
      if (initObj.hasOwnProperty('is_program_running')) {
        this.is_program_running = initObj.is_program_running
      }
      else {
        this.is_program_running = false;
      }
      if (initObj.hasOwnProperty('is_emergency_stopped')) {
        this.is_emergency_stopped = initObj.is_emergency_stopped
      }
      else {
        this.is_emergency_stopped = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotMode
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [is_robot_connected]
    bufferOffset = _serializer.bool(obj.is_robot_connected, buffer, bufferOffset);
    // Serialize message field [is_real_robot_enabled]
    bufferOffset = _serializer.bool(obj.is_real_robot_enabled, buffer, bufferOffset);
    // Serialize message field [is_robot_power_on]
    bufferOffset = _serializer.bool(obj.is_robot_power_on, buffer, bufferOffset);
    // Serialize message field [is_robot_running]
    bufferOffset = _serializer.bool(obj.is_robot_running, buffer, bufferOffset);
    // Serialize message field [is_program_running]
    bufferOffset = _serializer.bool(obj.is_program_running, buffer, bufferOffset);
    // Serialize message field [is_emergency_stopped]
    bufferOffset = _serializer.bool(obj.is_emergency_stopped, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotMode
    let len;
    let data = new RobotMode(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_robot_connected]
    data.is_robot_connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_real_robot_enabled]
    data.is_real_robot_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_robot_power_on]
    data.is_robot_power_on = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_robot_running]
    data.is_robot_running = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_program_running]
    data.is_program_running = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_emergency_stopped]
    data.is_emergency_stopped = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rokae_msgs/RobotMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37b92b84717583128a825ed4248ac20e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is for robot state mode structure
    float64 timestamp
    bool is_robot_connected
    bool is_real_robot_enabled
    bool is_robot_power_on
    bool is_robot_running
    bool is_program_running
    bool is_emergency_stopped
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotMode(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.is_robot_connected !== undefined) {
      resolved.is_robot_connected = msg.is_robot_connected;
    }
    else {
      resolved.is_robot_connected = false
    }

    if (msg.is_real_robot_enabled !== undefined) {
      resolved.is_real_robot_enabled = msg.is_real_robot_enabled;
    }
    else {
      resolved.is_real_robot_enabled = false
    }

    if (msg.is_robot_power_on !== undefined) {
      resolved.is_robot_power_on = msg.is_robot_power_on;
    }
    else {
      resolved.is_robot_power_on = false
    }

    if (msg.is_robot_running !== undefined) {
      resolved.is_robot_running = msg.is_robot_running;
    }
    else {
      resolved.is_robot_running = false
    }

    if (msg.is_program_running !== undefined) {
      resolved.is_program_running = msg.is_program_running;
    }
    else {
      resolved.is_program_running = false
    }

    if (msg.is_emergency_stopped !== undefined) {
      resolved.is_emergency_stopped = msg.is_emergency_stopped;
    }
    else {
      resolved.is_emergency_stopped = false
    }

    return resolved;
    }
};

module.exports = RobotMode;
