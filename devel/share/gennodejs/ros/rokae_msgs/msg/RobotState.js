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

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.joint_pose = null;
      this.joint_velocity = null;
      this.joint_cmd_acceleration = null;
      this.joint_torque = null;
      this.joint_filter_torque = null;
      this.arm_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('joint_pose')) {
        this.joint_pose = initObj.joint_pose
      }
      else {
        this.joint_pose = [];
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = [];
      }
      if (initObj.hasOwnProperty('joint_cmd_acceleration')) {
        this.joint_cmd_acceleration = initObj.joint_cmd_acceleration
      }
      else {
        this.joint_cmd_acceleration = [];
      }
      if (initObj.hasOwnProperty('joint_torque')) {
        this.joint_torque = initObj.joint_torque
      }
      else {
        this.joint_torque = [];
      }
      if (initObj.hasOwnProperty('joint_filter_torque')) {
        this.joint_filter_torque = initObj.joint_filter_torque
      }
      else {
        this.joint_filter_torque = [];
      }
      if (initObj.hasOwnProperty('arm_angle')) {
        this.arm_angle = initObj.arm_angle
      }
      else {
        this.arm_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [joint_pose]
    bufferOffset = _arraySerializer.float64(obj.joint_pose, buffer, bufferOffset, null);
    // Serialize message field [joint_velocity]
    bufferOffset = _arraySerializer.float64(obj.joint_velocity, buffer, bufferOffset, null);
    // Serialize message field [joint_cmd_acceleration]
    bufferOffset = _arraySerializer.float64(obj.joint_cmd_acceleration, buffer, bufferOffset, null);
    // Serialize message field [joint_torque]
    bufferOffset = _arraySerializer.float64(obj.joint_torque, buffer, bufferOffset, null);
    // Serialize message field [joint_filter_torque]
    bufferOffset = _arraySerializer.float64(obj.joint_filter_torque, buffer, bufferOffset, null);
    // Serialize message field [arm_angle]
    bufferOffset = _serializer.float64(obj.arm_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_pose]
    data.joint_pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_cmd_acceleration]
    data.joint_cmd_acceleration = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_torque]
    data.joint_torque = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_filter_torque]
    data.joint_filter_torque = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [arm_angle]
    data.arm_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_pose.length;
    length += 8 * object.joint_velocity.length;
    length += 8 * object.joint_cmd_acceleration.length;
    length += 8 * object.joint_torque.length;
    length += 8 * object.joint_filter_torque.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rokae_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd80e953e8359840e704f48ebdde5ced';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Data structure for robot status description
    
    # timestamp
    float64 timestamp                    # time stamp   
    
    # joint status
    float64[] joint_pose                 # joint_pose
    float64[] joint_velocity             # joint_velocity
    float64[] joint_cmd_acceleration         # joint_acceleration
    float64[] joint_torque               # joint_torque
    float64[] joint_filter_torque        # joint_filter_torque
    
    # arm angle state
    float64 arm_angle                    # arm angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotState(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.joint_pose !== undefined) {
      resolved.joint_pose = msg.joint_pose;
    }
    else {
      resolved.joint_pose = []
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = []
    }

    if (msg.joint_cmd_acceleration !== undefined) {
      resolved.joint_cmd_acceleration = msg.joint_cmd_acceleration;
    }
    else {
      resolved.joint_cmd_acceleration = []
    }

    if (msg.joint_torque !== undefined) {
      resolved.joint_torque = msg.joint_torque;
    }
    else {
      resolved.joint_torque = []
    }

    if (msg.joint_filter_torque !== undefined) {
      resolved.joint_filter_torque = msg.joint_filter_torque;
    }
    else {
      resolved.joint_filter_torque = []
    }

    if (msg.arm_angle !== undefined) {
      resolved.arm_angle = msg.arm_angle;
    }
    else {
      resolved.arm_angle = 0.0
    }

    return resolved;
    }
};

module.exports = RobotState;
