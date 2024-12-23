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

class JointPosVel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.positions = null;
      this.velocities = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('velocities')) {
        this.velocities = initObj.velocities
      }
      else {
        this.velocities = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointPosVel
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [positions]
    bufferOffset = _arraySerializer.float64(obj.positions, buffer, bufferOffset, null);
    // Serialize message field [velocities]
    bufferOffset = _arraySerializer.float64(obj.velocities, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointPosVel
    let len;
    let data = new JointPosVel(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [positions]
    data.positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocities]
    data.velocities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.positions.length;
    length += 8 * object.velocities.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rokae_msgs/JointPosVel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'deb4fb4efd7a8f0c7867baee56f1a9a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is for joint postion and velocity accquire.
    float64 timestamp
    float64[] positions
    float64[] velocities
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointPosVel(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.positions !== undefined) {
      resolved.positions = msg.positions;
    }
    else {
      resolved.positions = []
    }

    if (msg.velocities !== undefined) {
      resolved.velocities = msg.velocities;
    }
    else {
      resolved.velocities = []
    }

    return resolved;
    }
};

module.exports = JointPosVel;
