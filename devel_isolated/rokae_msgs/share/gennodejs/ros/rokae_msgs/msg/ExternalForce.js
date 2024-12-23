// Auto-generated. Do not edit!

// (in-package rokae_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ExternalForce {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force = null;
      this.moment = null;
    }
    else {
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('moment')) {
        this.moment = initObj.moment
      }
      else {
        this.moment = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExternalForce
    // Serialize message field [force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.force, buffer, bufferOffset);
    // Serialize message field [moment]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.moment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExternalForce
    let len;
    let data = new ExternalForce(null);
    // Deserialize message field [force]
    data.force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [moment]
    data.moment = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rokae_msgs/ExternalForce';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '572f71e60f73d6a9e5c0b5d4c329a492';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # force vector separated into force and moment
    geometry_msgs/Vector3 force
    geometry_msgs/Vector3 moment
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExternalForce(null);
    if (msg.force !== undefined) {
      resolved.force = geometry_msgs.msg.Vector3.Resolve(msg.force)
    }
    else {
      resolved.force = new geometry_msgs.msg.Vector3()
    }

    if (msg.moment !== undefined) {
      resolved.moment = geometry_msgs.msg.Vector3.Resolve(msg.moment)
    }
    else {
      resolved.moment = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = ExternalForce;
