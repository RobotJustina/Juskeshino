// Auto-generated. Do not edit!

// (in-package robotino_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorReadings {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.velocities = null;
      this.positions = null;
      this.currents = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('velocities')) {
        this.velocities = initObj.velocities
      }
      else {
        this.velocities = [];
      }
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('currents')) {
        this.currents = initObj.currents
      }
      else {
        this.currents = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorReadings
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [velocities]
    bufferOffset = _arraySerializer.float32(obj.velocities, buffer, bufferOffset, null);
    // Serialize message field [positions]
    bufferOffset = _arraySerializer.int32(obj.positions, buffer, bufferOffset, null);
    // Serialize message field [currents]
    bufferOffset = _arraySerializer.float32(obj.currents, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorReadings
    let len;
    let data = new MotorReadings(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [velocities]
    data.velocities = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [positions]
    data.positions = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [currents]
    data.currents = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.velocities.length;
    length += 4 * object.positions.length;
    length += 4 * object.currents.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotino_msgs/MotorReadings';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3974e9bd8305667fc0637697b49a8e1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    float32[] velocities 	# in rpm
    int32[] positions
    float32[] currents		# in A
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorReadings(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.velocities !== undefined) {
      resolved.velocities = msg.velocities;
    }
    else {
      resolved.velocities = []
    }

    if (msg.positions !== undefined) {
      resolved.positions = msg.positions;
    }
    else {
      resolved.positions = []
    }

    if (msg.currents !== undefined) {
      resolved.currents = msg.currents;
    }
    else {
      resolved.currents = []
    }

    return resolved;
    }
};

module.exports = MotorReadings;
