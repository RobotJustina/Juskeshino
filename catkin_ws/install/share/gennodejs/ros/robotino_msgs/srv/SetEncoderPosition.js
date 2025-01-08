// Auto-generated. Do not edit!

// (in-package robotino_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetEncoderPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEncoderPositionRequest
    // Serialize message field [position]
    bufferOffset = _serializer.uint32(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.uint32(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEncoderPositionRequest
    let len;
    let data = new SetEncoderPositionRequest(null);
    // Deserialize message field [position]
    data.position = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotino_msgs/SetEncoderPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e17f4b08b3e2cdbbf2b0d78e4f62e5d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 position		# in encoder ticks
    uint32 velocity		# in ticks/s
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetEncoderPositionRequest(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    return resolved;
    }
};

class SetEncoderPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEncoderPositionResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEncoderPositionResponse
    let len;
    let data = new SetEncoderPositionResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotino_msgs/SetEncoderPositionResponse';
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
    const resolved = new SetEncoderPositionResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetEncoderPositionRequest,
  Response: SetEncoderPositionResponse,
  md5sum() { return 'e17f4b08b3e2cdbbf2b0d78e4f62e5d6'; },
  datatype() { return 'robotino_msgs/SetEncoderPosition'; }
};
