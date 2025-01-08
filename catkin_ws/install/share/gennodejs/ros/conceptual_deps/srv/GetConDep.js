// Auto-generated. Do not edit!

// (in-package conceptual_deps.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let StringArray = require('../msg/StringArray.js');

//-----------------------------------------------------------

class GetConDepRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConDepRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConDepRequest
    let len;
    let data = new GetConDepRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'conceptual_deps/GetConDepRequest';
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
    const resolved = new GetConDepRequest(null);
    return resolved;
    }
};

class GetConDepResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cds = null;
    }
    else {
      if (initObj.hasOwnProperty('cds')) {
        this.cds = initObj.cds
      }
      else {
        this.cds = new StringArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConDepResponse
    // Serialize message field [cds]
    bufferOffset = StringArray.serialize(obj.cds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConDepResponse
    let len;
    let data = new GetConDepResponse(null);
    // Deserialize message field [cds]
    data.cds = StringArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += StringArray.getMessageSize(object.cds);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'conceptual_deps/GetConDepResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6530474bcb00416881ea95591006435a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    StringArray  cds
    
    
    ================================================================================
    MSG: conceptual_deps/StringArray
    string[]  data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetConDepResponse(null);
    if (msg.cds !== undefined) {
      resolved.cds = StringArray.Resolve(msg.cds)
    }
    else {
      resolved.cds = new StringArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetConDepRequest,
  Response: GetConDepResponse,
  md5sum() { return '6530474bcb00416881ea95591006435a'; },
  datatype() { return 'conceptual_deps/GetConDep'; }
};
