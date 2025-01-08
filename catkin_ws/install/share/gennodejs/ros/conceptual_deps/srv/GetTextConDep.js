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

class GetTextConDepRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTextConDepRequest
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTextConDepRequest
    let len;
    let data = new GetTextConDepRequest(null);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.text);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'conceptual_deps/GetTextConDepRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74697ed3d931f6eede8bf3a8dfeca160';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTextConDepRequest(null);
    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

class GetTextConDepResponse {
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
    // Serializes a message object of type GetTextConDepResponse
    // Serialize message field [cds]
    bufferOffset = StringArray.serialize(obj.cds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTextConDepResponse
    let len;
    let data = new GetTextConDepResponse(null);
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
    return 'conceptual_deps/GetTextConDepResponse';
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
    const resolved = new GetTextConDepResponse(null);
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
  Request: GetTextConDepRequest,
  Response: GetTextConDepResponse,
  md5sum() { return 'b7e9f26875b56c920fd22b0d43671c0b'; },
  datatype() { return 'conceptual_deps/GetTextConDep'; }
};
