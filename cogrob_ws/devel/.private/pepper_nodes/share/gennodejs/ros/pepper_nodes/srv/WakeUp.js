// Auto-generated. Do not edit!

// (in-package pepper_nodes.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class WakeUpRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WakeUpRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WakeUpRequest
    let len;
    let data = new WakeUpRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pepper_nodes/WakeUpRequest';
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
    const resolved = new WakeUpRequest(null);
    return resolved;
    }
};

class WakeUpResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ack = null;
    }
    else {
      if (initObj.hasOwnProperty('ack')) {
        this.ack = initObj.ack
      }
      else {
        this.ack = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WakeUpResponse
    // Serialize message field [ack]
    bufferOffset = _serializer.string(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WakeUpResponse
    let len;
    let data = new WakeUpResponse(null);
    // Deserialize message field [ack]
    data.ack = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.ack);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pepper_nodes/WakeUpResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6a73f722475d64a28238118997ef482';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string ack
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WakeUpResponse(null);
    if (msg.ack !== undefined) {
      resolved.ack = msg.ack;
    }
    else {
      resolved.ack = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: WakeUpRequest,
  Response: WakeUpResponse,
  md5sum() { return 'b6a73f722475d64a28238118997ef482'; },
  datatype() { return 'pepper_nodes/WakeUp'; }
};
