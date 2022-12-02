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

class ExecuteJSRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.js = null;
    }
    else {
      if (initObj.hasOwnProperty('js')) {
        this.js = initObj.js
      }
      else {
        this.js = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteJSRequest
    // Serialize message field [js]
    bufferOffset = _serializer.string(obj.js, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteJSRequest
    let len;
    let data = new ExecuteJSRequest(null);
    // Deserialize message field [js]
    data.js = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.js);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pepper_nodes/ExecuteJSRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e04c1e56762d4f55445fb8ba190fb908';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string js
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteJSRequest(null);
    if (msg.js !== undefined) {
      resolved.js = msg.js;
    }
    else {
      resolved.js = ''
    }

    return resolved;
    }
};

class ExecuteJSResponse {
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
    // Serializes a message object of type ExecuteJSResponse
    // Serialize message field [ack]
    bufferOffset = _serializer.string(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteJSResponse
    let len;
    let data = new ExecuteJSResponse(null);
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
    return 'pepper_nodes/ExecuteJSResponse';
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
    const resolved = new ExecuteJSResponse(null);
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
  Request: ExecuteJSRequest,
  Response: ExecuteJSResponse,
  md5sum() { return '0bc1212ef5c5830fe8dbd8060c89a89d'; },
  datatype() { return 'pepper_nodes/ExecuteJS'; }
};
