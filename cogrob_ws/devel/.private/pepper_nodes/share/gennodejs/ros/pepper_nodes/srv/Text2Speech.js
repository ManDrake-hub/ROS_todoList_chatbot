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

class Text2SpeechRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speech = null;
    }
    else {
      if (initObj.hasOwnProperty('speech')) {
        this.speech = initObj.speech
      }
      else {
        this.speech = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Text2SpeechRequest
    // Serialize message field [speech]
    bufferOffset = _serializer.string(obj.speech, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Text2SpeechRequest
    let len;
    let data = new Text2SpeechRequest(null);
    // Deserialize message field [speech]
    data.speech = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.speech);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pepper_nodes/Text2SpeechRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f212b08e2dfacb9148fa1a62023e9ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string speech
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Text2SpeechRequest(null);
    if (msg.speech !== undefined) {
      resolved.speech = msg.speech;
    }
    else {
      resolved.speech = ''
    }

    return resolved;
    }
};

class Text2SpeechResponse {
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
    // Serializes a message object of type Text2SpeechResponse
    // Serialize message field [ack]
    bufferOffset = _serializer.string(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Text2SpeechResponse
    let len;
    let data = new Text2SpeechResponse(null);
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
    return 'pepper_nodes/Text2SpeechResponse';
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
    const resolved = new Text2SpeechResponse(null);
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
  Request: Text2SpeechRequest,
  Response: Text2SpeechResponse,
  md5sum() { return '8be93dfc13258654eb30fdcda5227e42'; },
  datatype() { return 'pepper_nodes/Text2Speech'; }
};
