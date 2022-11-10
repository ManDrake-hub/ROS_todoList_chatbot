// Auto-generated. Do not edit!

// (in-package rasa_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DialogueRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_text = null;
    }
    else {
      if (initObj.hasOwnProperty('input_text')) {
        this.input_text = initObj.input_text
      }
      else {
        this.input_text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DialogueRequest
    // Serialize message field [input_text]
    bufferOffset = _serializer.string(obj.input_text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DialogueRequest
    let len;
    let data = new DialogueRequest(null);
    // Deserialize message field [input_text]
    data.input_text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.input_text);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rasa_ros/DialogueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a278206837cb7b09dde368e542dc10d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string input_text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DialogueRequest(null);
    if (msg.input_text !== undefined) {
      resolved.input_text = msg.input_text;
    }
    else {
      resolved.input_text = ''
    }

    return resolved;
    }
};

class DialogueResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.answer = null;
    }
    else {
      if (initObj.hasOwnProperty('answer')) {
        this.answer = initObj.answer
      }
      else {
        this.answer = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DialogueResponse
    // Serialize message field [answer]
    bufferOffset = _serializer.string(obj.answer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DialogueResponse
    let len;
    let data = new DialogueResponse(null);
    // Deserialize message field [answer]
    data.answer = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.answer);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rasa_ros/DialogueResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7e708f879c94bb931716d8f4f130f30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string answer
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DialogueResponse(null);
    if (msg.answer !== undefined) {
      resolved.answer = msg.answer;
    }
    else {
      resolved.answer = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: DialogueRequest,
  Response: DialogueResponse,
  md5sum() { return 'af708bc8927c16924fd75aabb46f9abb'; },
  datatype() { return 'rasa_ros/Dialogue'; }
};
