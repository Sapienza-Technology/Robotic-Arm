// Auto-generated. Do not edit!

// (in-package tag_utils.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetTagRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTagRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTagRequest
    let len;
    let data = new GetTagRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tag_utils/GetTagRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5e4a7d59c68f74eabcec876a00216aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTagRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

class GetTagResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tag = null;
    }
    else {
      if (initObj.hasOwnProperty('tag')) {
        this.tag = initObj.tag
      }
      else {
        this.tag = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTagResponse
    // Check that the constant length array field [tag] has the right length
    if (obj.tag.length !== 3) {
      throw new Error('Unable to serialize array field tag - length must be 3')
    }
    // Serialize message field [tag]
    bufferOffset = _arraySerializer.float32(obj.tag, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTagResponse
    let len;
    let data = new GetTagResponse(null);
    // Deserialize message field [tag]
    data.tag = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tag_utils/GetTagResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '437562c37a0b78009dd646ef9093b67f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[3] tag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTagResponse(null);
    if (msg.tag !== undefined) {
      resolved.tag = msg.tag;
    }
    else {
      resolved.tag = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: GetTagRequest,
  Response: GetTagResponse,
  md5sum() { return '3cb26f00cb9a158b9a65f3463d45d1c7'; },
  datatype() { return 'tag_utils/GetTag'; }
};
