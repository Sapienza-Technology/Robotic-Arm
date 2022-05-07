// Auto-generated. Do not edit!

// (in-package braccio_urdf_description.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Float6Array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x1 = null;
      this.x2 = null;
      this.x3 = null;
      this.x4 = null;
      this.x5 = null;
      this.x6 = null;
    }
    else {
      if (initObj.hasOwnProperty('x1')) {
        this.x1 = initObj.x1
      }
      else {
        this.x1 = 0.0;
      }
      if (initObj.hasOwnProperty('x2')) {
        this.x2 = initObj.x2
      }
      else {
        this.x2 = 0.0;
      }
      if (initObj.hasOwnProperty('x3')) {
        this.x3 = initObj.x3
      }
      else {
        this.x3 = 0.0;
      }
      if (initObj.hasOwnProperty('x4')) {
        this.x4 = initObj.x4
      }
      else {
        this.x4 = 0.0;
      }
      if (initObj.hasOwnProperty('x5')) {
        this.x5 = initObj.x5
      }
      else {
        this.x5 = 0.0;
      }
      if (initObj.hasOwnProperty('x6')) {
        this.x6 = initObj.x6
      }
      else {
        this.x6 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Float6Array
    // Serialize message field [x1]
    bufferOffset = _serializer.float64(obj.x1, buffer, bufferOffset);
    // Serialize message field [x2]
    bufferOffset = _serializer.float64(obj.x2, buffer, bufferOffset);
    // Serialize message field [x3]
    bufferOffset = _serializer.float64(obj.x3, buffer, bufferOffset);
    // Serialize message field [x4]
    bufferOffset = _serializer.float64(obj.x4, buffer, bufferOffset);
    // Serialize message field [x5]
    bufferOffset = _serializer.float64(obj.x5, buffer, bufferOffset);
    // Serialize message field [x6]
    bufferOffset = _serializer.float64(obj.x6, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Float6Array
    let len;
    let data = new Float6Array(null);
    // Deserialize message field [x1]
    data.x1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x2]
    data.x2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x3]
    data.x3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x4]
    data.x4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x5]
    data.x5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x6]
    data.x6 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'braccio_urdf_description/Float6Array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd6ec219eb9feb2baf2bfcaae69e9c5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x1
    float64 x2
    float64 x3
    float64 x4
    float64 x5
    float64 x6
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Float6Array(null);
    if (msg.x1 !== undefined) {
      resolved.x1 = msg.x1;
    }
    else {
      resolved.x1 = 0.0
    }

    if (msg.x2 !== undefined) {
      resolved.x2 = msg.x2;
    }
    else {
      resolved.x2 = 0.0
    }

    if (msg.x3 !== undefined) {
      resolved.x3 = msg.x3;
    }
    else {
      resolved.x3 = 0.0
    }

    if (msg.x4 !== undefined) {
      resolved.x4 = msg.x4;
    }
    else {
      resolved.x4 = 0.0
    }

    if (msg.x5 !== undefined) {
      resolved.x5 = msg.x5;
    }
    else {
      resolved.x5 = 0.0
    }

    if (msg.x6 !== undefined) {
      resolved.x6 = msg.x6;
    }
    else {
      resolved.x6 = 0.0
    }

    return resolved;
    }
};

module.exports = Float6Array;
