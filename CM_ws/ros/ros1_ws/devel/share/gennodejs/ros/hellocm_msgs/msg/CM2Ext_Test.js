// Auto-generated. Do not edit!

// (in-package hellocm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CM2Ext_Test {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.test1 = null;
    }
    else {
      if (initObj.hasOwnProperty('test1')) {
        this.test1 = initObj.test1
      }
      else {
        this.test1 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CM2Ext_Test
    // Serialize message field [test1]
    bufferOffset = _serializer.float64(obj.test1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CM2Ext_Test
    let len;
    let data = new CM2Ext_Test(null);
    // Deserialize message field [test1]
    data.test1 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hellocm_msgs/CM2Ext_Test';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1544912b023f841df84f892fda2b3c07';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 test1
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CM2Ext_Test(null);
    if (msg.test1 !== undefined) {
      resolved.test1 = msg.test1;
    }
    else {
      resolved.test1 = 0.0
    }

    return resolved;
    }
};

module.exports = CM2Ext_Test;
