// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RTKRel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.differential_age = null;
      this.ar_ratio = null;
      this.vector_to_base = null;
      this.distance_to_base = null;
      this.heading_to_base = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('differential_age')) {
        this.differential_age = initObj.differential_age
      }
      else {
        this.differential_age = 0.0;
      }
      if (initObj.hasOwnProperty('ar_ratio')) {
        this.ar_ratio = initObj.ar_ratio
      }
      else {
        this.ar_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('vector_to_base')) {
        this.vector_to_base = initObj.vector_to_base
      }
      else {
        this.vector_to_base = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('distance_to_base')) {
        this.distance_to_base = initObj.distance_to_base
      }
      else {
        this.distance_to_base = 0.0;
      }
      if (initObj.hasOwnProperty('heading_to_base')) {
        this.heading_to_base = initObj.heading_to_base
      }
      else {
        this.heading_to_base = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RTKRel
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [differential_age]
    bufferOffset = _serializer.float32(obj.differential_age, buffer, bufferOffset);
    // Serialize message field [ar_ratio]
    bufferOffset = _serializer.float32(obj.ar_ratio, buffer, bufferOffset);
    // Serialize message field [vector_to_base]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vector_to_base, buffer, bufferOffset);
    // Serialize message field [distance_to_base]
    bufferOffset = _serializer.float32(obj.distance_to_base, buffer, bufferOffset);
    // Serialize message field [heading_to_base]
    bufferOffset = _serializer.float32(obj.heading_to_base, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RTKRel
    let len;
    let data = new RTKRel(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [differential_age]
    data.differential_age = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ar_ratio]
    data.ar_ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vector_to_base]
    data.vector_to_base = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance_to_base]
    data.distance_to_base = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_to_base]
    data.heading_to_base = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/RTKRel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08bc407c0d6e7678dc9accbcee16cae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 differential_age 				# Age of differential (seconds)
    float32 ar_ratio 						# Ambiguity resolution ratio factor for validation
    geometry_msgs/Vector3 vector_to_base 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1
    float32 distance_to_base 				# Distance to Base (m)
    float32 heading_to_base 				# Angle from north to vectorToBase in local tangent plane. (rad)
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RTKRel(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.differential_age !== undefined) {
      resolved.differential_age = msg.differential_age;
    }
    else {
      resolved.differential_age = 0.0
    }

    if (msg.ar_ratio !== undefined) {
      resolved.ar_ratio = msg.ar_ratio;
    }
    else {
      resolved.ar_ratio = 0.0
    }

    if (msg.vector_to_base !== undefined) {
      resolved.vector_to_base = geometry_msgs.msg.Vector3.Resolve(msg.vector_to_base)
    }
    else {
      resolved.vector_to_base = new geometry_msgs.msg.Vector3()
    }

    if (msg.distance_to_base !== undefined) {
      resolved.distance_to_base = msg.distance_to_base;
    }
    else {
      resolved.distance_to_base = 0.0
    }

    if (msg.heading_to_base !== undefined) {
      resolved.heading_to_base = msg.heading_to_base;
    }
    else {
      resolved.heading_to_base = 0.0
    }

    return resolved;
    }
};

module.exports = RTKRel;
