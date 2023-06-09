// Auto-generated. Do not edit!

// (in-package detection_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ObjectHypothesisWithPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Class = null;
      this.score = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('Class')) {
        this.Class = initObj.Class
      }
      else {
        this.Class = '';
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectHypothesisWithPose
    // Serialize message field [Class]
    bufferOffset = _serializer.string(obj.Class, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float64(obj.score, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectHypothesisWithPose
    let len;
    let data = new ObjectHypothesisWithPose(null);
    // Deserialize message field [Class]
    data.Class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Class);
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection_msgs/ObjectHypothesisWithPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89cf9c36df70a761869a8c1170b49418';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # An object hypothesis that contains position information.
    
    # The unique numeric ID of object detected. To get additional information about
    #   this ID, such as its human-readable name, listeners should perform a lookup
    #   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
    string Class
    
    # The probability or confidence value of the detected object. By convention,
    #   this value should lie in the range [0-1].
    float64 score
    
    # The 6D pose of the object hypothesis. This pose should be
    #   defined as the pose of some fixed reference point on the object, such a
    #   the geometric center of the bounding box or the center of mass of the
    #   object.
    # Note that this pose is not stamped; frame information can be defined by
    #   parent messages.
    # Also note that different classes predicted for the same input data may have
    #   different predicted 6D poses.
    geometry_msgs/Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectHypothesisWithPose(null);
    if (msg.Class !== undefined) {
      resolved.Class = msg.Class;
    }
    else {
      resolved.Class = ''
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = ObjectHypothesisWithPose;
