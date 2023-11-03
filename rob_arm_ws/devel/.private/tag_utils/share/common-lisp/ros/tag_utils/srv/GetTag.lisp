; Auto-generated. Do not edit!


(cl:in-package tag_utils-srv)


;//! \htmlinclude GetTag-request.msg.html

(cl:defclass <GetTag-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass GetTag-request (<GetTag-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTag-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTag-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tag_utils-srv:<GetTag-request> is deprecated: use tag_utils-srv:GetTag-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetTag-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tag_utils-srv:id-val is deprecated.  Use tag_utils-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTag-request>) ostream)
  "Serializes a message object of type '<GetTag-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTag-request>) istream)
  "Deserializes a message object of type '<GetTag-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTag-request>)))
  "Returns string type for a service object of type '<GetTag-request>"
  "tag_utils/GetTagRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTag-request)))
  "Returns string type for a service object of type 'GetTag-request"
  "tag_utils/GetTagRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTag-request>)))
  "Returns md5sum for a message object of type '<GetTag-request>"
  "3cb26f00cb9a158b9a65f3463d45d1c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTag-request)))
  "Returns md5sum for a message object of type 'GetTag-request"
  "3cb26f00cb9a158b9a65f3463d45d1c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTag-request>)))
  "Returns full string definition for message of type '<GetTag-request>"
  (cl:format cl:nil "int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTag-request)))
  "Returns full string definition for message of type 'GetTag-request"
  (cl:format cl:nil "int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTag-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTag-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTag-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude GetTag-response.msg.html

(cl:defclass <GetTag-response> (roslisp-msg-protocol:ros-message)
  ((tag
    :reader tag
    :initarg :tag
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetTag-response (<GetTag-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTag-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTag-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tag_utils-srv:<GetTag-response> is deprecated: use tag_utils-srv:GetTag-response instead.")))

(cl:ensure-generic-function 'tag-val :lambda-list '(m))
(cl:defmethod tag-val ((m <GetTag-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tag_utils-srv:tag-val is deprecated.  Use tag_utils-srv:tag instead.")
  (tag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTag-response>) ostream)
  "Serializes a message object of type '<GetTag-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTag-response>) istream)
  "Deserializes a message object of type '<GetTag-response>"
  (cl:setf (cl:slot-value msg 'tag) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'tag)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTag-response>)))
  "Returns string type for a service object of type '<GetTag-response>"
  "tag_utils/GetTagResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTag-response)))
  "Returns string type for a service object of type 'GetTag-response"
  "tag_utils/GetTagResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTag-response>)))
  "Returns md5sum for a message object of type '<GetTag-response>"
  "3cb26f00cb9a158b9a65f3463d45d1c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTag-response)))
  "Returns md5sum for a message object of type 'GetTag-response"
  "3cb26f00cb9a158b9a65f3463d45d1c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTag-response>)))
  "Returns full string definition for message of type '<GetTag-response>"
  (cl:format cl:nil "float32[3] tag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTag-response)))
  "Returns full string definition for message of type 'GetTag-response"
  (cl:format cl:nil "float32[3] tag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTag-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTag-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTag-response
    (cl:cons ':tag (tag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTag)))
  'GetTag-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTag)))
  'GetTag-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTag)))
  "Returns string type for a service object of type '<GetTag>"
  "tag_utils/GetTag")