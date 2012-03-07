; Auto-generated. Do not edit!


(cl:in-package cob_3d_registration-srv)


;//! \htmlinclude EvaluationAssumptions-request.msg.html

(cl:defclass <EvaluationAssumptions-request> (roslisp-msg-protocol:ros-message)
  ((transformation
    :reader transformation
    :initarg :transformation
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass EvaluationAssumptions-request (<EvaluationAssumptions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvaluationAssumptions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvaluationAssumptions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<EvaluationAssumptions-request> is deprecated: use cob_3d_registration-srv:EvaluationAssumptions-request instead.")))

(cl:ensure-generic-function 'transformation-val :lambda-list '(m))
(cl:defmethod transformation-val ((m <EvaluationAssumptions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:transformation-val is deprecated.  Use cob_3d_registration-srv:transformation instead.")
  (transformation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvaluationAssumptions-request>) ostream)
  "Serializes a message object of type '<EvaluationAssumptions-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'transformation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'transformation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvaluationAssumptions-request>) istream)
  "Deserializes a message object of type '<EvaluationAssumptions-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'transformation) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'transformation)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvaluationAssumptions-request>)))
  "Returns string type for a service object of type '<EvaluationAssumptions-request>"
  "cob_3d_registration/EvaluationAssumptionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationAssumptions-request)))
  "Returns string type for a service object of type 'EvaluationAssumptions-request"
  "cob_3d_registration/EvaluationAssumptionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvaluationAssumptions-request>)))
  "Returns md5sum for a message object of type '<EvaluationAssumptions-request>"
  "626236ec60cdef9acadc5e978b9f804c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvaluationAssumptions-request)))
  "Returns md5sum for a message object of type 'EvaluationAssumptions-request"
  "626236ec60cdef9acadc5e978b9f804c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvaluationAssumptions-request>)))
  "Returns full string definition for message of type '<EvaluationAssumptions-request>"
  (cl:format cl:nil "float64[] transformation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvaluationAssumptions-request)))
  "Returns full string definition for message of type 'EvaluationAssumptions-request"
  (cl:format cl:nil "float64[] transformation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvaluationAssumptions-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'transformation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvaluationAssumptions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EvaluationAssumptions-request
    (cl:cons ':transformation (transformation msg))
))
;//! \htmlinclude EvaluationAssumptions-response.msg.html

(cl:defclass <EvaluationAssumptions-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass EvaluationAssumptions-response (<EvaluationAssumptions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvaluationAssumptions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvaluationAssumptions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<EvaluationAssumptions-response> is deprecated: use cob_3d_registration-srv:EvaluationAssumptions-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EvaluationAssumptions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:success-val is deprecated.  Use cob_3d_registration-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvaluationAssumptions-response>) ostream)
  "Serializes a message object of type '<EvaluationAssumptions-response>"
  (cl:let* ((signed (cl:slot-value msg 'success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvaluationAssumptions-response>) istream)
  "Deserializes a message object of type '<EvaluationAssumptions-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvaluationAssumptions-response>)))
  "Returns string type for a service object of type '<EvaluationAssumptions-response>"
  "cob_3d_registration/EvaluationAssumptionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationAssumptions-response)))
  "Returns string type for a service object of type 'EvaluationAssumptions-response"
  "cob_3d_registration/EvaluationAssumptionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvaluationAssumptions-response>)))
  "Returns md5sum for a message object of type '<EvaluationAssumptions-response>"
  "626236ec60cdef9acadc5e978b9f804c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvaluationAssumptions-response)))
  "Returns md5sum for a message object of type 'EvaluationAssumptions-response"
  "626236ec60cdef9acadc5e978b9f804c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvaluationAssumptions-response>)))
  "Returns full string definition for message of type '<EvaluationAssumptions-response>"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvaluationAssumptions-response)))
  "Returns full string definition for message of type 'EvaluationAssumptions-response"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvaluationAssumptions-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvaluationAssumptions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EvaluationAssumptions-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EvaluationAssumptions)))
  'EvaluationAssumptions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EvaluationAssumptions)))
  'EvaluationAssumptions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationAssumptions)))
  "Returns string type for a service object of type '<EvaluationAssumptions>"
  "cob_3d_registration/EvaluationAssumptions")