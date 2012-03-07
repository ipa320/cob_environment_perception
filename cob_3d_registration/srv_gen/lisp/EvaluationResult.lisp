; Auto-generated. Do not edit!


(cl:in-package cob_3d_registration-srv)


;//! \htmlinclude EvaluationResult-request.msg.html

(cl:defclass <EvaluationResult-request> (roslisp-msg-protocol:ros-message)
  ((duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (memory
    :reader memory
    :initarg :memory
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil)
   (transformation
    :reader transformation
    :initarg :transformation
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass EvaluationResult-request (<EvaluationResult-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvaluationResult-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvaluationResult-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<EvaluationResult-request> is deprecated: use cob_3d_registration-srv:EvaluationResult-request instead.")))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <EvaluationResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:duration-val is deprecated.  Use cob_3d_registration-srv:duration instead.")
  (duration m))

(cl:ensure-generic-function 'memory-val :lambda-list '(m))
(cl:defmethod memory-val ((m <EvaluationResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:memory-val is deprecated.  Use cob_3d_registration-srv:memory instead.")
  (memory m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <EvaluationResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:state-val is deprecated.  Use cob_3d_registration-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'transformation-val :lambda-list '(m))
(cl:defmethod transformation-val ((m <EvaluationResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:transformation-val is deprecated.  Use cob_3d_registration-srv:transformation instead.")
  (transformation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvaluationResult-request>) ostream)
  "Serializes a message object of type '<EvaluationResult-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'memory)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvaluationResult-request>) istream)
  "Deserializes a message object of type '<EvaluationResult-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'memory) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvaluationResult-request>)))
  "Returns string type for a service object of type '<EvaluationResult-request>"
  "cob_3d_registration/EvaluationResultRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationResult-request)))
  "Returns string type for a service object of type 'EvaluationResult-request"
  "cob_3d_registration/EvaluationResultRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvaluationResult-request>)))
  "Returns md5sum for a message object of type '<EvaluationResult-request>"
  "a6e0d8a505a344b10a76bd984ce24d22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvaluationResult-request)))
  "Returns md5sum for a message object of type 'EvaluationResult-request"
  "a6e0d8a505a344b10a76bd984ce24d22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvaluationResult-request>)))
  "Returns full string definition for message of type '<EvaluationResult-request>"
  (cl:format cl:nil "float64 duration~%int64 memory~%bool state~%float64[] transformation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvaluationResult-request)))
  "Returns full string definition for message of type 'EvaluationResult-request"
  (cl:format cl:nil "float64 duration~%int64 memory~%bool state~%float64[] transformation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvaluationResult-request>))
  (cl:+ 0
     8
     8
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'transformation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvaluationResult-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EvaluationResult-request
    (cl:cons ':duration (duration msg))
    (cl:cons ':memory (memory msg))
    (cl:cons ':state (state msg))
    (cl:cons ':transformation (transformation msg))
))
;//! \htmlinclude EvaluationResult-response.msg.html

(cl:defclass <EvaluationResult-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass EvaluationResult-response (<EvaluationResult-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvaluationResult-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvaluationResult-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<EvaluationResult-response> is deprecated: use cob_3d_registration-srv:EvaluationResult-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EvaluationResult-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:success-val is deprecated.  Use cob_3d_registration-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvaluationResult-response>) ostream)
  "Serializes a message object of type '<EvaluationResult-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvaluationResult-response>) istream)
  "Deserializes a message object of type '<EvaluationResult-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvaluationResult-response>)))
  "Returns string type for a service object of type '<EvaluationResult-response>"
  "cob_3d_registration/EvaluationResultResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationResult-response)))
  "Returns string type for a service object of type 'EvaluationResult-response"
  "cob_3d_registration/EvaluationResultResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvaluationResult-response>)))
  "Returns md5sum for a message object of type '<EvaluationResult-response>"
  "a6e0d8a505a344b10a76bd984ce24d22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvaluationResult-response)))
  "Returns md5sum for a message object of type 'EvaluationResult-response"
  "a6e0d8a505a344b10a76bd984ce24d22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvaluationResult-response>)))
  "Returns full string definition for message of type '<EvaluationResult-response>"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvaluationResult-response)))
  "Returns full string definition for message of type 'EvaluationResult-response"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvaluationResult-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvaluationResult-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EvaluationResult-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EvaluationResult)))
  'EvaluationResult-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EvaluationResult)))
  'EvaluationResult-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvaluationResult)))
  "Returns string type for a service object of type '<EvaluationResult>"
  "cob_3d_registration/EvaluationResult")