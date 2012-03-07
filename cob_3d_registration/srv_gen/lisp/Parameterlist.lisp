; Auto-generated. Do not edit!


(cl:in-package cob_3d_registration-srv)


;//! \htmlinclude Parameterlist-request.msg.html

(cl:defclass <Parameterlist-request> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (values
    :reader values
    :initarg :values
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (types
    :reader types
    :initarg :types
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Parameterlist-request (<Parameterlist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameterlist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameterlist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<Parameterlist-request> is deprecated: use cob_3d_registration-srv:Parameterlist-request instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <Parameterlist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:names-val is deprecated.  Use cob_3d_registration-srv:names instead.")
  (names m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <Parameterlist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:values-val is deprecated.  Use cob_3d_registration-srv:values instead.")
  (values m))

(cl:ensure-generic-function 'types-val :lambda-list '(m))
(cl:defmethod types-val ((m <Parameterlist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:types-val is deprecated.  Use cob_3d_registration-srv:types instead.")
  (types m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameterlist-request>) ostream)
  "Serializes a message object of type '<Parameterlist-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'values))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'types))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'types))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameterlist-request>) istream)
  "Deserializes a message object of type '<Parameterlist-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'types) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'types)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameterlist-request>)))
  "Returns string type for a service object of type '<Parameterlist-request>"
  "cob_3d_registration/ParameterlistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameterlist-request)))
  "Returns string type for a service object of type 'Parameterlist-request"
  "cob_3d_registration/ParameterlistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameterlist-request>)))
  "Returns md5sum for a message object of type '<Parameterlist-request>"
  "b53ad76a356e707c7b3aa8a4b8e9a778")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameterlist-request)))
  "Returns md5sum for a message object of type 'Parameterlist-request"
  "b53ad76a356e707c7b3aa8a4b8e9a778")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameterlist-request>)))
  "Returns full string definition for message of type '<Parameterlist-request>"
  (cl:format cl:nil "string[] names~%string[] values~%string[] types~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameterlist-request)))
  "Returns full string definition for message of type 'Parameterlist-request"
  (cl:format cl:nil "string[] names~%string[] values~%string[] types~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameterlist-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'types) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameterlist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameterlist-request
    (cl:cons ':names (names msg))
    (cl:cons ':values (values msg))
    (cl:cons ':types (types msg))
))
;//! \htmlinclude Parameterlist-response.msg.html

(cl:defclass <Parameterlist-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass Parameterlist-response (<Parameterlist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameterlist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameterlist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<Parameterlist-response> is deprecated: use cob_3d_registration-srv:Parameterlist-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Parameterlist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:success-val is deprecated.  Use cob_3d_registration-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameterlist-response>) ostream)
  "Serializes a message object of type '<Parameterlist-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameterlist-response>) istream)
  "Deserializes a message object of type '<Parameterlist-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameterlist-response>)))
  "Returns string type for a service object of type '<Parameterlist-response>"
  "cob_3d_registration/ParameterlistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameterlist-response)))
  "Returns string type for a service object of type 'Parameterlist-response"
  "cob_3d_registration/ParameterlistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameterlist-response>)))
  "Returns md5sum for a message object of type '<Parameterlist-response>"
  "b53ad76a356e707c7b3aa8a4b8e9a778")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameterlist-response)))
  "Returns md5sum for a message object of type 'Parameterlist-response"
  "b53ad76a356e707c7b3aa8a4b8e9a778")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameterlist-response>)))
  "Returns full string definition for message of type '<Parameterlist-response>"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameterlist-response)))
  "Returns full string definition for message of type 'Parameterlist-response"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameterlist-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameterlist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameterlist-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Parameterlist)))
  'Parameterlist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Parameterlist)))
  'Parameterlist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameterlist)))
  "Returns string type for a service object of type '<Parameterlist>"
  "cob_3d_registration/Parameterlist")