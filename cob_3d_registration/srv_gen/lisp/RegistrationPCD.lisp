; Auto-generated. Do not edit!


(cl:in-package cob_3d_registration-srv)


;//! \htmlinclude RegistrationPCD-request.msg.html

(cl:defclass <RegistrationPCD-request> (roslisp-msg-protocol:ros-message)
  ((pcd_fn
    :reader pcd_fn
    :initarg :pcd_fn
    :type cl:string
    :initform "")
   (img_fn
    :reader img_fn
    :initarg :img_fn
    :type cl:string
    :initform "")
   (depth_img_fn
    :reader depth_img_fn
    :initarg :depth_img_fn
    :type cl:string
    :initform ""))
)

(cl:defclass RegistrationPCD-request (<RegistrationPCD-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegistrationPCD-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegistrationPCD-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<RegistrationPCD-request> is deprecated: use cob_3d_registration-srv:RegistrationPCD-request instead.")))

(cl:ensure-generic-function 'pcd_fn-val :lambda-list '(m))
(cl:defmethod pcd_fn-val ((m <RegistrationPCD-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:pcd_fn-val is deprecated.  Use cob_3d_registration-srv:pcd_fn instead.")
  (pcd_fn m))

(cl:ensure-generic-function 'img_fn-val :lambda-list '(m))
(cl:defmethod img_fn-val ((m <RegistrationPCD-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:img_fn-val is deprecated.  Use cob_3d_registration-srv:img_fn instead.")
  (img_fn m))

(cl:ensure-generic-function 'depth_img_fn-val :lambda-list '(m))
(cl:defmethod depth_img_fn-val ((m <RegistrationPCD-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:depth_img_fn-val is deprecated.  Use cob_3d_registration-srv:depth_img_fn instead.")
  (depth_img_fn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegistrationPCD-request>) ostream)
  "Serializes a message object of type '<RegistrationPCD-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pcd_fn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pcd_fn))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'img_fn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'img_fn))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'depth_img_fn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'depth_img_fn))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegistrationPCD-request>) istream)
  "Deserializes a message object of type '<RegistrationPCD-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pcd_fn) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pcd_fn) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'img_fn) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'img_fn) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_img_fn) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'depth_img_fn) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegistrationPCD-request>)))
  "Returns string type for a service object of type '<RegistrationPCD-request>"
  "cob_3d_registration/RegistrationPCDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegistrationPCD-request)))
  "Returns string type for a service object of type 'RegistrationPCD-request"
  "cob_3d_registration/RegistrationPCDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegistrationPCD-request>)))
  "Returns md5sum for a message object of type '<RegistrationPCD-request>"
  "557fdc13782bfc93783b1523e234caa8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegistrationPCD-request)))
  "Returns md5sum for a message object of type 'RegistrationPCD-request"
  "557fdc13782bfc93783b1523e234caa8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegistrationPCD-request>)))
  "Returns full string definition for message of type '<RegistrationPCD-request>"
  (cl:format cl:nil "string pcd_fn~%string img_fn~%string depth_img_fn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegistrationPCD-request)))
  "Returns full string definition for message of type 'RegistrationPCD-request"
  (cl:format cl:nil "string pcd_fn~%string img_fn~%string depth_img_fn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegistrationPCD-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pcd_fn))
     4 (cl:length (cl:slot-value msg 'img_fn))
     4 (cl:length (cl:slot-value msg 'depth_img_fn))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegistrationPCD-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegistrationPCD-request
    (cl:cons ':pcd_fn (pcd_fn msg))
    (cl:cons ':img_fn (img_fn msg))
    (cl:cons ':depth_img_fn (depth_img_fn msg))
))
;//! \htmlinclude RegistrationPCD-response.msg.html

(cl:defclass <RegistrationPCD-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass RegistrationPCD-response (<RegistrationPCD-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegistrationPCD-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegistrationPCD-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_3d_registration-srv:<RegistrationPCD-response> is deprecated: use cob_3d_registration-srv:RegistrationPCD-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RegistrationPCD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_3d_registration-srv:success-val is deprecated.  Use cob_3d_registration-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegistrationPCD-response>) ostream)
  "Serializes a message object of type '<RegistrationPCD-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegistrationPCD-response>) istream)
  "Deserializes a message object of type '<RegistrationPCD-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegistrationPCD-response>)))
  "Returns string type for a service object of type '<RegistrationPCD-response>"
  "cob_3d_registration/RegistrationPCDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegistrationPCD-response)))
  "Returns string type for a service object of type 'RegistrationPCD-response"
  "cob_3d_registration/RegistrationPCDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegistrationPCD-response>)))
  "Returns md5sum for a message object of type '<RegistrationPCD-response>"
  "557fdc13782bfc93783b1523e234caa8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegistrationPCD-response)))
  "Returns md5sum for a message object of type 'RegistrationPCD-response"
  "557fdc13782bfc93783b1523e234caa8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegistrationPCD-response>)))
  "Returns full string definition for message of type '<RegistrationPCD-response>"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegistrationPCD-response)))
  "Returns full string definition for message of type 'RegistrationPCD-response"
  (cl:format cl:nil "int64 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegistrationPCD-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegistrationPCD-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegistrationPCD-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegistrationPCD)))
  'RegistrationPCD-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegistrationPCD)))
  'RegistrationPCD-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegistrationPCD)))
  "Returns string type for a service object of type '<RegistrationPCD>"
  "cob_3d_registration/RegistrationPCD")