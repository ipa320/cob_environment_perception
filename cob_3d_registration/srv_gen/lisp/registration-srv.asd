
(cl:in-package :asdf)

(defsystem "registration-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RegistrationPCD" :depends-on ("_package_RegistrationPCD"))
    (:file "_package_RegistrationPCD" :depends-on ("_package"))
    (:file "EvaluationResult" :depends-on ("_package_EvaluationResult"))
    (:file "_package_EvaluationResult" :depends-on ("_package"))
    (:file "Parameterlist" :depends-on ("_package_Parameterlist"))
    (:file "_package_Parameterlist" :depends-on ("_package"))
    (:file "EvaluationAssumptions" :depends-on ("_package_EvaluationAssumptions"))
    (:file "_package_EvaluationAssumptions" :depends-on ("_package"))
  ))