
(cl:in-package :asdf)

(defsystem "pepper_nodes-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ExecuteJS" :depends-on ("_package_ExecuteJS"))
    (:file "_package_ExecuteJS" :depends-on ("_package"))
    (:file "LoadUrl" :depends-on ("_package_LoadUrl"))
    (:file "_package_LoadUrl" :depends-on ("_package"))
    (:file "Text2Speech" :depends-on ("_package_Text2Speech"))
    (:file "_package_Text2Speech" :depends-on ("_package"))
  ))