(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(setq *rs-list* (load-from-seq-pattern-file (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/jaxon-run/tmp")))

(setq *tm-list* (mapcar #'(lambda (rs) (cadr (memq :time rs))) *rs-list*)) 
(setq *av-list* (mapcar #'(lambda (rs) (cadr (memq :angle-vector rs))) *rs-list*)) 
(setq *rc-list* (mapcar #'(lambda (rs) (cadr (memq :root-coords rs))) *rs-list*)) 
(setq *zmp-list* (mapcar #'(lambda (rs) (cadr (memq :zmp rs))) *rs-list*)) 
(setq *wrench-list* (mapcar #'(lambda (rs) (cadr (memq :wrench rs))) *rs-list*)) 
(setq *opt-list* (mapcar #'(lambda (rs) (cadr (memq :contact-state rs))) *rs-list*)) 

(defun contact-state-to-optional-data (contact-state)
  (mapcar
    #'(lambda (state)
        (cond ((equal state :support) (print 1))
              ((equal state :swing) (print 0))
              (t 0)))
   contact-state))

(jaxon_red-init)
(setq *robot* *jaxon_red*)

(print (car (list (car *av-list*))))
 
(send *ri* :angle-vector-sequence-full
  (list (car *av-list*))
  (list (car *tm-list*))
  :root-coords (list (car *rc-list*))
  :zmp (list (car *zmp-list*))
  :wrench (list (car *wrench-list*))
  :optional (contact-state-to-optional-data (list (car *opt-list*)))
  )
(send *ri* wait-interpolation-seq)
