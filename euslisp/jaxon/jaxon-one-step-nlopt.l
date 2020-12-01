(load "./sample-sqp-optimization-instant.l")
(load "package://eus_qp/optmotiongen/euslisp/bspline-dynamic-configuration-task.l")
(load "package://tennis/euslisp/robot-interface.l")
(load "package://eus_nlopt/euslisp/nlopt")

(setq *joint-num-control-point* 10)
(setq *centroid-num-control-point* 20)
(setq *max-force* 2000)
(setq *max-angular-moment* 1.0)
(setq *min-force* (* -1 *max-force*))
(setq *min-angular-moment* (* -1 *max-angular-moment*))

(defun make-bspline-control-point-boundary-from-min-or-max-vector (min-or-max-vector num-control-point)
  (reduce #'(lambda (lhs rhs)
             (flatten (concatenate cons lhs (make-list num-control-point :initial-element rhs))))
    (list-insert nil 0 (coerce min-or-max-vector cons))
    )
  )

(defun jaxon-one-step
    (&key
     ;; time
     (start-time 0.0)
     (finish-time 1.0)
     (takeoff0-time 0.3)
     (landing0-time 0.7)
     ;; motion
     (landing0-pos (float-vector 400 200 0))
     ;; task-time-divide-num
     (kin-task-time-divide-num 20)
     (eom-task-time-divide-num 20)
     (centroid-task-time-divide-num 20)
     (posture-task-time-divide-num 5)
     ;; bspline setting
     (joint-num-control-point *joint-num-control-point*)
     (centroid-num-control-point *centroid-num-control-point*)
     (bspline-order 4)
     (cog-bspline-order 5)
     ;; other setting
     (optimize-torque? nil)
     (avoid-collision? nil)
     (graph-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/jaxon-one-step/graph.pdf"))
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/jaxon-one-step/tmp"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup arrow
  (setq *arrow-rleg* (instance arrow-object :init))
  (send *arrow-rleg* :newcoords
        (make-coords :pos (float-vector 0 -305 0) :rpy (list -0.5 0 0)))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 305 0) :rpy (list 0.5 0 0)))
  (setq *arrow-lleg-landing0* (instance arrow-object :init))
  (send *arrow-lleg-landing0* :newcoords
        (send (send *arrow-lleg* :copy-worldcoords) :translate landing0-pos))
  (setq *arrow-list*
        (list *arrow-rleg* *arrow-lleg* *arrow-lleg-landing0*))
  ;; setup robot
  (setup-jaxon_red-for-sample-sqp-optimization)
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*))
  (send *robot-env* :put :max-angular-moment (float-vector (min *max-angular-moment* 1.0) (min *max-angular-moment* 1.0) (min *max-angular-moment* 1.0)))
  (send *robot-env* :put :arrow-rleg *arrow-rleg*)
  (send *robot-env* :put :arrow-lleg *arrow-lleg*)
  (send *robot-env* :put :arrow-lleg-landing0 *arrow-lleg-landing0*)
  (send *robot-env* :put :landing0-time landing0-time)
  (send *robot-env* :put :takeoff0-time takeoff0-time)
  ;; setup contact constraint
  (send *robot-env* :put :rleg-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.7
                  :mu-rot 0.001
                  :max-fz (min *max-force* 2000)
                  :support-polygon (send *robot-env* :robot :get :rleg_link5-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :put :lleg-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.7
                  :mu-rot 0.001
                  :max-fz (min *max-force* 2000)
                  :support-polygon (send *robot-env* :robot :get :lleg_link5-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :robot :reset-pose)
  (send *robot-env* :robot :legs :move-end-pos (float-vector 0 0 200) :world)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot-env*) *arrow-list* (if avoid-collision? (list *obst*))))

  (setq *bspline-dynamic-config-task*
        (instance bspline-dynamic-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if optimize-torque?
                      (send *robot-env* :drive-joint-list) nil)
                  :posture-joint-list
                  (subseq (send *robot-env* :joint-list) 6)
                  :posture-joint-angle-list
                  (subseq (send-all (send *robot-env* :joint-list) :joint-angle) 6)
                  :kin-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- kin-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range kin-task-time-divide-num))
                  :eom-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- eom-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range eom-task-time-divide-num))
                  :centroid-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- centroid-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range centroid-task-time-divide-num))
                  :posture-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- posture-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range posture-task-time-divide-num))
                  :theta-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point joint-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config (length (send *robot-env* :joint-list))
                            :stationery-start-finish-regular-scale 1e2
                            :diff-square-integration-regular-scale 1e-2
                            )
                  :cog-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order cog-bspline-order
                            :dim-instant-config 3
                            :stationery-start-finish-regular-scale 1e6
                            :diff-square-integration-regular-scale 10.0
                            )
                  :ang-moment-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config 3
                            :stationery-start-finish-regular-scale 1e6
                            :zero-diff-stationery-start-finish-regular-scale 1e6
                            :diff-square-integration-regular-scale 1.0
                            )
                  :wrench-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config (* 2 6)
                            :stationery-start-finish-regular-scale 1e-5
                            :diff-square-integration-regular-scale 1e-5
                            )
                  :torque-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point joint-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config 0
                            )
                  :kin-target-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :takeoff0-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg) :copy-worldcoords)
                                   ))
                            ((< tm (send robot-env :get :landing0-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)))
                            (t
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg-landing0) :copy-worldcoords))))
                      )
                  :kin-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :takeoff0-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing0-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords))))
                      )
                  :contact-target-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :takeoff0-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg) :copy-worldcoords)
                                   ))
                            ((< tm (send robot-env :get :landing0-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   nil
                                   ))
                            (t
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg) :copy-worldcoords)
                                   )))
                      )
                  :contact-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :takeoff0-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing0-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   nil
                                   ))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   )))
                      )
                  :contact-constraint-list-func
                  #'(lambda (robot-env tm)
                      (list (send robot-env :get :rleg-contact-constraint)
                            (send robot-env :get :lleg-contact-constraint)
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                  :kin-task-scale 1e1
                  :cog-task-scale 1e3
                  :ang-moment-task-scale 0.0
                  :posture-task-scale 5e-1
                  :norm-regular-scale-max 1e-4
                  :norm-regular-scale-offset 1e-7
                  ))

  (setq *initial-state* (send *bspline-dynamic-config-task* :config-vector))
  (setq *state-dimension* (send *bspline-dynamic-config-task* :dim-config))
  (setq *equality-dimension* (length (send *bspline-dynamic-config-task* :config-equality-constraint-vector)))
  (setq *inequality-dimension* (length (send *bspline-dynamic-config-task* :config-inequality-constraint-vector)))
  (setq *state-min-vector*
    (concatenate float-vector
      (make-bspline-control-point-boundary-from-min-or-max-vector (send *bspline-dynamic-config-task* :theta-min-vector) *joint-num-control-point*)
      (make-bspline-control-point-boundary-from-min-or-max-vector (scale -1 (send *bspline-dynamic-config-task* :cog-max-vector)) *centroid-num-control-point*)
      (make-bspline-control-point-boundary-from-min-or-max-vector (scale -1 (send *bspline-dynamic-config-task* :ang-moment-max-vector)) *centroid-num-control-point*)
      ;; wrench-min-vector ;; FIXME
      (make-bspline-control-point-boundary-from-min-or-max-vector (concatenate float-vector (float-vector *min-force* *min-force* *min-force* *min-angular-moment* *min-angular-moment* *min-angular-moment*) (float-vector *min-force* *min-force* *min-force* *min-angular-moment* *min-angular-moment* *min-angular-moment*))
                                                                  *centroid-num-control-point*)
      ;; torque-min-vector ;; manual says todo; zero dim
      ;; phi-min-vector
      (send *bspline-dynamic-config-task* :phi-min-vector)
      ))
  (setq *state-max-vector*
    (concatenate float-vector
      (make-bspline-control-point-boundary-from-min-or-max-vector (send *bspline-dynamic-config-task* :theta-max-vector) *joint-num-control-point*)
      (make-bspline-control-point-boundary-from-min-or-max-vector (scale 1 (send *bspline-dynamic-config-task* :cog-max-vector)) *centroid-num-control-point*)
      (make-bspline-control-point-boundary-from-min-or-max-vector (scale 1 (send *bspline-dynamic-config-task* :ang-moment-max-vector)) *centroid-num-control-point*)
      ;; wrench-max-vector ;; FIXME
      (make-bspline-control-point-boundary-from-min-or-max-vector (concatenate float-vector (float-vector *max-force* *max-force* *max-force* *max-angular-moment* *max-angular-moment* *max-angular-moment*) (float-vector *max-force* *max-force* *max-force* *max-angular-moment* *max-angular-moment* *max-angular-moment*))
                                                                  *centroid-num-control-point*)
      ;; torque-max-vector ;; manual says todo; zero dim
      ;; phi-max-vector
      (send *bspline-dynamic-config-task* :phi-max-vector)
      ))
  (setq *limit-margin* 1e-3)
  (dotimes (i *state-dimension*)
    (when (< (elt *initial-state* i) (elt *state-min-vector* i))
      (setf (aref *initial-state* i) (+ (elt *state-min-vector* i) *limit-margin*))
      )
    (when (> (elt *initial-state* i) (elt *state-max-vector* i))
      (setf (aref *initial-state* i) (- (elt *state-max-vector* i) *limit-margin*))
      )
    )
  (nlopt-optimize
    :initial-state *initial-state*
    :state-min-vector *state-min-vector*
    :state-max-vector *state-max-vector*
    :state-dimension *state-dimension*
    :evaluation-function (read-from-string "optmotiongen-f")
    :evaluation-function-gradient (read-from-string "optmotiongen-df")
    :equality-function (read-from-string "optmotiongen-g")
    :equality-function-gradient (read-from-string "optmotiongen-dg")
    :inequality-function (read-from-string "optmotiongen-h")
    :inequality-function-gradient (read-from-string "optmotiongen-dh")
    :equality-dimension *equality-dimension*
    :inequality-dimension *inequality-dimension*
    :alg SLSQP
    :ftol 1e-16
    :xtol 1e-16
    :eqthre -1e-320
    :max-eval 10
    :max-time (* 24 60 60)
    )

  (when graph-filename
    (send *bspline-dynamic-config-task* :generate-graph :graph-filename graph-filename))

  (when output-filename
    (unix:system
     (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" output-filename))
    (setq *rs-list*
          (send *bspline-dynamic-config-task* :generate-robot-state-list
                :robot-env *robot-env*
                :joint-name-list (send-all (send *robot-env* :robot :joint-list-orig) :name)
                :root-link-name (send (car (send *robot-env* :robot :links)) :name)
                :step-time 0.004
                ))
    (play-robot-state-list
     :rs-list *rs-list*
     :robot-env *robot-env*
     :joint-name-list (send-all (send *robot-env* :robot :joint-list-orig) :name)
     :root-link-name (send (car (send *robot-env* :robot :links)) :name)
     )
    (dump-loadable-structure (format nil "~a.l" output-filename) *rs-list*)
    (dump-seq-pattern-file
     *rs-list*
     output-filename
     :robot (send *robot-env* :robot)
     ))

  (send *bspline-dynamic-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(jaxon-one-step :optimize-torque? t)~%")


(defun v1-to-float-vector (v1 v1-dim)
  (setq config-vector (instantiate float-vector v1-dim))
  (dotimes (i v1-dim) (setf (aref config-vector i) (aref v1 i)))
  config-vector
  )

(defun-c-callable optmotiongen-f ((v1 :string) (v2 :string)) :integer
  (send *bspline-dynamic-config-task* :set-config (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  ;; (send *bspline-dynamic-config-task* :set-config v1) ; is this enough?
  (setq task-value (send *bspline-dynamic-config-task* :task-value))
  (setq eval-value (/ (v. task-value task-value) 2.0))
  (setf (aref v2 0) eval-value)
  ;; (format t "f ~A~%" eval-value)
  (send *irtviewer* :draw-objects)
  0
  )

(defun-c-callable optmotiongen-df ((v1 :string) (v2 :string)) :integer
  (send *bspline-dynamic-config-task* :set-config (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  (setq task-value (send *bspline-dynamic-config-task* :task-value))
  (setq task-jacobian (send *bspline-dynamic-config-task* :task-jacobian))
  (setq deval-value (transform (transpose task-jacobian) task-value))
  (dotimes (i (length deval-value))
    (setf (aref v2 i) (elt deval-value i))
    )
  ;; (format t "df ~A~%" deval-value)
  0
  )

(defun-c-callable optmotiongen-g ((v1 :string) (v2 :string)) :integer
  (setq v1-float-vector (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  (send *bspline-dynamic-config-task* :set-config v1-float-vector)
  (setq eq-vector (send *bspline-dynamic-config-task* :config-equality-constraint-vector))
  (setq eq-value eq-vector)
  (dotimes (i (length eq-value))
    (setf (aref v2 i) (elt eq-value i))
    )
  ;; (format t "g ~A~%" eq-value)
  0
  )

(defun-c-callable optmotiongen-dg ((v1 :string) (v2 :string)) :integer
  (send *bspline-dynamic-config-task* :set-config (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  (setq eq-matrix (send *bspline-dynamic-config-task* :config-equality-constraint-matrix))
  ;; linear constraints only
  (setq eq-jacobian eq-matrix)
  (setq eq-jacobian-flat (scale -1 (array-entity eq-jacobian)))
  (dotimes (i (length eq-jacobian-flat))
    (setf (aref v2 i) (elt eq-jacobian-flat i))
    )
  ;; (format t "dg ~A~%" eq-jacobian-flat)
  0
  )

(defun-c-callable optmotiongen-h ((v1 :string) (v2 :string)) :integer
  (setq v1-float-vector (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  (send *bspline-dynamic-config-task* :set-config v1-float-vector)
  (send *bspline-dynamic-config-task* :update-collision-inequality-constraint)
  (setq ieq-vector (send *bspline-dynamic-config-task* :config-inequality-constraint-vector))
  (setq ieq-value ieq-vector)
  (dotimes (i (length ieq-value))
    (setf (aref v2 i) (elt ieq-value i))
    )
  ;; (format t "h ~A~%" ieq-value)
  0
  )

(defun-c-callable optmotiongen-dh ((v1 :string) (v2 :string)) :integer
  (send *bspline-dynamic-config-task* :set-config (v1-to-float-vector v1 (send *bspline-dynamic-config-task* :dim-config)))
  (send *bspline-dynamic-config-task* :update-collision-inequality-constraint)
  (setq ieq-matrix (send *bspline-dynamic-config-task* :config-inequality-constraint-matrix))
  ;; linear constraints only
  (setq ieq-jacobian ieq-matrix)
  (setq ieq-jacobian-flat (scale -1 (array-entity ieq-jacobian)))
  (dotimes (i (length ieq-jacobian-flat))
    (setf (aref v2 i) (elt ieq-jacobian-flat i))
    )
  ;; (format t "dh ~A~%" ieq-jacobian-flat)
  0
  )

(jaxon-one-step :optimize-torque? t)
