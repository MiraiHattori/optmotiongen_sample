#!/usr/bin/env roseus
(require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper")
(require "package://eus_qp/optmotiongen/euslisp/discrete-kinematics-configuration-task")
(require "irteus/demo/sample-arm-model")
(require "models/arrow-object")
(require "package://eus_nlopt/euslisp/nlopt")


(defun sample-arm-reach-ik-raw
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send *arrow* :copy-worldcoords))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :end-coords))
                  ))
  (setq *initial-state* (send *instant-config-task* :variant-config-vector))
  (setq *state-dimension* (send *instant-config-task* :dim-variant-config))
  (setq *equality-dimension* (length (send *instant-config-task* :variant-config-equality-constraint-vector)))
  (setq *inequality-dimension* (length (send *instant-config-task* :variant-config-inequality-constraint-vector)))
  (setq *state-min-vector*
    (concatenate float-vector
      (send *instant-config-task* :theta-min-vector)
      (coerce (make-list (length (send *instant-config-task* :wrench)) :initial-element -1000) float-vector)
      (send *instant-config-task* :torque-min-vector)
      ))
  (setq *state-max-vector*
    (concatenate float-vector
      (send *instant-config-task* :theta-max-vector)
      (coerce (make-list (length (send *instant-config-task* :wrench)) :initial-element 1000) float-vector)
      (send *instant-config-task* :torque-max-vector)
      ))
  ;; truncate *initial-state*
  (setq *limit-margin* 1e-3)
  (dotimes (i *state-dimension*)
    (when (< (elt *initial-state* i) (elt *state-min-vector* i))
      (setf (aref *initial-state* i) (+ (elt *state-min-vector* i) *limit-margin*))
      )
    (when (> (elt *initial-state* i) (elt *state-max-vector* i))
      (setf (aref *initial-state* i) (- (elt *state-max-vector* i) *limit-margin*))
      )
    )
  (format t "~A~%" *state-min-vector*)
  (format t "~A~%" *state-max-vector*)

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
    :max-eval 1000000
    :max-time (* 24 60 60)
    )

  t)
(warn "(sample-arm-reach-ik-raw)~%")

(defun v1-to-float-vector (v1 v1-dim)
  (setq variant-config-vector (instantiate float-vector v1-dim))
  (dotimes (i v1-dim) (setf (aref variant-config-vector i) (aref v1 i)))
  variant-config-vector
  )

(defun-c-callable optmotiongen-f ((v1 :string) (v2 :string)) :integer
  (send *instant-config-task* :set-variant-config (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  ;; (send *instant-config-task* :set-variant-config v1) ; is this enough?
  (setq task-value (send *instant-config-task* :task-value))
  (setq eval-value (/ (v. task-value task-value) 2.0))
  (setf (aref v2 0) eval-value)
  ;; (format t "f ~A~%" eval-value)
  (send *irtviewer* :draw-objects)
  0
  )

(defun-c-callable optmotiongen-df ((v1 :string) (v2 :string)) :integer
  (send *instant-config-task* :set-variant-config (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  (setq task-value (send *instant-config-task* :task-value))
  (setq variant-task-jacobian (send *instant-config-task* :variant-task-jacobian))
  (setq deval-value (transform (transpose variant-task-jacobian) task-value))
  (dotimes (i (length deval-value))
    (setf (aref v2 i) (elt deval-value i))
    )
  ;; (format t "df ~A~%" deval-value)
  0
  )

(defun-c-callable optmotiongen-g ((v1 :string) (v2 :string)) :integer
  (setq v1-float-vector (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  (send *instant-config-task* :set-variant-config v1-float-vector)
  (setq variant-eq-vector (send *instant-config-task* :variant-config-equality-constraint-vector))
  (setq variant-eq-value variant-eq-vector)
  (dotimes (i (length variant-eq-value))
    (setf (aref v2 i) (elt variant-eq-value i))
    )
  ;; (format t "g ~A~%" variant-eq-value)
  0
  )

(defun-c-callable optmotiongen-dg ((v1 :string) (v2 :string)) :integer
  (send *instant-config-task* :set-variant-config (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  (setq variant-eq-matrix (send *instant-config-task* :variant-config-equality-constraint-matrix))
  ;; linear constraints only
  (setq eq-jacobian variant-eq-matrix)
  (setq eq-jacobian-flat (scale -1 (array-entity variant-eq-jacobian)))
  (dotimes (i (length eq-jacobian-flat))
    (setf (aref v2 i) (elt eq-jacobian-flat i))
    )
  ;; (format t "dg ~A~%" eq-jacobian-flat)
  0
  )

(defun-c-callable optmotiongen-h ((v1 :string) (v2 :string)) :integer
  (setq v1-float-vector (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  (send *instant-config-task* :set-variant-config v1-float-vector)
  (send *instant-config-task* :update-collision-inequality-constraint)
  (setq variant-ieq-vector (send *instant-config-task* :variant-config-inequality-constraint-vector))
  (setq ieq-value variant-ieq-vector)
  (dotimes (i (length ieq-value))
    (setf (aref v2 i) (elt ieq-value i))
    )
  ;; (format t "h ~A~%" ieq-value)
  0
  )

(defun-c-callable optmotiongen-dh ((v1 :string) (v2 :string)) :integer
  (send *instant-config-task* :set-variant-config (v1-to-float-vector v1 (send *instant-config-task* :dim-variant-config)))
  (send *instant-config-task* :update-collision-inequality-constraint)
  (setq variant-ieq-matrix (send *instant-config-task* :variant-config-inequality-constraint-matrix))
  ;; linear constraints only
  (setq ieq-jacobian variant-ieq-matrix)
  (setq ieq-jacobian-flat (scale -1 (array-entity ieq-jacobian)))
  (dotimes (i (length ieq-jacobian-flat))
    (setf (aref v2 i) (elt ieq-jacobian-flat i))
    )
  ;; (format t "dh ~A~%" ieq-jacobian-flat)
  0
  )


(sample-arm-reach-ik-raw)
