(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://tennis/euslisp/is-choreonoid.l")


(setq *rs-list* (load-from-seq-pattern-file (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/jaxon-one-step/tmp")))

(setq *first-tm* (cadr (memq :time (car *rs-list*))))
(setq *tm-list* (mapcar #'(lambda (rs) (- (cadr (memq :time rs)) *first-tm*)) *rs-list*))
(setq *av-list-real* (mapcar #'(lambda (rs) (cadr (memq :angle-vector rs))) *rs-list*))
(setq *rc-list* (mapcar #'(lambda (rs) (cadr (memq :root-coords rs))) *rs-list*))
(setq *zmp-list* (mapcar #'(lambda (rs) (cadr (memq :zmp rs))) *rs-list*))
;; (setq *wrench-list* (mapcar #'(lambda (rs) (cadr (memq :wrench-list rs))) *rs-list*))
(setq *cs-list* (mapcar #'(lambda (rs) (cadr (memq :contact-state rs))) *rs-list*))

;; bug included; (rs-listからopt-listは作れないかも)
;; 肢の数 (4)
(setq limb-num (length (car *cs-list*)))

;; *cs-list*が長さ2以上は前提
(when (< (length *cs-list*) 2)
  (warn "*cs-list* length must be more than 1~%")
  (exit))

;; 制御周期
(setq timestep (- (elt *tm-list* 1) (elt *tm-list* 0)))

(setq *max-remain-tm* 1.0)

(defun push-last (target-list target-value)
  (append target-list (list target-value))
  )


(defun make-optional-data-from-cs-list (cs-list)
  ;; 肢ごと，時間ごとの2次元リスト ;; n番目の肢のiステップ目の接触切り替え時間は(elt (elt *略* i) n)でアクセス可能
  (setq *optional-data-remain-time-list* nil)
  (dotimes (limb-idx limb-num)
    ;; :swing :swing :swing :support :supportなら(3, 2)というリストになるように
    (let ((hist nil)
          (last-state (elt (car cs-list) limb-idx))
          (cur-state (elt (car cs-list) limb-idx))
          (cnt 0)
          (remain-tm *max-remain-tm*)
          (opt-remain-each-limb 0)
          (max-remain-step (round (/ *max-remain-tm* timestep))))
      ;; 何連続同じstateかのカウント
      (dotimes (ts-idx (length cs-list))
        (incf cnt)
        (setq cur-state (elt (elt cs-list ts-idx) limb-idx))
        (when (not (equal last-state cur-state))
          (progn
            (setq hist (push-last hist cnt))
            (setq last-state cur-state)
            (setq cnt 0)
            )
          )
        )
      (setq hist (push-last hist cnt))
      ;; ここまででhist完成
      ;; 実際にoptionalに渡すリストのうち，ここのdotimesループ内の肢の分だけ抽出したもの
      ;; 多分1.0が最大からだんだん残り時間が減っていく
      (setq opt-remain-each-limb '())
      (dolist (cnt (butlast hist :n 1)) ;; butlast: listの最後から1つを削除したリスト
        (dotimes (i cnt)
          (progn
            (setq remain-tm (* (- cnt i) timestep))
            (when (>= remain-tm *max-remain-tm*)
              (setq remain-tm *max-remain-tm*)
              )
            (setq opt-remain-each-limb (push-last opt-remain-each-limb remain-tm)))
            )
        )
      (dotimes (i (car (last hist)))
        (setq opt-remain-each-limb (push-last opt-remain-each-limb *max-remain-tm*))
        )
      (setq *optional-data-remain-time-list* (push-last *optional-data-remain-time-list* opt-remain-each-limb))
      )
    )
  (let ((one-step)
        (result '()))
    (dotimes (step (length cs-list))
      (setq one-step
        (concatenate float-vector
          (mapcar
            #'(lambda (state)
                (cond ((equal state :support) 1)
                      ((equal state :swing) 0)
                      (t 0)))
           (elt cs-list step))
          ;; 右脚を取り出し，次にタイムを取り出す
          (float-vector (elt (elt *optional-data-remain-time-list* 0) step) ; right remain time
                        (elt (elt *optional-data-remain-time-list* 1) step)) ; left remain time
          (float-vector 1 1)
           )
         )
      (setq result (push-last result one-step))
       )
    result
     )
  )

(setq *is-choreonoid* (is-choreonoid))
(setq *is-choreonoid* t)

(when *is-choreonoid*
  (format t "this is choreonoid~%"))

;; ハンドの関節角度を追加
(if *is-choreonoid*
  (progn
    (setq *av-list-choreonoid* '())
    (dolist (av *av-list-real*)
      (setq *av-list-choreonoid*
        (push-last *av-list-choreonoid*
                   (concatenate float-vector av (float-vector (deg2rad -80) (deg2rad 80) (deg2rad -80) (deg2rad 80)))))
      )
    (setq *av-list* *av-list-choreonoid*)
    )
  ;; else
  (setq *av-list* *av-list-real*)
  )

(setq *opt-list* (make-optional-data-from-cs-list *cs-list*))

(print (list (car *tm-list*)))
(print (list (car *av-list*)))
(print (list (car *rc-list*)))
(print (list (car *zmp-list*)))
;; (print (list (car *wrench-list*)))
(print (list (car *opt-list*)))
(jaxon_red-init)
(setq *robot* *jaxon_red*)
; (send *ri* :load-pattern (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/jaxon-one-step/tmp"))
(print (car *av-list*))
; これはchoreonoidかどうかによらない
(send *ri* :angle-vector (car *av-list-real*) 5000)
(send *ri* :wait-interpolation)
;(send *ri* :angle-vector-sequence-full
;  (list (car *av-list*))
;  (list (car *tm-list*))
;  :root-coords (list (car *rc-list*))
;  :zmp (list (car *zmp-list*))
;  ;; :wrench (list (car *wrench-list*))
;  :optional (list (car *opt-list*))
;  )
;(send *ri* :wait-interpolation-seq)

(format t "press enter to exec whole motion ~%")
(read-line)
; (print *av-list*)
(send *ri* :angle-vector-sequence-full
  *av-list*
  *tm-list*
  :root-coords *rc-list*
  :zmp *zmp-list*
  ;; :wrench *wrench-list*
  :optional *opt-list*
  )
(send *ri* :wait-interpolation-seq)
