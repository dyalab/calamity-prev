(in-package :robray)


(setf (ros-package-path)
      (merge-pathnames "ros_ws/src/franka_ros/"
                       (user-homedir-pathname)))



(defparameter *panda*
  (load-scene-file "package://franka_description/robots/panda_arm_hand.urdf"))

(defparameter *g* (/ (+ 1 (sqrt 5d0)) 2))

(defparameter *l* 8e-2)

(defparameter *spine-height* 1.75)
(defparameter *shoulder-height* 1.47)


(defparameter *plate-thickness* .1e-2)

(defparameter *shoulder-top* (- *spine-height* *shoulder-height*))

(defparameter *shoulder-len* (* *shoulder-top* *g*))

(defparameter *spine-off* (* *shoulder-top* *g* .5))




(defun atanvec (v)
  (atan (elt v 0)
        (elt v 1)))

(defparameter *back* .5)



(defparameter *back-height* (* *back* *g*))

(defparameter *back-len* (+ (vec-norm  (vec *back* *back-height*))
                            0e-2))
(defparameter *back-angle* (atan *back* *back-height*))


(defparameter *top-vec* (vec *shoulder-len* (- *spine-height* *shoulder-height*)))
(defparameter *top-len* (- (vec-norm *top-vec*) *l*))
(defparameter *top-angle* (atanvec *top-vec*))


(defparameter *shoulder-strut-vec* (vec *spine-off* (* *g* *spine-off*)))
(defparameter *shoulder-strut-len* (- (vec-norm *shoulder-strut-vec*) 0))
(defparameter *shoulder-strut-angle* (atanvec *shoulder-strut-vec*))

(defparameter *frame-options* (draw-options-default :color '(.5 .5 .5)))

(defun albox (x y z)
  (scene-geometry-box *frame-options*
                      (vec x y z)))
(defun albox-z (z)
  (scene-geometry-box *frame-options*
                      (vec *l* *l* z)))
(defun z-strut (parent name len)
  (scene-frame-fixed parent name
                     :tf (tf* nil (vec 0 0 (* .5 len)))
                     :geometry (albox-z (abs len))))

(defparameter *frame*
  (scene-graph

   (scene-frame-fixed nil "foot")

   (scene-frame-fixed "foot" "plate"
                      :tf (tf* nil nil)
                      :geometry (albox 2 2 *plate-thickness*))

   (scene-frame-fixed "foot" "spine "
                      :tf (tf* nil (vec 0 0 (* .5 *shoulder-height*)))
                      :geometry (albox *l* *l* *shoulder-height*))

   ;; (scene-frame-fixed "foot" "spine-left"
   ;;                    :tf (tf* nil (vec 0 *spine-off* (* .5 *spine-height*)))
   ;;                    :geometry (albox *l* *l* *spine-height*))
   ;; (scene-frame-fixed "foot" "spine-right"
   ;;                    :tf (tf* nil (vec 0 (- *spine-off*) (* .5 *spine-height*)))
   ;;                    :geometry (albox *l* *l* *spine-height*))

   (scene-frame-fixed "foot" "cross-top"
                      :tf (tf* nil (vec 0 0 (- *spine-height* (* .5 *l*))))
                      :geometry (albox *l*
                                       (+ (* 2 *spine-off*) *l*)
                                       *l*))

   (scene-frame-fixed "foot" "cross-mid"
                      :tf (tf* nil (vec 0 0  *shoulder-height* ))
                      :geometry (albox *l*
                                       (+ (* 2 *spine-off*) *l*)
                                       *l*))

   (scene-frame-fixed "foot" "shoulder-left"
                      :tf (tf* nil (vec (* .5 *shoulder-len*) *spine-off* *shoulder-height*))
                      :geometry (albox *shoulder-len* *l* *l*))
   (scene-frame-fixed "foot" "shoulder-right"
                      :tf (tf* nil (vec (* .5 *shoulder-len*) (- *spine-off*) *shoulder-height*))
                      :geometry (albox *shoulder-len* *l* *l*))

   (scene-frame-fixed "foot" "shoulder-top"
                      :tf (tf* nil (vec (- *shoulder-len* (* .5 *l*)) 0 *shoulder-height* ))
                      :geometry (albox *l*
                                       (- (* 2 *spine-off*) *l*)
                                       *l*))

   (scene-frame-fixed "foot" "shoulder-vert-left"
                      :tf (tf* nil (vec 0 *spine-off* (+ *shoulder-height*
                                                         (* .5 *shoulder-top*))))
                      :geometry (albox *l* *l* *shoulder-top*))

   (scene-frame-fixed "foot" "shoulder-vert-right"
                      :tf (tf* nil (vec 0 (- *spine-off*) (+ *shoulder-height*
                                                             (* .5 *shoulder-top*))))
                      :geometry (albox *l* *l* *shoulder-top*))


   (scene-frame-fixed "foot" "back-foot"
                      :tf (tf* (y-angle *back-angle*) (vec (- *back*) 0 0)))

   (scene-frame-fixed "foot" "left-foot"
                      :tf (tf* (x-angle *back-angle*) (vec 0  *back*  0)))

   (scene-frame-fixed "foot" "right-foot"
                      :tf (tf* (x-angle (- *back-angle*)) (vec 0  (- *back*)  0)))

   ;; (scene-frame-fixed "back-foot" "back-left"
   ;;                    :tf (tf* nil (vec 0 *spine-off*  (* .495 *back-len*)))
   ;;                    :geometry (albox *l* *l* *back-len*))

   ;; (scene-frame-fixed "back-foot" "back-right"
   ;;                    :tf (tf* nil (vec 0 (- *spine-off*)  (* .495 *back-len*)))
   ;;                    :geometry (albox *l* *l* *back-len*))

   (scene-frame-fixed "back-foot" "back-center"
                      :tf (tf* nil (vec 0 0  (* .475 *back-len*)))
                      :geometry (albox *l* *l* *back-len*))

   (scene-frame-fixed "left-foot" "left-strut"
                      :tf (tf* nil (vec 0 0 (* .475 *back-len*)))
                      :geometry (albox *l* *l* *back-len*))

   (scene-frame-fixed "right-foot" "right-strut"
                      :tf (tf* nil (vec 0 0 (* .475 *back-len*)))
                      :geometry (albox *l* *l* *back-len*))


   (scene-frame-fixed "foot" "top-support"
                      :tf (tf* (y-angle (- *top-angle*))
                               (vec 0 0  (- *spine-height* (* .5 *l*)))))

   (scene-frame-fixed "top-support" "top-left"
                      :tf (tf* nil (vec 0  *spine-off* (* -.5 *top-len* )))
                      :geometry (albox *l* *l* *top-len*))

   (scene-frame-fixed "top-support" "top-right"
                      :tf (tf* nil (vec 0  (- *spine-off*) (* -.5 *top-len* )))
                      :geometry (albox *l* *l* *top-len*))


   (scene-frame-fixed "foot" "shoulder-strut-left-base"
                      :tf (tf* (x-angle (* -1 *shoulder-strut-angle*))
                               (vec 0 *spine-off* *shoulder-height*)))

   (z-strut "shoulder-strut-left-base" "shoulder-left-strut"
                      (- *shoulder-strut-len*))

   (scene-frame-fixed "foot" "shoulder-strut-right-base"
                      :tf (tf* (x-angle (* 1 *shoulder-strut-angle*))
                               (vec 0 (- *spine-off*) *shoulder-height*)))

   (z-strut "shoulder-strut-right-base" "shoulder-right-strut"
            (- *shoulder-strut-len*))

   ))


(defparameter *scene*
  (scene-graph *frame*
               (prefix-scene-graph "left/"  *panda*
                                   :root "foot"
                                   :tf (tf* (x-angle (* -.5 pi))
                                            (vec (- *shoulder-len* *l*)
                                                 (+ *spine-off* (/ *l* 2))
                                                 *shoulder-height*)))
               (prefix-scene-graph "right/"  *panda*
                                   :root "foot"
                                   :tf (tf* (x-angle (* .5 pi))
                                            (vec (- *shoulder-len* *l*)
                                                 (+ (- *spine-off*) (/ *l* -2))
                                                 *shoulder-height*)))

               ))

;(win-set-scene-graph *panda*)
(win-set-scene-graph *scene*)

(win-run)
