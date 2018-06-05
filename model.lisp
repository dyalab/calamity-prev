(in-package :robray)


(setf (ros-package-path)
      (merge-pathnames "ros_ws/src/franka_ros/"
                       (user-homedir-pathname)))



(defparameter *panda*
  (load-scene-file "package://franka_description/robots/panda_arm_hand.urdf"))


(defparameter *l* 8e-2)

(defparameter *spine-height* 1.75)
(defparameter *shoulder-height* 1.5)
(defparameter *spine-off* .25)
(defparameter *plate-thickness* .1e-2)

(defparameter *shoulder-len* .5)

(defparameter *back* .75)


(defparameter *back-len* (- (vec-norm  (vec *back* *spine-height*))
                            1e-2))
(defparameter *back-angle* (atan *back*
                                 *spine-height*))


(defparameter *top-vec* (vec *shoulder-len* (- *spine-height* *shoulder-height*)))
(defparameter *top-len* (- (vec-norm *top-vec*) *l*))
(defparameter *top-angle* (atan (elt *top-vec* 0)
                                (elt *top-vec* 1)))

(defparameter *frame-options* (draw-options-default :color '(.5 .5 .5)))

(defun albox (x y z)
  (scene-geometry-box *frame-options*
                      (vec x y z)))

(defparameter *frame*
  (scene-graph

   (scene-frame-fixed nil "foot")

   (scene-frame-fixed "foot" "spine-left"
                      :tf (tf* nil (vec 0 *spine-off* (* .5 *spine-height*)))
                      :geometry (albox *l* *l* *spine-height*))
   (scene-frame-fixed "foot" "plate"
                      :tf (tf* nil nil)
                      :geometry (albox 2 2 *plate-thickness*))
   (scene-frame-fixed "foot" "spine-right"
                      :tf (tf* nil (vec 0 (- *spine-off*) (* .5 *spine-height*)))
                      :geometry (albox *l* *l* *spine-height*))
   (scene-frame-fixed "foot" "cross-top"
                      :tf (tf* nil (vec 0 0 (- *spine-height* (* .5 *l*))))
                      :geometry (albox *l*
                                       (- (* 2 *spine-off*) *l*)
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


   (scene-frame-fixed "foot" "back-foot"
                      :tf (tf* (y-angle *back-angle*) (vec (- *back*) 0 0)))

   (scene-frame-fixed "back-foot" "back-left"
                      :tf (tf* nil (vec 0 *spine-off*  (* .495 *back-len*)))
                      :geometry (albox *l* *l* *back-len*))

   (scene-frame-fixed "back-foot" "back-right"
                      :tf (tf* nil (vec 0 (- *spine-off*)  (* .495 *back-len*)))
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
