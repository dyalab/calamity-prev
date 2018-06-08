(in-package :robray)


(setf (ros-package-path)
      (merge-pathnames "ros_ws/src/franka_ros/"
                       (user-homedir-pathname)))



(defparameter *panda*
  (load-scene-file "package://franka_description/robots/panda_arm_hand.urdf"))

(defparameter *g* (/ (+ 1 (sqrt 5d0)) 2))

(defparameter *l* 8e-2)

(defparameter *spine-height* (inches 64))
(defparameter *shoulder-height* (- *spine-height* (inches 12)))


(defparameter *plate-thickness* .1e-2)

(defparameter *shoulder-top* (- *spine-height* *shoulder-height*))

(defparameter *shoulder-len* (* *shoulder-top* *g*))

(defparameter *spine-off* (* *shoulder-top* *g* .5))

(defun inches  (x)
  (* x 2.54e-2))


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

(defparameter *frame-options* (draw-options-default :color (octet-color 200 200 204)
                                                    :metallic t
                                                    ;;:ambient .5
                                                    :brilliance 1.5
                                                    :crand .05
                                                    :reflection 0.05
                                                    :roughness .01
                                                    :diffuse 0.9
                                                    :specular .70))

(defun albox (x y z)
  (scene-geometry-box *frame-options*
                      (vec x y z)))
(defun albox-z (z)
  (albox *l* *l* z))

(defun z-strut (parent name len)
  (scene-frame-fixed parent name
                     :tf (tf* nil (vec 0 0 (* .5 len)))
                     :geometry (albox-z (abs len))))


(defun draw-table (parent name
                     options
                   &key
                     tf
                     height
                     width
                     length
                     thickness
                     leg)
  (flet ((leg (leg-name x y)
           (scene-frame-fixed name (draw-subframe name leg-name)
                              :tf (tf* nil (vec (- (* x length .5) (* x leg .5))
                                                (- (* y width .5) (* y leg .5))
                                                (/ height 2)))
                              :geometry (scene-geometry-box options
                                                            (vec leg leg
                                                                 height )))))

    (scene-graph
     (scene-frame-fixed parent name
                        :tf (or tf (tf* nil nil)))
     (scene-frame-fixed name (draw-subframe name "top")
                        :tf (tf* nil (vec 0 0 height))
                        :geometry (scene-geometry-box options (vec length width thickness)))
     (leg "0" 1 1)
     (leg "1" 1 -1)
     (leg "2" -1 1)
     (leg "3" -1 -1)
     )))


(defparameter *frame*
  (scene-graph

   (scene-frame-fixed nil "foot")

   (scene-frame-fixed "foot" "plate"
                      :tf (tf* nil (vec .25 0 0))
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
               ;; (draw-table "foot" "table"
               ;;             (draw-options-default
               ;;              ;:color (octet-color #x3f #x3e #x43)
               ;;              :color (octet-color 150 75 0)
               ;;              :specular '(.5 .5 .5))
               ;;             :tf (tf* nil (vec .6 0 0))
               ;;             :leg (inches 2)
               ;;             :height (inches 30)
               ;;             :width (inches 72)
               ;;             :thickness (inches 1)
               ;;             :length (inches 36))
               ))

(defparameter *configs*
  (alist-configuration-map
   `(
     ("right/panda_joint2" . ,(*  .25 pi))
     ("right/panda_joint4" . ,(* -.5 pi))
     ("right/panda_joint5" . ,(* -.5 pi))
     ("right/panda_joint6" . ,(* .5 pi))
     ("right/panda_finger_joint1" . 0)

     ("left/panda_joint2" . ,(*  .25 pi))
     ("left/panda_joint4" . ,(* -.5 pi))
     ("left/panda_joint5" . ,(* .5 pi))
     ("left/panda_joint6" . ,(* .5 pi))
     ("left/panda_finger_joint1" . 0)
   )))

;(win-set-scene-graph *panda*)
(win-set-scene-graph *scene*)

(win-set-config *configs*)

(win-run)

(setf (win-tf-camera)
      (TF*
       (QUATERNION* -0.27981351128657683d0 -0.4615744818575912d0
                    -0.7198694573721699d0 -0.4363958764034004d0)
       (VEC3* 2.744059488577463d0 1.4710023847272296d0 1.8807830794159102d0)))

(defparameter *repo-path*
  (merge-pathnames "git/calamity/"
                   (user-homedir-pathname)))

(render-win :output "calamity.pov"
            ;;:options (render-options-4k)
            :options (render-options-full-hd)
            :include (merge-pathnames "scene.inc"
                                      *repo-path*)
            )
