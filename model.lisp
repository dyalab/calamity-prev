(in-package :robray)


(setf (ros-package-path)
      (merge-pathnames "ros_ws/src/franka_ros/"
                       (user-homedir-pathname)))



(defparameter *panda*
  (load-scene-file "package://franka_description/robots/panda_arm_hand.urdf"))


(win-set-scene-graph *panda*)

(win-run)
