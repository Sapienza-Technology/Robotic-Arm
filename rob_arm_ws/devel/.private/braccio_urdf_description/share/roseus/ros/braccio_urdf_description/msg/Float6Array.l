;; Auto-generated. Do not edit!


(when (boundp 'braccio_urdf_description::Float6Array)
  (if (not (find-package "BRACCIO_URDF_DESCRIPTION"))
    (make-package "BRACCIO_URDF_DESCRIPTION"))
  (shadow 'Float6Array (find-package "BRACCIO_URDF_DESCRIPTION")))
(unless (find-package "BRACCIO_URDF_DESCRIPTION::FLOAT6ARRAY")
  (make-package "BRACCIO_URDF_DESCRIPTION::FLOAT6ARRAY"))

(in-package "ROS")
;;//! \htmlinclude Float6Array.msg.html


(defclass braccio_urdf_description::Float6Array
  :super ros::object
  :slots (_x1 _x2 _x3 _x4 _x5 _x6 ))

(defmethod braccio_urdf_description::Float6Array
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:x2 __x2) 0.0)
    ((:x3 __x3) 0.0)
    ((:x4 __x4) 0.0)
    ((:x5 __x5) 0.0)
    ((:x6 __x6) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _x2 (float __x2))
   (setq _x3 (float __x3))
   (setq _x4 (float __x4))
   (setq _x5 (float __x5))
   (setq _x6 (float __x6))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:x4
   (&optional __x4)
   (if __x4 (setq _x4 __x4)) _x4)
  (:x5
   (&optional __x5)
   (if __x5 (setq _x5 __x5)) _x5)
  (:x6
   (&optional __x6)
   (if __x6 (setq _x6 __x6)) _x6)
  (:serialization-length
   ()
   (+
    ;; float64 _x1
    8
    ;; float64 _x2
    8
    ;; float64 _x3
    8
    ;; float64 _x4
    8
    ;; float64 _x5
    8
    ;; float64 _x6
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x3
       (sys::poke _x3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x4
       (sys::poke _x4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x5
       (sys::poke _x5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x6
       (sys::poke _x6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x1
     (setq _x1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x2
     (setq _x2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x3
     (setq _x3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x4
     (setq _x4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x5
     (setq _x5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x6
     (setq _x6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get braccio_urdf_description::Float6Array :md5sum-) "bd6ec219eb9feb2baf2bfcaae69e9c5e")
(setf (get braccio_urdf_description::Float6Array :datatype-) "braccio_urdf_description/Float6Array")
(setf (get braccio_urdf_description::Float6Array :definition-)
      "float64 x1
float64 x2
float64 x3
float64 x4
float64 x5
float64 x6
")



(provide :braccio_urdf_description/Float6Array "bd6ec219eb9feb2baf2bfcaae69e9c5e")


