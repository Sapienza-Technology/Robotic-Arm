;; Auto-generated. Do not edit!


(when (boundp 'tag_utils::GetTag)
  (if (not (find-package "TAG_UTILS"))
    (make-package "TAG_UTILS"))
  (shadow 'GetTag (find-package "TAG_UTILS")))
(unless (find-package "TAG_UTILS::GETTAG")
  (make-package "TAG_UTILS::GETTAG"))
(unless (find-package "TAG_UTILS::GETTAGREQUEST")
  (make-package "TAG_UTILS::GETTAGREQUEST"))
(unless (find-package "TAG_UTILS::GETTAGRESPONSE")
  (make-package "TAG_UTILS::GETTAGRESPONSE"))

(in-package "ROS")





(defclass tag_utils::GetTagRequest
  :super ros::object
  :slots (_id ))

(defmethod tag_utils::GetTagRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tag_utils::GetTagResponse
  :super ros::object
  :slots (_tag ))

(defmethod tag_utils::GetTagResponse
  (:init
   (&key
    ((:tag __tag) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tag __tag)
   self)
  (:tag
   (&optional __tag)
   (if __tag (setq _tag __tag)) _tag)
  (:serialization-length
   ()
   (+
    ;; float32[3] _tag
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[3] _tag
     (dotimes (i 3)
       (sys::poke (elt _tag i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[3] _tag
   (dotimes (i (length _tag))
     (setf (elt _tag i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass tag_utils::GetTag
  :super ros::object
  :slots ())

(setf (get tag_utils::GetTag :md5sum-) "3cb26f00cb9a158b9a65f3463d45d1c7")
(setf (get tag_utils::GetTag :datatype-) "tag_utils/GetTag")
(setf (get tag_utils::GetTag :request) tag_utils::GetTagRequest)
(setf (get tag_utils::GetTag :response) tag_utils::GetTagResponse)

(defmethod tag_utils::GetTagRequest
  (:response () (instance tag_utils::GetTagResponse :init)))

(setf (get tag_utils::GetTagRequest :md5sum-) "3cb26f00cb9a158b9a65f3463d45d1c7")
(setf (get tag_utils::GetTagRequest :datatype-) "tag_utils/GetTagRequest")
(setf (get tag_utils::GetTagRequest :definition-)
      "int32 id
---
float32[3] tag

")

(setf (get tag_utils::GetTagResponse :md5sum-) "3cb26f00cb9a158b9a65f3463d45d1c7")
(setf (get tag_utils::GetTagResponse :datatype-) "tag_utils/GetTagResponse")
(setf (get tag_utils::GetTagResponse :definition-)
      "int32 id
---
float32[3] tag

")



(provide :tag_utils/GetTag "3cb26f00cb9a158b9a65f3463d45d1c7")


