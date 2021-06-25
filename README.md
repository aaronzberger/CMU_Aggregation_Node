# CMU_Aggregation_Node

ROS_NAMESPACE=/mapping rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _queue_size:=10
 rosrun rqt_reconfigure rqt_reconfigure
 rosrun image_view stereo_view stereo:=mapping image:=image_rect _approximate_sync:=True _queue_size:=10