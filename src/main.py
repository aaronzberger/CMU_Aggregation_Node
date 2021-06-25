#!/home/aaron/py27/bin/python

import rospy
from rospy import topics
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time

# Define how many frames to aggregate into one pointcloud
AGGREGATION_COUNT = 10

def get_default_message():
    obj = PointCloud2()
    obj.height = 1
    obj.is_dense = True
    obj.is_bigendian = False
    # (8-bit int * [X, Y, Z, R] = 32 bits)
    obj.point_step = 32
    obj.fields = [PointField(name='x', offset=0, datatype=7, count=1),
                  PointField(name='y', offset=4, datatype=7, count=1),
                  PointField(name='z', offset=8, datatype=7, count=1),
                  PointField(name='intensity', offset=16, datatype=7, count=1)]
    return obj

class Aggregation_Node:
    def __init__(self):
        self.sub_registered = rospy.Subscriber(
            '/velodyne_cloud_registered', PointCloud2, self.registered_callback)
        self.pub_aggregated = rospy.Publisher(
            '/velodyne_aggregated', PointCloud2, queue_size=1)

        self.cloud_list = []

    def registered_callback(self, data):
        if len(self.cloud_list) < AGGREGATION_COUNT:
            self.cloud_list.append(data)
            return
        self.cloud_list.pop(0)
        self.cloud_list.append(data)

        merged_cloud = get_default_message()
        for cloud_msg in self.cloud_list:
            merged_cloud.data += cloud_msg.data
            merged_cloud.width += cloud_msg.width

        merged_cloud.header = Header(stamp=self.cloud_list[-1].header.stamp, frame_id='camera_init')
        merged_cloud.row_step = merged_cloud.width * merged_cloud.point_step

        self.pub_aggregated.publish(merged_cloud)


if __name__ == "__main__":
    rospy.init_node('aggregation', log_level=rospy.INFO)

    aggregation_node = Aggregation_Node()

    rospy.loginfo('started aggregation node')

    rospy.spin()
