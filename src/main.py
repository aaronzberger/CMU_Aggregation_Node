#!/home/aaron/py27/bin/python

import rospy
from rospy import topics
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

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

        self.frame_counter = 0
        self.merged_cloud = get_default_message()

    def registered_callback(self, data):
        # print('ORIGINAL:')
        # print('header', data.header, 'height', data.height, 'width', data.width, 'fields', data.fields,
        #       'bigend', data.is_bigendian, 'ptstep', data.point_step, 'rowstep', data.row_step, 'dense', data.is_dense)
        self.merged_cloud.data += data.data
        self.merged_cloud.width += data.width

        self.frame_counter += 1

        if self.frame_counter >= AGGREGATION_COUNT:
            self.merged_cloud.header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
            self.merged_cloud.row_step = self.merged_cloud.width * self.merged_cloud.point_step

            # print('NEW:')
            # print('header', self.merged_cloud.header, 'height', self.merged_cloud.height, 'width', self.merged_cloud.width, 'fields', self.merged_cloud.fields,
            #       'bigend', self.merged_cloud.is_bigendian, 'ptstep', self.merged_cloud.point_step, 'rowstep', self.merged_cloud.row_step, 'dense', self.merged_cloud.is_dense)
            self.pub_aggregated.publish(self.merged_cloud)

            self.frame_counter = 0
            self.merged_cloud = get_default_message()


if __name__ == "__main__":
    rospy.init_node('aggregation', log_level=rospy.INFO)

    aggregation_node = Aggregation_Node()

    rospy.loginfo('started aggregation node')

    rospy.spin()
