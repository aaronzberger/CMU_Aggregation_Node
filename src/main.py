#!/home/aaron/py36/bin/python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from scipy.spatial.transform import Rotation as R
import message_filters

# Define how many frames to aggregate into one pointcloud
AGGREGATION_COUNT = 5


def point_cloud(points=None, parent_frame='velodyne'):
    '''
    Create a PointCloud2 message.
    Parameters:
        points: Nx4 array of xyz positions (m) and reflectances (0-1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    '''
    ros_dtype = PointField.FLOAT32

    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    if points is not None:
        points = points.astype(dtype)

    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(['x', 'y', 'z', 'intensity'])]

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

    if points is None:
        return PointCloud2(
            header=header,
            height=1,
            is_dense=True,
            is_bigendian=False,
            point_step=(itemsize * 4),
            width=0,
            fields=fields)

    data = points.astype(dtype).tobytes()

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 4),
        row_step=(itemsize * 4 * points.shape[0]),
        data=data)

class Aggregation_Node:
    def __init__(self):

        self.sub_registered = message_filters.Subscriber(
            '/velodyne_cloud_registered', PointCloud2)
        self.sub_transform = message_filters.Subscriber(
            '/aft_mapped_to_init', Odometry)
        ts = message_filters.TimeSynchronizer(
            [self.sub_registered, self.sub_transform], queue_size=20)
        ts.registerCallback(self.velodyne_callback)

        self.pub_aggregated = rospy.Publisher(
            '/velodyne_aggregated', PointCloud2, queue_size=1)
        self.cloud_list = []

    def velodyne_callback(self, data_velodyne, data_transform):
        def odom_to_matrix(msg):
            frame = np.eye(4)
            frame[:3, :3] = R.from_quat([getattr(msg.pose.pose.orientation, i) for i in 'xyzw']).as_matrix()
            frame[:3, 3] = [getattr(msg.pose.pose.position, i) for i in 'xyz']
            return frame

        np_lidar = pointcloud2_to_xyz_array(data_velodyne)
        self.cloud_list.append(np_lidar)

        if len(self.cloud_list) > AGGREGATION_COUNT:
            self.cloud_list.pop(0)

        final_cloud = np.empty((0, 4), dtype=np.float)
        for cloud in self.cloud_list:
            this_frame = odom_to_matrix(data_transform)
            this_frame = np.linalg.inv(this_frame)

            # Pad with reflectances
            cloud = np.concatenate(
                (cloud, np.ones((cloud.shape[0], 1))), axis=1)

            cloud = cloud.transpose()

            transformed =  this_frame @ cloud
            transformed = transformed.transpose()

            final_cloud = np.append(final_cloud, transformed, axis=0)

        ros_transformed = point_cloud(final_cloud, 'velodyne')

        ros_transformed.header = Header(stamp=data_velodyne.header.stamp, frame_id='velodyne')
        ros_transformed.row_step = ros_transformed.width * ros_transformed.point_step
        self.pub_aggregated.publish(ros_transformed)


if __name__ == "__main__":
    rospy.init_node('aggregation', log_level=rospy.INFO)

    aggregation_node = Aggregation_Node()

    rospy.loginfo('started aggregation node')

    rospy.spin()
