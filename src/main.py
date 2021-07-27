#!/home/aaron/py36/bin/python

import rospy
from rospy import topics
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time
from nav_msgs.msg import Odometry
from interpolate_topic import Interpolation_Subscriber
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from scipy.spatial.transform import Rotation as R
import ros_numpy
import sys

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
        self.sub_registered = rospy.Subscriber(
            '/velodyne_points', PointCloud2, self.velodyne_callback)
        self.sub_transform = Interpolation_Subscriber(
            '/aft_mapped_to_init', Odometry,
            get_fn=lambda data: [getattr(data.pose.pose.position, i) for i in 'xyz'] + \
                                [getattr(data.pose.pose.orientation, i) for i in 'xyzw'])

        self.pub_aggregated = rospy.Publisher(
            '/velodyne_aggregated', PointCloud2, queue_size=1)
        self.cloud_list = []

    def velodyne_callback(self, data_velodyne):
        transform_data = self.sub_transform.get(data_velodyne.header.stamp)
        if transform_data is None:
            return

        def data_to_matrix(transform):
            frame = np.eye(4)
            frame[:3, :3] = R.from_quat(transform[3:]).as_matrix()
            frame[:3, 3] = transform[:3]
            return frame

        self.cloud_list.append([point_cloud(), data_to_matrix(transform_data), data_velodyne.header.stamp])
        self.cloud_list[-1][0].data = data_velodyne.data
        self.cloud_list[-1][0].width = data_velodyne.width
        print(sys.getsizeof(self.cloud_list[-1][0].data), self.cloud_list[-1][0].width)
        print('Add new frame with width', self.cloud_list[-1][0].width)

        for i in range(len(self.cloud_list[:-1])):
            this_frame = data_to_matrix(transform_data)

            # lidar_ = ros_numpy.numpify(data_velodyne)
            # np_lidar = np.zeros((lidar_.shape[0], 3))
            # np_lidar[:, 0] = lidar_['x']
            # np_lidar[:, 1] = lidar_['y']
            # np_lidar[:, 2] = lidar_['z']
            # np_lidar[:, 3] = lidar_['intensity']
            # np_lidar = np_lidar.astype(np.float32)

            # print(np_lidar[180])

            np_lidar = pointcloud2_to_xyz_array(data_velodyne)

            # Pad with reflectances
            np_lidar = np.concatenate(
                (np_lidar, np.ones((np_lidar.shape[0], 1))), axis=1)

            np_lidar = np_lidar.transpose()

            # print(np_lidar.shape, self.cloud_list[i][1].shape, this_frame.shape)
            transformed = self.cloud_list[i][1] @ this_frame @ np_lidar
            transformed = transformed.transpose()
            ros_transformed = point_cloud(transformed, 'velodyne')
            self.cloud_list[i][0].data += ros_transformed.data
            self.cloud_list[i][0].width += ros_transformed.width
            print(sys.getsizeof(self.cloud_list[i][0].data), self.cloud_list[i][0].width)
            print('added this cloud to index {}, from {} width to {}'.format(i, self.cloud_list[i][0].width - ros_transformed.width, self.cloud_list[i][0].width))
        
        print('widths', [item[0].width for item in self.cloud_list])

        if len(self.cloud_list) >= AGGREGATION_COUNT:
            pub_msg = self.cloud_list[0][0]
            pub_msg.header = Header(stamp=self.cloud_list[0][2], frame_id='velodyne')
            pub_msg.row_step = pub_msg.width * pub_msg.point_step
            self.pub_aggregated.publish(pub_msg)
            print('published index 0 with width', pub_msg.width)
            self.cloud_list.pop(0)


if __name__ == "__main__":
    rospy.init_node('aggregation', log_level=rospy.INFO)

    aggregation_node = Aggregation_Node()

    rospy.loginfo('started aggregation node')

    rospy.spin()
