import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time

class CylinderDetectorNode(Node):
    def __init__(self):
        super().__init__('cylinder_detector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/drone/front_depth/points',
            self.pointcloud_callback,
            10)

        self.pose_pub = self.create_publisher(PoseArray, '/detected_cylinders', 10)

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to Open3D point cloud
        points = self.pointcloud2_to_xyz(msg)
        if points.shape[0] < 100:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Voxel downsampling to reduce computation
        pcd = pcd.voxel_down_sample(voxel_size=0.05)

        # Cylinder detection using RANSAC
        try:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                                     ransac_n=3,
                                                     num_iterations=1000)
            inlier_cloud = pcd.select_by_index(inliers, invert=True)

            # Estimate normals before cylinder segmentation
            inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

            cylinder_model, inliers_cylinder = inlier_cloud.segment_cylinder(
                distance_threshold=0.05,
                radius_bounds=(0.2, 1.0),
                ransac_n=3,
                num_iterations=1000
            )

            cylinder_cloud = inlier_cloud.select_by_index(inliers_cylinder)

            center = np.mean(np.asarray(cylinder_cloud.points), axis=0)
            height = np.max(cylinder_cloud.points, axis=0)[2] - np.min(cylinder_cloud.points, axis=0)[2]
            radius = cylinder_model.radius if hasattr(cylinder_model, 'radius') else 0.5

            # Publish pose
            pose_msg = Pose()
            pose_msg.position.x = float(center[0])
            pose_msg.position.y = float(center[1])
            pose_msg.position.z = float(center[2])

            pose_array = PoseArray()
            pose_array.header = msg.header
            pose_array.poses.append(pose_msg)
            self.pose_pub.publish(pose_array)

            self.get_logger().info(f"Cylinder at {center}, Height: {height:.2f}, Radius: {radius:.2f}")

        except Exception as e:
            self.get_logger().warn(f"No cylinder found: {str(e)}")

    def pointcloud2_to_xyz(self, cloud_msg):
        pc = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.array(list(pc))

def main(args=None):
    rclpy.init(args=args)
    node = CylinderDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
