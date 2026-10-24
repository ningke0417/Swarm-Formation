#include <cmath>
#include <gtest/gtest.h>
#include <plan_env/grid_map.h>

namespace {
void spinFor(double seconds)
{
  const ros::WallTime end = ros::WallTime::now() + ros::WallDuration(seconds);
  while (ros::ok() && ros::WallTime::now() < end)
  {
    ros::spinOnce();
    ros::WallDuration(0.005).sleep();
  }
}

TEST(CloudESDF, InterpolationAtOccupiedEdgeRemainsOccupiedAfterRepeatedUpdates)
{
  ros::NodeHandle nh("~");
  GridMap map;
  map.initMap(nh);
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("grid_map/odom", 1, true);
  auto cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map/cloud", 1, true);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  while ((odom_pub.getNumSubscribers() == 0 || cloud_pub.getNumSubscribers() == 0)
         && ros::WallTime::now() < deadline)
    spinFor(0.02);
  ASSERT_GT(odom_pub.getNumSubscribers(), 0u);
  ASSERT_GT(cloud_pub.getNumSubscribers(), 0u);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = -1.0;
  odom.pose.pose.position.z = 1.0;
  odom.pose.pose.orientation.w = 1.0;
  odom_pub.publish(odom);
  spinFor(0.1);
  ASSERT_TRUE(map.odomValid());

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.05f, 0.05f, 1.05f));
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = "world";

  // This lies inside the lowest occupied y voxel, below its center. The
  // interpolation stencil includes a neighbor outside the cloud bounds.
  const Eigen::Vector3d edge(0.05, -0.075, 1.05);
  const Eigen::Vector3d nearby_free(0.05, -0.45, 1.05);
  for (int update = 0; update < 4; ++update)
  {
    cloud_pub.publish(message);
    spinFor(0.12);
    map.updateESDF3d();
    ASSERT_EQ(map.getInflateOccupancy(edge), 1);
    double edge_distance, free_distance;
    map.evaluateEDT(edge, edge_distance);
    map.evaluateEDT(nearby_free, free_distance);
    EXPECT_TRUE(std::isfinite(edge_distance));
    EXPECT_LT(edge_distance, 0.1) << "Occupied edge must not mix with the 10000 m sentinel";
    EXPECT_GT(edge_distance, -0.5);
    EXPECT_GT(free_distance, 0.2);
    EXPECT_LT(free_distance, 0.7);
  }

  Eigen::Vector3d gradient;
  map.evaluateFirstGrad(nearby_free, gradient);
  EXPECT_LT(gradient.y(), -0.5);
}
}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cloud_esdf_test");
  return RUN_ALL_TESTS();
}
