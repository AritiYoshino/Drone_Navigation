#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class VinsToPx4Odom
{
public:
  VinsToPx4Odom() : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/uav1/vins_estimator/imu_propagate");
    pnh_.param<std::string>("output_topic", output_topic_, "/mavros/odometry/out");
    pnh_.param<std::string>("parent_frame", parent_frame_, "odom");
    pnh_.param<std::string>("child_frame", child_frame_, "base_link");
    pnh_.param("publish_rate", publish_rate_, 50.0);
    pnh_.param("input_timeout", input_timeout_, 0.15);
    pnh_.param("position_stddev", position_stddev_, 0.10);
    pnh_.param("orientation_stddev", orientation_stddev_, 0.10);
    pnh_.param("velocity_stddev", velocity_stddev_, 0.10);
    pnh_.param("angular_velocity_stddev", angular_velocity_stddev_, 0.20);

    if (publish_rate_ <= 0.0 || input_timeout_ <= 0.0)
    {
      ROS_FATAL("publish_rate and input_timeout must be positive");
      ros::shutdown();
      return;
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
    odom_sub_ = nh_.subscribe(input_topic_, 10, &VinsToPx4Odom::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &VinsToPx4Odom::publishTimer, this);

    ROS_INFO("VINS to PX4 odometry bridge: %s -> %s at %.1f Hz",
             input_topic_.c_str(), output_topic_.c_str(), publish_rate_);
  }

private:
  static bool finite(double value)
  {
    return std::isfinite(value);
  }

  bool validInput(const nav_msgs::Odometry &msg) const
  {
    const geometry_msgs::Point &p = msg.pose.pose.position;
    const geometry_msgs::Quaternion &q = msg.pose.pose.orientation;
    const geometry_msgs::Vector3 &v = msg.twist.twist.linear;

    return finite(p.x) && finite(p.y) && finite(p.z) &&
           finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
           finite(v.x) && finite(v.y) && finite(v.z);
  }

  void setCovariance(nav_msgs::Odometry &msg) const
  {
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);

    const double position_variance = position_stddev_ * position_stddev_;
    const double orientation_variance = orientation_stddev_ * orientation_stddev_;
    const double velocity_variance = velocity_stddev_ * velocity_stddev_;
    const double angular_velocity_variance =
        angular_velocity_stddev_ * angular_velocity_stddev_;

    msg.pose.covariance[0] = position_variance;
    msg.pose.covariance[7] = position_variance;
    msg.pose.covariance[14] = position_variance;
    msg.pose.covariance[21] = orientation_variance;
    msg.pose.covariance[28] = orientation_variance;
    msg.pose.covariance[35] = orientation_variance;

    msg.twist.covariance[0] = velocity_variance;
    msg.twist.covariance[7] = velocity_variance;
    msg.twist.covariance[14] = velocity_variance;
    msg.twist.covariance[21] = angular_velocity_variance;
    msg.twist.covariance[28] = angular_velocity_variance;
    msg.twist.covariance[35] = angular_velocity_variance;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &input)
  {
    if (!validInput(*input))
    {
      ROS_WARN_THROTTLE(1.0, "Rejecting VINS odometry containing non-finite values");
      return;
    }

    const geometry_msgs::Quaternion &input_q = input->pose.pose.orientation;
    Eigen::Quaterniond q(input_q.w, input_q.x, input_q.y, input_q.z);
    if (!finite(q.norm()) || q.norm() < 1e-6)
    {
      ROS_WARN_THROTTLE(1.0, "Rejecting VINS odometry with invalid quaternion");
      return;
    }
    q.normalize();

    const geometry_msgs::Vector3 &input_v = input->twist.twist.linear;
    const Eigen::Vector3d velocity_world(input_v.x, input_v.y, input_v.z);
    const Eigen::Vector3d velocity_body = q.conjugate() * velocity_world;

    latest_odom_ = *input;
    latest_odom_.header.frame_id = parent_frame_;
    latest_odom_.child_frame_id = child_frame_;
    latest_odom_.pose.pose.orientation.x = q.x();
    latest_odom_.pose.pose.orientation.y = q.y();
    latest_odom_.pose.pose.orientation.z = q.z();
    latest_odom_.pose.pose.orientation.w = q.w();
    latest_odom_.twist.twist.linear.x = velocity_body.x();
    latest_odom_.twist.twist.linear.y = velocity_body.y();
    latest_odom_.twist.twist.linear.z = velocity_body.z();
    latest_odom_.twist.twist.angular.x = 0.0;
    latest_odom_.twist.twist.angular.y = 0.0;
    latest_odom_.twist.twist.angular.z = 0.0;
    setCovariance(latest_odom_);

    last_receive_time_ = ros::Time::now();
    ++input_generation_;
    have_input_ = true;
  }

  void publishTimer(const ros::TimerEvent &)
  {
    if (!have_input_)
    {
      ROS_WARN_THROTTLE(2.0, "Waiting for VINS odometry on %s", input_topic_.c_str());
      return;
    }

    const double input_age = (ros::Time::now() - last_receive_time_).toSec();
    if (input_age > input_timeout_)
    {
      ROS_ERROR_THROTTLE(1.0, "VINS odometry timeout: %.3f s", input_age);
      return;
    }

    if (published_generation_ == input_generation_)
      return;

    odom_pub_.publish(latest_odom_);
    published_generation_ = input_generation_;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  ros::Timer publish_timer_;

  std::string input_topic_;
  std::string output_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  double publish_rate_;
  double input_timeout_;
  double position_stddev_;
  double orientation_stddev_;
  double velocity_stddev_;
  double angular_velocity_stddev_;

  nav_msgs::Odometry latest_odom_;
  ros::Time last_receive_time_;
  unsigned long input_generation_{0};
  unsigned long published_generation_{0};
  bool have_input_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vins_to_px4_odom");
  VinsToPx4Odom bridge;
  ros::spin();
  return 0;
}
