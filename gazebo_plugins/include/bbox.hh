#ifndef BBOX_HH
#define BBOX_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/RayShape.hh>
#include <visualization_msgs/Marker.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class Bbox : public RayPlugin
  {
    /// \brief Constructor
    public: Bbox();

    /// \brief Destructor
    public: ~Bbox();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: ros::Publisher pub2_;
    private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;
    private: PubQueue<visualization_msgs::Marker>::Ptr pub_queue2_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief tf prefix
    private: std::string tf_prefix_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    /// \brief prevents blocking
    private: PubMultiQueue pmq;

    private: std::string name_;
  };
}
#endif
