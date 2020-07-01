#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/RayShape.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bbox.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(Bbox)

////////////////////////////////////////////////////////////////////////////////
// Constructor
Bbox::Bbox()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Bbox::~Bbox()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Bbox::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;
  this->name_ = _parent->ParentName();

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("Bbox controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("laser", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&Bbox::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Bbox::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO_NAMED("laser", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&Bbox::LaserConnect, this),
      boost::bind(&Bbox::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();

    ros::AdvertiseOptions ao2 =
      ros::AdvertiseOptions::create<visualization_msgs::Marker>(
      this->topic_name_ + "/bboxs", 1,
      boost::bind(&Bbox::LaserConnect, this),
      boost::bind(&Bbox::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub2_ = this->rosnode_->advertise(ao2);
    this->pub_queue2_ = this->pmq.addPub<visualization_msgs::Marker>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void Bbox::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &Bbox::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void Bbox::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void Bbox::OnScan(ConstLaserScanStampedPtr &_msg)
{
  physics::EntityPtr current = this->world_->EntityByName(this->name_);
  ignition::math::Pose3d current_pose = current->WorldPose();
  current_pose = current_pose.Inverse();
  ignition::math::Vector3d current_position = current_pose.Pos();
  ignition::math::Quaterniond current_rotation = current_pose.Rot();
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "/global";
  tf.child_frame_id = this->frame_name_;
  tf.transform.translation.x = current_position.X();
  tf.transform.translation.y = current_position.Y();
  tf.transform.translation.z = current_position.Z();
  tf.transform.rotation.w = current_rotation.W();
  tf.transform.rotation.x = current_rotation.X();
  tf.transform.rotation.y = current_rotation.Y();
  tf.transform.rotation.z = current_rotation.Z();
  std::vector<std::string> objects;
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = this->frame_name_;
  line_list.header.stamp = ros::Time::now();
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;
  physics::MultiRayShapePtr multiRay = this->parent_ray_sensor_->LaserShape();
  unsigned int raycount = multiRay->RayCount();
  for (unsigned int i = 0; i < raycount; i++)
  {
    physics::RayShapePtr temp_ray = multiRay->Ray(i);
    std::string entityName;
    double dist;
    temp_ray->GetIntersection(dist,entityName);
    if (entityName == "" || std::count(objects.begin(), objects.end(), entityName))
    {
      continue;
    }
    objects.push_back(entityName);
    physics::EntityPtr entity = this->world_->EntityByName(entityName);
    ignition::math::Box box = entity->CollisionBoundingBox();
    ignition::math::Vector3d min, max;
    min = box.Min();
    max = box.Max();
    double max_x, max_y, max_z, min_x, min_y, min_z;

    geometry_msgs::PointStamped origin, transformed;
    origin.point.x = max.X();
    origin.point.y = max.Y();
    origin.point.z = max.Z();
    origin.header = tf.header;
    try {
      tf2::doTransform(origin, transformed, tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    max_x = transformed.point.x;
    max_y = transformed.point.y;
    max_z = transformed.point.z;
    origin.point.x = min.X();
    origin.point.y = min.Y();
    origin.point.z = min.Z();
    origin.header = tf.header;
    try {
      tf2::doTransform(origin, transformed, tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }
    min_x = transformed.point.x;
    min_y = transformed.point.y;
    min_z = transformed.point.z;

    geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
    p1.x = max_x;
    p1.y = max_y;
    p1.z = max_z;
    p2.x = min_x;
    p2.y = max_y;
    p2.z = max_z;
    p3.x = max_x;
    p3.y = min_y;
    p3.z = max_z;
    p4.x = min_x;
    p4.y = min_y;
    p4.z = max_z;
    p5.x = max_x;
    p5.y = max_y;
    p5.z = min_z;
    p6.x = min_x;
    p6.y = max_y;
    p6.z = min_z;
    p7.x = max_x;
    p7.y = min_y;
    p7.z = min_z;
    p8.x = min_x;
    p8.y = min_y;
    p8.z = min_z;
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    line_list.points.push_back(p1);
    line_list.points.push_back(p3);
    line_list.points.push_back(p1);
    line_list.points.push_back(p5);
    line_list.points.push_back(p8);
    line_list.points.push_back(p4);
    line_list.points.push_back(p8);
    line_list.points.push_back(p7);
    line_list.points.push_back(p8);
    line_list.points.push_back(p6);
    line_list.points.push_back(p5);
    line_list.points.push_back(p7);
    line_list.points.push_back(p3);
    line_list.points.push_back(p7);
    line_list.points.push_back(p2);
    line_list.points.push_back(p6);
    line_list.points.push_back(p2);
    line_list.points.push_back(p4);
    line_list.points.push_back(p5);
    line_list.points.push_back(p6);
    line_list.points.push_back(p4);
    line_list.points.push_back(p3);

  }
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
  this->pub_queue2_->push(line_list, this->pub2_);
}
}
