/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   mujoco_ros_control.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
**/


#include <boost/bind.hpp>
#include <mujoco_ros_control/mujoco_ros_control.h>
#include <mujoco_ros_control/visualization_utils.h>
#include <urdf/model.h>
#include <string>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>

mujoco_ros_control::MujocoVisualizationUtils &mj_vis_utils =
    mujoco_ros_control::MujocoVisualizationUtils::getInstance();

namespace enc = sensor_msgs::image_encodings;

bool PUBLISH_DEPTH = true;

namespace mujoco_ros_control
{
MujocoRosControl::MujocoRosControl()
: n_free_joints_(0)
{
}

MujocoRosControl::~MujocoRosControl()
{
  // deallocate existing mjModel
  mj_deleteModel(mujoco_model);

  // deallocate existing mjData
  mj_deleteData(mujoco_data);
  mj_deactivate();
}

bool MujocoRosControl::init(ros::NodeHandle &nodehandle)
{
      // Check that ROS has been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Unable to initialize Mujoco node.");
        return false;
    }

    if (nodehandle.getParam("mujoco_ros_control/key_path", key_path_))
    {
      ROS_INFO("Got param activation key path: %s", key_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'key_path', attempting activation with default ('%s')", key_path_.c_str());
    }

    // activation license mujoco
    mj_activate(key_path_.c_str());

    // publish clock for simulated time
    pub_clock_ = nodehandle.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // depth_pub_ definition
    // TODO: multiple of these, one per camera.
    // each camera should have: depth publisher, info publisher, fovy, name?
    pub_depth_ = nodehandle.advertise<sensor_msgs::Image>("/depth", 10);
    pub_cam_info_ = nodehandle.advertise<sensor_msgs::CameraInfo>("/cam_1_info", 10);

    // create robot node handle
    robot_node_handle = ros::NodeHandle("/");

    ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control node in namespace: %s", robot_namespace_.c_str());

    // read urdf from ros parameter server then setup actuators and mechanism control node.
    if (nodehandle.getParam("mujoco_ros_control/robot_description_param", robot_description_param_))
    {
      ROS_INFO("Got param Robot description: %s", robot_description_param_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_description_param'");
    }

    const std::string urdf_string = get_urdf(robot_description_param_);

    if (!parse_transmissions(urdf_string))
    {
      ROS_ERROR_NAMED("mujoco_ros_control", "Error parsing URDF in mujoco_ros_control node, node not active.\n");
      return false;
    }

    if (nodehandle.getParam("mujoco_ros_control/robot_model_path", robot_model_path_))
    {
      ROS_INFO("Got param: %s", robot_model_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_model_path'");
    }

    char error[1000];

    // create mjModel
    mujoco_model = mj_loadXML(robot_model_path_.c_str(), NULL, error, 1000);
    if (!mujoco_model)
    {
      printf("Could not load mujoco model with error: %s.\n", error);
      return false;
    }

    // create mjData corresponding to mjModel
    mujoco_data = mj_makeData(mujoco_model);
    if (!mujoco_data)
    {
      printf("Could not create mujoco data from model.\n");
      return false;
    }

    // check number of dofs
    get_number_of_dofs();

    // get the Mujoco simulation period
    ros::Duration mujoco_period(mujoco_model->opt.timestep);

    // set control period as mujoco_period
    control_period_ = mujoco_period;

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin>
          ("mujoco_ros_control", "mujoco_ros_control::RobotHWSimPlugin"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHWSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    // get robot links from urdf
    std::map<std::string, boost::shared_ptr<urdf::Link> > robot_links;
    robot_links = urdf_model_ptr->links_;
    std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator it;
    for (it = robot_links.begin(); it != robot_links.end(); ++it)
    {
      robot_link_names_.push_back(it->first);
    }

    // check for objects
    check_objects_in_scene();

    ROS_INFO("Initialising robot simulation interface...");
    try
    {
      if (!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, mujoco_model,
                                  mujoco_data, urdf_model_ptr, transmissions_, n_free_joints_))
      {
        ROS_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot sim interface");
        return false;
      }
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Failed to initialise robot simulation interface.");
      ROS_ERROR("%s", e.what());
      return false;
    }

    // create the controller manager
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_node_handle));
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("mujoco_ros_control" , "Failed to create robot sim interface loader: "
                             << ex.what());
    }
    ROS_INFO_NAMED("mujoco_ros_control", "Loaded mujoco_ros_control.");

    // set up the initial simulation environment
    setup_sim_environment();
    return true;
}

void MujocoRosControl::setup_sim_environment()
{
  XmlRpc::XmlRpcValue robot_joints, robot_initial_state;
  bool params_read_correctly = true;

  if (!robot_node_handle.getParam("robot_joints", robot_joints))
  {
    ROS_WARN("Failed to get param 'robot_joints'");
    params_read_correctly = false;
  }

  if (params_read_correctly && robot_node_handle.getParam("robot_initial_state", robot_initial_state))
  {
    for (int i = 0; i < robot_joints.size(); i++)
    {
      for (XmlRpc::XmlRpcValue::iterator it = robot_initial_state.begin(); it != robot_initial_state.end(); ++it)
      {
        if (robot_joints[i] == it->first)
        {
          mujoco_data->qpos[i] = it->second;
        }
      }
    }
  }
  else
  {
    ROS_WARN("Failed to get param 'robot_initial_state'");
    params_read_correctly = false;
  }

  if (!params_read_correctly)
  {
    for (int i=0; i < n_dof_-objects_in_scene_.size(); i++)
    {
      mujoco_data->qpos[i] = 0;
    }
  }

  // compute forward kinematics for new pos
  mj_forward(mujoco_model, mujoco_data);

  // run simulation to setup the new pos
  mj_step(mujoco_model, mujoco_data);
}

void MujocoRosControl::update()
{
  publish_sim_time();

  ros::Time sim_time = (ros::Time)mujoco_data->time;
  ros::Time sim_time_ros(sim_time.sec, sim_time.nsec);

  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  mj_step1(mujoco_model, mujoco_data);

  // check if we should update the controllers
  if (sim_period >= control_period_)
  {
    // store simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // update the robot simulation with the state of the mujoco model
    robot_hw_sim_->read(sim_time_ros, sim_period);

    bool reset_ctrls = false;

    // compute the controller commands
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrls);
  }

  // update the mujoco model with the result of the controller
  robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);

  last_write_sim_time_ros_ = sim_time_ros;
  mj_step2(mujoco_model, mujoco_data);

  publish_objects_in_scene();

  if (PUBLISH_DEPTH)
  {
    int dev_cam_num = 1;
    publish_depth_image(dev_cam_num);
    // // TODO: loop over cameras.
    // for(int i=0; i<(2);i++)
    // {
    //   publish_depth_image(i);
    // }
  }
}

// get the URDF XML from the parameter server
std::string MujocoRosControl::get_urdf(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (robot_node_handle.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_node_handle.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_param_.c_str());

      robot_node_handle.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Received urdf from param server, parsing...");

  return urdf_string;
}

// get Transmissions from the URDF
bool MujocoRosControl::parse_transmissions(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

std::string MujocoRosControl::geom_type_to_string(int geom_type)
{
  std::string result;
  switch (geom_type)
  {
    case 0 :
      result = mujoco_ros_msgs::ModelStates::PLANE;
      break;
    case 1 :
      result = mujoco_ros_msgs::ModelStates::HFIELD;
      break;
    case 2 :
      result = mujoco_ros_msgs::ModelStates::SPHERE;
      break;
    case 3 :
      result = mujoco_ros_msgs::ModelStates::CAPSULE;
      break;
    case 4 :
      result = mujoco_ros_msgs::ModelStates::ELLIPSOID;
      break;
    case 5 :
      result = mujoco_ros_msgs::ModelStates::CYLINDER;
      break;
    case 6 :
      result = mujoco_ros_msgs::ModelStates::BOX;
      break;
    case 7 :
      result = mujoco_ros_msgs::ModelStates::MESH;
      break;
    default:
      result = "unknown_type";
      break;
  }
  return result;
}

void MujocoRosControl::get_number_of_dofs()
{
  n_dof_ = mujoco_model->njnt;
}

void MujocoRosControl::publish_sim_time()
{
  ros::Time sim_time = (ros::Time)mujoco_data->time;

  // TODO: turn this back on & set a proper rate.
  // if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).toSec() < 1.0/pub_clock_frequency_)
  //   return;

  ros::Time current_time = (ros::Time)mujoco_data->time;
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(current_time.toSec());
  // publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

void MujocoRosControl::publish_depth_image(const int& fixedcamid)
{
  // if it isn't time, exit
  ros::Time sim_time = (ros::Time)mujoco_data->time;
  if (pub_depth_freq_ > 0 && (sim_time - last_pub_depth_time_).toSec() < 1.0/pub_depth_freq_)
    return;

  // render from cam_fixedcamid
  unsigned char* rgb = (unsigned char*)malloc(3*width*height);
  float* depth = (float*)malloc(width*height*sizeof(float));
  if( !rgb | !depth)
    ROS_ERROR("Could not allocate buffers");
  int result = mj_vis_utils.renderOffscreen(rgb, depth, height, width, fixedcamid);

  // params needed for conversion to real depth.
  // TODO: abstract this.
  float extent = mujoco_model->stat.extent;
  float znear = mujoco_model->vis.map.znear * extent;
  float zfar = mujoco_model->vis.map.zfar * extent;


  // TODO: I THINK this is right...
  float fovy = mujoco_model->cam_fovy[fixedcamid];

  // create depth message
  sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
  depth_msg->header.stamp    = sim_time;
  depth_msg->header.frame_id = "cam_" + std::to_string(fixedcamid) + "_optical";
  depth_msg->height          = height;
  depth_msg->width           = width;
  depth_msg->encoding        = enc::TYPE_32FC1;
  depth_msg->step            = width * (enc::bitDepth(depth_msg->encoding) / 8);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);
  float* depth_data = reinterpret_cast<float*>(&depth_msg->data[0]);

  // fill the depth message.
  // glReadPixels returns rows bottom to top, left to right.
  // so reverse the buffer vertically, convert to depth in meters
  for(int i=0; i<(width*height);i++)
  {
      int proper_row_number = std::floor( ((float)i) / ((float)width) );
      int proper_column_number = i - proper_row_number*width;
      int gl_row_number = height - proper_row_number;
      int proper_idx = gl_row_number*width + proper_column_number;

      // convert to real
      float real_depth = znear / (1.0 - depth[proper_idx] * (1.0 - znear / zfar));
      depth_data[i] = (float)real_depth;
  }

  // publish!
  last_pub_depth_time_ = sim_time;
  pub_depth_.publish(depth_msg);

  // grab parameters of the camera
  // TODO: move this elsewhere.
  float fovy_rad = fovy * 3.14159265 / 180.0;
  float cx = (float)width / 2.0;
  float cy = (float)height / 2.0;
  float fy = (float(height)/2.0) / tan(fovy_rad / 2.0f);
  float fx = fy;

  // publish synchronized camera_info
  sensor_msgs::CameraInfo ci;
  ci.header.stamp = sim_time;
  ci.header.frame_id = "cam_" + std::to_string(fixedcamid);
  ci.height = height;
  ci.width = width;

  // TODO: map from camera data (either a deque or a map?)
  ci.K[0] = fx;
  ci.K[1] = 0;
  ci.K[2] = cx;
  ci.K[3] = 0;
  ci.K[4] = fy;
  ci.K[5] = cy;
  ci.K[6] = 0;
  ci.K[7] = 0;
  ci.K[8] = 1;

  ci.R[0] = 1.0;
  ci.R[4] = 1.0;
  ci.R[8] = 1.0;

  ci.P[0] = fx;
  ci.P[1] = 0;
  ci.P[2] = cx;
  ci.P[3] = 0;
  ci.P[4] = 0;
  ci.P[5] = fy;
  ci.P[6] = cy;
  ci.P[7] = 0;
  ci.P[8] = 0;
  ci.P[9] = 0;
  ci.P[10] = 1;
  ci.P[11] = 0;

  pub_cam_info_.publish(ci);

  // get camera pose from mujoco.
  // TODO: param with camera number
  std:: string body_name = "cam_" + std::to_string(fixedcamid) + "_body";
  int mj_cam_id = mj_name2id(mujoco_model, mjOBJ_BODY, body_name.c_str());

  int start_idx_pos = mj_cam_id * 3;
  int start_idx_quat =  mj_cam_id * 4;
  float cam_x = mujoco_data->xpos[start_idx_pos + 0];
  float cam_y = mujoco_data->xpos[start_idx_pos + 1];
  float cam_z = mujoco_data->xpos[start_idx_pos + 2];
  float qw = mujoco_data->xquat[start_idx_quat + 0];
  float qx = mujoco_data->xquat[start_idx_quat + 1];
  float qy = mujoco_data->xquat[start_idx_quat + 2];
  float qz = mujoco_data->xquat[start_idx_quat + 3];

  // publish the camera transform
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(cam_x, cam_y, cam_z) );
  tf::Quaternion q(qx, qy, qz, qw );
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,
                                        ros::Time::now(),
                                        "world",
                                        "cam_" + std::to_string(fixedcamid)));

  // publish transform from cam_1 (rendering in mujoco/OpenGL convention)
  // to cam_optical, as if rendering in ROS/OpenCV convention
  tf::Transform transform_optical;
  transform_optical.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Vector3 axis(1, 0 ,0);
  float angle=3.14159265;
  tf::Quaternion q_optical(axis, angle );
  transform_optical.setRotation(q_optical);
  br.sendTransform(tf::StampedTransform(transform_optical,
                                        ros::Time::now(),
                                        "cam_" + std::to_string(fixedcamid),
                                        "cam_" + std::to_string(fixedcamid) + "_optical"));
}

void MujocoRosControl::check_objects_in_scene()
{
  int num_of_bodies = mujoco_model->nbody;
  int object_id;
  int joint_addr;
  int joint_type;
  int num_of_joints_for_body;
  std::string object_name;

  for (int object_id=0; object_id < num_of_bodies; object_id++)
  {
    object_name = mj_id2name(mujoco_model, 1, object_id);
    num_of_joints_for_body = mujoco_model->body_jntnum[object_id];
    if (0 == num_of_joints_for_body &&
        !(std::find(robot_link_names_.begin(), robot_link_names_.end(), object_name) != robot_link_names_.end()))
    {
      objects_in_scene_[object_id] = STATIC;
      ROS_INFO_STREAM("Static object found: " << object_name);
    }
    else if (1 == num_of_joints_for_body)
    {
      joint_addr = mujoco_model->body_jntadr[object_id];
      joint_type = mujoco_model->jnt_type[joint_addr];
      if (0 == joint_type)
      {
        objects_in_scene_[object_id] = FREE;
        n_free_joints_++;
        ROS_INFO_STREAM("Free object found: " << object_name);
      }
    }
  }
}

void MujocoRosControl::publish_objects_in_scene()
{
  const int geom_size_dim = 3;
  const int xpos_dim = 3;
  const int xquat_dim = 4;
  int geom_type;
  int geom_addr;
  geometry_msgs::Pose pose;
  std_msgs::Float64MultiArray size;
  mujoco_ros_msgs::ModelStates objects;

  for (std::map<int, Object_State>::iterator it = objects_in_scene_.begin(); it != objects_in_scene_.end(); it++ )
  {
    size.data.clear();
    geom_addr = mujoco_model->body_geomadr[it->first];
    geom_type = mujoco_model->geom_type[geom_addr];

    for (int i=0; i < geom_size_dim; i++)
    {
      size.data.push_back(mujoco_model->geom_size[3 * geom_addr + i]);
    }

    pose.position.x = mujoco_data->xpos[xpos_dim * it->first];
    pose.position.y = mujoco_data->xpos[xpos_dim * it->first + 1];
    pose.position.z = mujoco_data->xpos[xpos_dim * it->first + 2];
    pose.orientation.x = mujoco_data->xquat[xquat_dim * it->first + 1];
    pose.orientation.y = mujoco_data->xquat[xquat_dim * it->first + 2];
    pose.orientation.z = mujoco_data->xquat[xquat_dim * it->first + 3];
    pose.orientation.w = mujoco_data->xquat[xquat_dim * it->first];

    objects.name.push_back(mj_id2name(mujoco_model, 1, it->first));
    objects.type.push_back(geom_type_to_string(geom_type));
    objects.is_static.push_back(it->second);
    objects.size.push_back(size);
    objects.pose.push_back(pose);
  }

  objects_in_scene_publisher.publish(objects);
}
}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujoco_ros_control");

    ros::NodeHandle nh_;

    mujoco_ros_control::MujocoRosControl mujoco_ros_control;

    // initialize mujoco stuff
    if (!mujoco_ros_control.init(nh_))
    {
      ROS_ERROR("Could not initialise mujoco.");
      return 1;
    }

    // init GLFW
    if ( !glfwInit() )
      mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // make context current
    glfwMakeContextCurrent(window);

    // initialize mujoco visualization functions
    mj_vis_utils.init(mujoco_ros_control.mujoco_model, mujoco_ros_control.mujoco_data, window);

    // spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // run main loop, target real-time simulation and 60 fps rendering
    while ( ros::ok() && !glfwWindowShouldClose(window) )
    {
      // advance interactive simulation for 1/60 sec
      // Assuming MuJoCo can simulate faster than real-time, which it usually can,
      // this loop will finish on time for the next frame to be rendered at 60 fps.
      mjtNum sim_start = mujoco_ros_control.mujoco_data->time;

      while ( mujoco_ros_control.mujoco_data->time - sim_start < 1.0/60.0 && ros::ok() )
      {
        mujoco_ros_control.update();
      }
      mj_vis_utils.update(window);
    }

    mj_vis_utils.terminate();

    return 0;
}
