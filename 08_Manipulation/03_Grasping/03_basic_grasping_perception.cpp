#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <simple_grasping/object_support_segmentation.h>
#include <simple_grasping/shape_grasp_planner.h>
#include <grasping_msgs/FindGraspableObjectsAction.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

namespace simple_grasping
{

/**
 *  \brief ROS wrapper for shape grasp planner + object support segmentation
 */
class BasicGraspingPerception
{
  typedef actionlib::SimpleActionServer<grasping_msgs::FindGraspableObjectsAction> server_t;

public:
  BasicGraspingPerception(ros::NodeHandle n) : nh_(n), debug_(false), find_objects_(false)
  {
    // use_debug: enable/disable output of a cloud containing object points
    //With the following line, we can enable/disable the output of a cloud containing the object points of the graspable detections. 
    //This is why in the launch file, basic_perception_dbg.launch, we set the use_debug parameter to True. And because of this, you have been able to visualize the detections through RViz.
    nh_.getParam("use_debug", debug_);

    // frame_id: frame to transform cloud to (should be XY horizontal)
    //Here we set the world frame to base_link.
    world_frame_ = "base_link";
    nh_.getParam("frame_id", world_frame_);

    // Create planner
    //This line is very important. Here we are creating the planner that, using MoveIt, 
    //will calculate all the motions required in order to pick and place the object. The code of this planner is located in the script shape_grasp_planner.cpp.
    planner_.reset(new ShapeGraspPlanner(nh_));

    // Create perception
    // Here we are creating the Perception object that will be in charge of receiving the 
    // PointCloud data and processing it, in order to find any graspable object. You can better see this in the object_support_segmentation.cpp script.
    segmentation_.reset(new ObjectSupportSegmentation(nh_));

    // Advertise an action for perception + planning
    //Here we are advertising the Action Server that we used in the previous exercise, named find_objects. 
    //As you already know, this Action Server will be in charge of triggering the whole process 
    //of detecting the graspable object, and then calculating and generating all the motions in order to pick the object up and place it.
    server_.reset(new server_t(nh_, "find_objects",
                               boost::bind(&BasicGraspingPerception::executeCallback, this, _1),
                               false));

    // Publish debugging views
    //It basically says that, if the use_debug parameter is set to True, 2 new topics will be advertised containing the PointCloud data of the graspable object and its surface.
    if (debug_)
    {
      object_cloud_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("object_cloud", 1);
      support_cloud_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("support_cloud", 1);
    }

    // Range filter for cloud
    range_filter_.setFilterFieldName("z");
    range_filter_.setFilterLimits(0, 2.5);

    // Subscribe to head camera cloud
    // we are subscribing to the PointCloud of Fetch's head camera, which is published into the /head_camera/depth_registered/points topic
    cloud_sub_ = nh_.subscribe< pcl::PointCloud<pcl::PointXYZRGB> >("/head_camera/depth_registered/points",
                                                                     1,
                                                                     &BasicGraspingPerception::cloudCallback,
                                                                     this);

    // Start thread for action
    server_->start();

    ROS_INFO("basic_grasping_perception initialized");
  }

private:
//Here we are basically filtering all the noise in the PointCloud recevied from the Fetch robot and separating what is the graspable object from the surface.
  void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
  {
    // be lazy
    if (!find_objects_)
      return;

    ROS_DEBUG("Cloud recieved with %d points.", static_cast<int>(cloud->points.size()));

    // Filter out noisy long-range points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    range_filter_.setInputCloud(cloud);
    range_filter_.filter(*cloud_filtered);
    ROS_DEBUG("Filtered for range, now %d points.", static_cast<int>(cloud_filtered->points.size()));

    // Transform to grounded
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud(world_frame_, *cloud_filtered, *cloud_transformed, listener_))
    {
      ROS_ERROR("Error transforming to frame %s", world_frame_.c_str());
      return;
    }

    // Run segmentation
    objects_.clear();
    supports_.clear();
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> support_cloud;
    if (debug_)
    {
      object_cloud.header.frame_id = cloud_transformed->header.frame_id;
      support_cloud.header.frame_id = cloud_transformed->header.frame_id;
    }
    segmentation_->segment(cloud_transformed, objects_, supports_, object_cloud, support_cloud, debug_);

    if (debug_)
    {
      object_cloud_pub_.publish(object_cloud);
      support_cloud_pub_.publish(support_cloud);
    }

    // Ok to continue processing
    find_objects_ = false;
  }
//callback function of the action. Here, we are generating all the motion plans in order to be able to reach the graspable object detected
  void executeCallback(const grasping_msgs::FindGraspableObjectsGoalConstPtr& goal)
  {
    grasping_msgs::FindGraspableObjectsResult result;

    // Get objects
    find_objects_ = true;
    ros::Time t = ros::Time::now();
    while (find_objects_ == true)
    {
      ros::Duration(1/50.0).sleep();
      if (ros::Time::now() - t > ros::Duration(3.0))
      {
        find_objects_ = false;
        server_->setAborted(result, "Failed to get camera data in alloted time.");
        ROS_ERROR("Failed to get camera data in alloted time.");
        return;
      }
    }

    // Set object results
    for (size_t i = 0; i < objects_.size(); ++i)
    {
      grasping_msgs::GraspableObject g;
      g.object = objects_[i];
      if (goal->plan_grasps)
      {
        // Plan grasps for object
        planner_->plan(objects_[i], g.grasps);
      }
      result.objects.push_back(g);
    }
    // Set support surfaces
    result.support_surfaces = supports_;

    server_->setSucceeded(result, "Succeeded.");
  }

  ros::NodeHandle nh_;

  bool debug_;

  tf::TransformListener listener_;
  std::string world_frame_;

  bool find_objects_;
  std::vector<grasping_msgs::Object> objects_;
  std::vector<grasping_msgs::Object> supports_;

  ros::Subscriber cloud_sub_;
  ros::Publisher object_cloud_pub_;
  ros::Publisher support_cloud_pub_;

  boost::shared_ptr<ShapeGraspPlanner> planner_;
  boost::shared_ptr<ObjectSupportSegmentation> segmentation_;

  boost::shared_ptr<server_t> server_;

  pcl::PassThrough<pcl::PointXYZRGB> range_filter_;
};

}  // namespace simple_grasping

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basic_grasping_perception");
  ros::NodeHandle n("~");
  simple_grasping::BasicGraspingPerception perception_n_planning(n);
  ros::spin();
  return 0;
}
