#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("simple_swipe");


class SimpleSwipe : public rclcpp::Node
{
public:
    static constexpr double MIRROR_X = 0.7;
    static constexpr double MIRROR_Y = 0.0;
    static constexpr double MIRROR_Z = 0.75;
    static constexpr double SPRAY_DISTANCE = 0.4;
    
  SimpleSwipe() : Node("simple_Swipe")
{
    RCLCPP_INFO(LOGGER, "Simple swipe node initialized!");

}

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    
    std::vector<geometry_msgs::msg::Pose> calculateCleaningPaths()
  {
    std::vector<geometry_msgs::msg::Pose> cleaning_poses;

    // Mirror parameters (from your constants)
    double mirror_x = MIRROR_X;
    double mirror_y = MIRROR_Y;
    double mirror_z = MIRROR_Z;
    double cleaning_distance = 0.3; // 30cm from mirror

    // Cleaning area dimensions..play with this
    double clean_width = mirror_width_ * 0.8;   // Clean 80% of mirror width
    double clean_height = mirror_height_ * 0.8; // Clean 80% of mirror height
    double swipe_spacing = 0.1;                 // 8cm between horizontal swipes

    // Calculate cleaning positions
    double start_x = mirror_x - cleaning_distance;
    double start_y = mirror_y - (clean_width / 2.0);
    double end_y = mirror_y + (clean_width / 2.0);
    double top_z = mirror_z + (clean_height / 2.0);
    double bottom_z = mirror_z - (clean_height / 2.0);

    // Generate horizontal swipe waypoints (top to bottom)
    bool left_to_right = true;
    for (double z = top_z; z >= bottom_z; z -= swipe_spacing)
    {

      // Start point of swipe
      geometry_msgs::msg::Pose start_pose;
      start_pose.position.x = start_x;
      start_pose.position.y = left_to_right ? start_y : end_y;
      start_pose.position.z = z;
      // Brush pointing toward mirror
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.707;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.707;
      cleaning_poses.push_back(start_pose);

      // End point of swipe
      geometry_msgs::msg::Pose end_pose = start_pose;
      end_pose.position.y = left_to_right ? end_y : start_y;
      cleaning_poses.push_back(end_pose);

      left_to_right = !left_to_right; // Alternate direction
    }

    RCLCPP_INFO(LOGGER, "Generated %zu cleaning waypoints", cleaning_poses.size());
    return cleaning_poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculateMirrorCorners()
  {
    std::vector<geometry_msgs::msg::Pose> corners;
    
    // Initialize mirror_pose_ properly
    mirror_pose_.position.x = MIRROR_X;
    mirror_pose_.position.y = MIRROR_Y;
    mirror_pose_.position.z = MIRROR_Z;
    mirror_pose_.orientation.w = 1.0;

    // Calculate spray positions in front of mirror corners
    double spray_x = mirror_pose_.position.x - SPRAY_DISTANCE; // In front of mirror
    double center_y = mirror_pose_.position.y;
    double center_z = mirror_pose_.position.z;


    // Corner offsets relative to mirror center
    double half_width = mirror_width_ / 2.0;
    double half_height = mirror_height_ / 2.0;

    // Four corners: top-left, top-right, bottom-right, bottom-left
    std::vector<std::pair<double, double>> offsets = {
        {-half_width, half_height}, // top-left
        {half_width, half_height},  // top-right
        {half_width, -half_height}, // bottom-right
        {-half_width, -half_height} // bottom-left
    };

    std::vector<std::string> corner_names = {"top-left", "top-right", "bottom-right", "bottom-left"};

    for (size_t i = 0; i < offsets.size(); ++i)
    {
      const auto &offset = offsets[i];
      geometry_msgs::msg::Pose corner_pose;
      corner_pose.position.x = spray_x;
      corner_pose.position.y = center_y + offset.first;
      corner_pose.position.z = center_z + offset.second;

      corners.push_back(corner_pose);

      // Log each corner position
      RCLCPP_INFO(LOGGER, "Mirror corner %zu (%s): x=%.3f, y=%.3f, z=%.3f",
                  i + 1, corner_names[i].c_str(),
                  corner_pose.position.x, corner_pose.position.y, corner_pose.position.z);
    }

    return corners;
  }
    
     
    void performSwipingMotion()
    {
        RCLCPP_INFO(LOGGER, "Starting Swiping Motion");
        
        // Generate cleaning waypoints using your existing function logic
        std::vector<geometry_msgs::msg::Pose> cleaning_poses;
        double cleaning_distance = 0.3;
        
        double clean_width = mirror_width_ * 0.8;
        double clean_height = mirror_height_ * 0.8;
        double swipe_spacing = 0.1;
        
        double start_x = MIRROR_X - cleaning_distance;
        double start_y = MIRROR_Y - (clean_width / 2.0);
        double end_y = MIRROR_Y + (clean_width / 2.0);
        double top_z = MIRROR_Z + (clean_height / 2.0);
        double bottom_z = MIRROR_Z - (clean_height / 2.0);
        
        bool left_to_right = true;
        for (double z = top_z; z >= bottom_z; z -= swipe_spacing)
        {
            geometry_msgs::msg::Pose start_pose;
            start_pose.position.x = start_x;
            start_pose.position.y = left_to_right ? start_y : end_y;
            start_pose.position.z = z;
            start_pose.orientation.x = 0.0;
            start_pose.orientation.y = 0.707;
            start_pose.orientation.z = 0.0;
            start_pose.orientation.w = 0.707;
            cleaning_poses.push_back(start_pose);
            
            geometry_msgs::msg::Pose end_pose = start_pose;
            end_pose.position.y = left_to_right ? end_y : start_y;
            cleaning_poses.push_back(end_pose);
            
            left_to_right = !left_to_right;
        }
        
        RCLCPP_INFO(LOGGER, "Generated %zu cleaning waypoints", cleaning_poses.size());
        
        // Execute Cartesian path 
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;       // 1cm resolution
        
        double fraction = arm_group_->computeCartesianPath(cleaning_poses, eef_step, trajectory);
        
        if (fraction > 0.8) {  // If at least 80% of path is feasible
            RCLCPP_INFO(LOGGER, "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory = trajectory;  
            
            auto result = arm_group_->execute(plan);
            if (result == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_INFO(LOGGER, "Swiping motion completed successfully!");
            } else {
                RCLCPP_ERROR(LOGGER, "Failed to execute swiping motion");
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Could not compute valid Cartesian path (only %.2f%% achieved)", fraction * 100.0);
        }
        
        RCLCPP_INFO(LOGGER, "=== Swiping Motion Complete ===");

            //  Move to ready pose 
        RCLCPP_INFO(LOGGER, "Moving to ready position...");
        arm_group_->setNamedTarget("ready");

    }

private:
    geometry_msgs::msg::Pose mirror_pose_;
    double mirror_width_ = 0.6;
    double mirror_height_ = 0.4;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleSwipe>();
    
    
    try {
    node->arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, "arm");
    node->gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, "gripper");
        
        RCLCPP_INFO(LOGGER, "Move groups initialized!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Failed to initialize move groups: %s", e.what());
        return -1;
    }
    

    node->performSwipingMotion();
    
    RCLCPP_INFO(LOGGER, "swipe complete! Node spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}