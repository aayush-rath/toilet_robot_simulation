#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toilet_surface_clean");

class ToiletSurfaceClean : public rclcpp::Node
{
public:
    ToiletSurfaceClean() : Node("toilet_surface_clean")
    {
        RCLCPP_INFO(LOGGER, "Toilet surface cleaning node initialized!");
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

    std::vector<geometry_msgs::msg::Pose> loadWaypoints()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        geometry_msgs::msg::Pose p;

        // Orientation: brush pointing downward in world frame (Z-)
        p.orientation.x = 0.0;
        p.orientation.y = 1.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;

        p.position.x = 0.8227649927139282;
        p.position.y = 0.029333114624023438;
        p.position.z = -0.07027789950370789;
        waypoints.push_back(p);

        p.position.x = 0.7745530605316162;
        p.position.y = 0.11037003993988037;
        p.position.z = -0.05868794023990631;
        waypoints.push_back(p);

        p.position.x = 0.7207593321800232;
        p.position.y = 0.19842898845672607;
        p.position.z = -0.03275929391384125;
        waypoints.push_back(p);

        p.position.x = 0.5332620143890381;
        p.position.y = 0.31353139877319336;
        p.position.z = -0.039636239409446716;
        waypoints.push_back(p);

        p.position.x = 0.4290635585784912;
        p.position.y = 0.20071962475776672;
        p.position.z = -0.025134429335594177;
        waypoints.push_back(p);

        p.position.x = 0.4205976724624634;
        p.position.y = 0.13388091325759888;
        p.position.z = -0.002856343984603882;
        waypoints.push_back(p);

        p.position.x = 0.28687524795532227;
        p.position.y = -0.0040201060473918915;
        p.position.z = 0.01621788740158081;
        waypoints.push_back(p);

        RCLCPP_INFO(LOGGER, "Loaded %zu waypoints with downward brush orientation", waypoints.size());
        return waypoints;
    }

    void performWaypointCleaningMotion()
    {
        RCLCPP_INFO(LOGGER, "Starting waypoint cleaning motion...");

        auto waypoints = loadWaypoints();

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.8)
        {
            RCLCPP_INFO(LOGGER, "Path computed successfully (%.2f%% achieved)", fraction * 100.0);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory = trajectory;

            auto result = arm_group_->execute(plan);
            if (result == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                RCLCPP_INFO(LOGGER, "Waypoint cleaning motion completed successfully!");
            }
            else
            {
                RCLCPP_ERROR(LOGGER, "Failed to execute cleaning motion");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Could not compute valid path (only %.2f%% achieved)", fraction * 100.0);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ToiletSurfaceClean>();

    try
    {
        node->arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "arm");

        RCLCPP_INFO(LOGGER, "Move group initialized!");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(LOGGER, "Failed to initialize move group: %s", e.what());
        return -1;
    }

    node->performWaypointCleaningMotion();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
