#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <geometry_msgs/msg/point.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rebotarm_msgs/srv/check_trajectory_clearance.hpp>

namespace
{
constexpr std::size_t kMaxStates = 120;

struct ClearanceSample
{
  bool available{ false };
  double distance_m{ 0.0 };
  bool collision{ false };
  std::string body_1;
  std::string body_2;
  std::string source;
  geometry_msgs::msg::Point nearest_point_1;
  geometry_msgs::msg::Point nearest_point_2;
};

bool usable(const collision_detection::DistanceResult& result)
{
  return std::isfinite(result.minimum_distance.distance);
}

ClearanceSample selectMinimum(
  const collision_detection::DistanceResult& world,
  const collision_detection::DistanceResult& self)
{
  const bool have_world = usable(world);
  const bool have_self = usable(self);
  if (!have_world && !have_self)
    return {};

  const bool use_self =
    have_self &&
    (!have_world ||
     self.minimum_distance.distance < world.minimum_distance.distance);
  const auto& result = use_self ? self : world;
  const auto& minimum = result.minimum_distance;

  ClearanceSample sample;
  sample.available = true;
  sample.distance_m = minimum.distance;
  sample.collision = result.collision || minimum.distance <= 0.0;
  sample.body_1 = minimum.link_names[0];
  sample.body_2 = minimum.link_names[1];
  sample.source = use_self ? "self" : "world";
  sample.nearest_point_1.x = minimum.nearest_points[0].x();
  sample.nearest_point_1.y = minimum.nearest_points[0].y();
  sample.nearest_point_1.z = minimum.nearest_points[0].z();
  sample.nearest_point_2.x = minimum.nearest_points[1].x();
  sample.nearest_point_2.y = minimum.nearest_points[1].y();
  sample.nearest_point_2.z = minimum.nearest_points[1].z();
  return sample;
}
}  // namespace

class ClearanceServiceNode : public rclcpp::Node
{
public:
  using Service = rebotarm_msgs::srv::CheckTrajectoryClearance;

  ClearanceServiceNode()
  : Node("rebotarm_clearance_service")
  {
  }

  void initialize()
  {
    const auto robot_description =
      declare_parameter<std::string>("robot_description_parameter",
                                     "robot_description");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto planning_scene_topic =
      declare_parameter<std::string>("planning_scene_topic",
                                     "/monitored_planning_scene");
    const auto get_scene_service =
      declare_parameter<std::string>("get_planning_scene_service",
                                     "/get_planning_scene");

    planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), robot_description);
    if (!planning_scene_monitor_->getPlanningScene())
      throw std::runtime_error("failed to initialize MoveIt planning scene");

    planning_scene_monitor_->startStateMonitor(joint_states_topic);
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic);
    planning_scene_monitor_->startWorldGeometryMonitor();
    if (!planning_scene_monitor_->requestPlanningSceneState(get_scene_service))
    {
      RCLCPP_WARN(get_logger(),
                  "initial planning scene request failed; waiting for scene "
                  "monitor updates");
    }

    service_ = create_service<Service>(
      "/rebotarm/check_trajectory_clearance",
      std::bind(&ClearanceServiceNode::check, this, std::placeholders::_1,
                std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "full-robot trajectory clearance service ready (max %zu states)",
                kMaxStates);
  }

private:
  static collision_detection::DistanceRequest makeRequest(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& group_name)
  {
    collision_detection::DistanceRequest request;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.type = collision_detection::DistanceRequestType::GLOBAL;
    request.acm = &scene->getAllowedCollisionMatrix();
    request.group_name = group_name;
    if (!group_name.empty())
      request.enableGroup(scene->getRobotModel());
    return request;
  }

  void check(const std::shared_ptr<Service::Request> request,
             std::shared_ptr<Service::Response> response)
  {
    if (request->states.empty())
    {
      response->success = true;
      response->message = "trajectory has no sampled states";
      return;
    }
    if (request->states.size() > kMaxStates)
    {
      response->message = "request exceeds the 120-state safety limit";
      return;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(
      planning_scene_monitor_);
    if (!request->group_name.empty() &&
        !scene->getRobotModel()->hasJointModelGroup(request->group_name))
    {
      response->message = "unknown MoveIt group: " + request->group_name;
      return;
    }

    const auto distance_request = makeRequest(scene, request->group_name);
    response->available.reserve(request->states.size());
    response->distance_m.reserve(request->states.size());
    response->collision.reserve(request->states.size());
    response->body_1.reserve(request->states.size());
    response->body_2.reserve(request->states.size());
    response->source.reserve(request->states.size());
    response->nearest_point_1.reserve(request->states.size());
    response->nearest_point_2.reserve(request->states.size());

    for (std::size_t index = 0; index < request->states.size(); ++index)
    {
      const auto& joint_state = request->states[index];
      if (joint_state.name.empty() ||
          joint_state.name.size() != joint_state.position.size())
      {
        response->message =
          "invalid joint state at sample " + std::to_string(index);
        return;
      }
      std::unordered_set<std::string> seen;
      moveit::core::RobotState state = scene->getCurrentState();
      for (std::size_t joint = 0; joint < joint_state.name.size(); ++joint)
      {
        const auto& name = joint_state.name[joint];
        const double value = joint_state.position[joint];
        if (!seen.insert(name).second ||
            !scene->getRobotModel()->hasVariable(name) ||
            !std::isfinite(value))
        {
          response->message =
            "unknown, duplicate or non-finite joint at sample " +
            std::to_string(index) + ": " + name;
          return;
        }
        state.setVariablePosition(name, value);
      }
      state.update();

      collision_detection::DistanceResult world_result;
      scene->getCollisionEnv()->distanceRobot(
        distance_request, world_result, state);

      collision_detection::DistanceResult self_result;
      scene->getCollisionEnvUnpadded()->distanceSelf(
        distance_request, self_result, state);

      const auto sample = selectMinimum(world_result, self_result);
      response->available.push_back(sample.available);
      response->distance_m.push_back(sample.distance_m);
      response->collision.push_back(sample.collision);
      response->body_1.push_back(sample.body_1);
      response->body_2.push_back(sample.body_2);
      response->source.push_back(sample.source);
      response->nearest_point_1.push_back(sample.nearest_point_1);
      response->nearest_point_2.push_back(sample.nearest_point_2);
    }

    response->success = true;
    response->message =
      "full-robot clearance computed for " +
      std::to_string(request->states.size()) + " sampled states";
  }

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Service<Service>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try
  {
    auto node = std::make_shared<ClearanceServiceNode>();
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("rebotarm_clearance_service"),
                 "%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
