#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_position_ik.hpp>

class MoveItTest : public rclcpp::Node{
public: 
  MoveItTest() : Node("moveit_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {

    ik_service_node_ = rclcpp::Node::make_shared("ik_srv_node");
    get_ik_cient_ = ik_service_node_->create_client
      <moveit_msgs::srv::GetPositionIK>("compute_ik");

    t.reset(new std::thread(std::bind(&MoveItTest::run,this)));
  }

  ~MoveItTest(){
    t->join();
  }

  void run(){
    rclcpp::sleep_for(std::chrono::seconds(5));

    move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(
      this->shared_from_this(),"manipulator"));
    
    geometry_msgs::msg::Pose start_pose_;
    start_pose_.position.x = 0.5452804;
    start_pose_.position.y = 0.3748915;
    start_pose_.position.z = 0.3953994;
    start_pose_.orientation.x = 0.0;
    start_pose_.orientation.y = 0.70710678118;
    start_pose_.orientation.z = 0.0;
    start_pose_.orientation.w = 0.70710678118;

    move_group_interface_->setPoseTarget(start_pose_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(move_group_interface_->plan(plan));
    
    RCLCPP_ERROR(this->get_logger(), "\nJ0 pos from last point in traj: %.7lf",
      plan.trajectory_.joint_trajectory.points.back().positions[0]);

    moveit_msgs::srv::GetPositionIK::Request::SharedPtr req;
      req.reset(new moveit_msgs::srv::GetPositionIK::Request());
      req->ik_request.pose_stamped.pose = start_pose_;
      req->ik_request.group_name = "manipulator";
      auto future = get_ik_cient_->async_send_request(req);

      try {
        if (rclcpp::spin_until_future_complete(ik_service_node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
          auto resp = future.get();
          int succ = resp->error_code.val;
          auto solution_ = resp->solution.joint_state.position[0];
          RCLCPP_ERROR(this->get_logger(), "j0 pos from ik srv: %.7lf", solution_);
        }
      }
      catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "IK Service failed: " << e.what());
        return;
      }
  }

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
  std::shared_ptr<std::thread> t;
  rclcpp::Node::SharedPtr ik_service_node_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr get_ik_cient_; 
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto obj = std::make_shared<MoveItTest>();
  rclcpp::spin(obj);

  return 0;
}
