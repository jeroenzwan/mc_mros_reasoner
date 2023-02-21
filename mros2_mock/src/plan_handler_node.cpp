// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mros2_msgs/action/execute_plan.hpp"
#include<ctime>

using namespace std::chrono_literals;

class PlanHandlerServer : public rclcpp::Node
{
public:
  using ExecutePlan = mros2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;

  PlanHandlerServer()
  : rclcpp::Node("plan_handler_server_node")
  {
    using namespace std::placeholders;

    server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->action_server = rclcpp_action::create_server<ExecutePlan>(
      this,
      "plan_handler",
      std::bind(&PlanHandlerServer::handle_goal, this, _1, _2),
      std::bind(&PlanHandlerServer::handle_cancel, this, _1),
      std::bind(&PlanHandlerServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      server_cb_group_);
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"bluerov", "uuv"});
    problem_expert_->addInstance(plansys2::Instance{"pl1", "pipeline"});

    problem_expert_->addInstance(plansys2::Instance{"f_action", "function"});
    problem_expert_->addInstance(plansys2::Instance{"fd_recharge", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_search_pipeline", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_follow_pipeline", "functiondesign"});

    problem_expert_->addInstance(plansys2::Instance{"f_maintain_motion", "function"});
    problem_expert_->addInstance(plansys2::Instance{"fd_set_speed_high", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_set_speed_medium", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_set_speed_low", "functiondesign"});

    problem_expert_->addInstance(plansys2::Instance{"f_go_to_recharge_waypoints", "function"});
    problem_expert_->addInstance(plansys2::Instance{"fd_generate_recharge_wp", "functiondesign"});

    problem_expert_->addInstance(plansys2::Instance{"f_search_pipeline_waypoints", "function"});
    problem_expert_->addInstance(plansys2::Instance{"fd_spiral_high", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_spiral_medium", "functiondesign"});
    problem_expert_->addInstance(plansys2::Instance{"fd_spiral_low", "functiondesign"});

    problem_expert_->addInstance(plansys2::Instance{"f_follow_pipeline_waypoints", "function"});
    problem_expert_->addInstance(plansys2::Instance{"fd_generate_follow_wp", "functiondesign"});
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp_action::Server<ExecutePlan>::SharedPtr action_server;
  rclcpp::CallbackGroup::SharedPtr server_cb_group_;
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PlanHandlerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    problem_expert_->setGoal(plansys2::Goal{"(and(pipeline_inspected pl1))"});

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);
    RCLCPP_INFO(this->get_logger(), "After plan generation");

    auto result = std::make_shared<ExecutePlan::Result>();


    // plansys2_msgs::srv::GetPlan plan = 
    RCLCPP_INFO(this->get_logger(), typeid(plan).name());

    for (auto const& item : plan.value().items) {
      RCLCPP_INFO(this->get_logger(), item.action.c_str());
      RCLCPP_INFO(this->get_logger(), "duration is %f", item.duration);
      RCLCPP_INFO(this->get_logger(), "time is %f", item.time);
    }
    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return;
    }
    
    if (goal_handle->get_goal()->change_plan) {
      RCLCPP_INFO(this->get_logger(), "Canceling plan");
      executor_client_->cancel_plan_execution();
      rclcpp::sleep_for(1s);
    }

    // Execute the plan
    if (executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_INFO(this->get_logger(), "Executing plan");
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanHandlerServer>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
