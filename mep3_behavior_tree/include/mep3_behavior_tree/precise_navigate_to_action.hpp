// Copyright 2021 Memristor Robotics
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

#ifndef MEP3_BEHAVIOR_TREE__PRECISE_NAVIGATE_TO_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__PRECISE_NAVIGATE_TO_ACTION_HPP_

#include <string>

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_behavior_tree/pose_2d.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mep3_behavior_tree
{

    class PreciseNavigateToAction
        : public mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
    {
    public:
        explicit PreciseNavigateToAction(
            const std::string &name,
            const BT::NodeConfiguration &config)
            : mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(
                  name,
                  config,
                  "precise_navigate_to_pose")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<BT::Pose2D>("goal")};
        }
    };

    void PreciseNavigateToAction::on_tick()
    {
        BT::Pose2D goal;
        getInput("goal", goal);

        goal_.pose.header.frame_id = "map";
        goal_.pose.header.stamp = node_->get_clock()->now();

        // Position
        goal_.pose.pose.position.x = goal.x;
        goal_.pose.pose.position.y = goal.y;

        // Orientation (yaw)
        // https://math.stackexchange.com/a/1972382
        goal_.pose.pose.orientation.w = std::cos(goal.theta / 2.0);
        goal_.pose.pose.orientation.z = std::sin(goal.theta / 2.0);
    }

    BT::NodeStatus PreciseNavigateToAction::on_success()
    {
        std::cout << "Navigation succesful " << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__PRECISE_NAVIGATE_TO_ACTION_HPP_