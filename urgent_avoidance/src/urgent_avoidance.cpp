#include <iostream>
#include <memory>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "goal_update_srvs/srv/find_road_shoulder_goal.hpp"

using FindRoadShoulderGoal = goal_update_srvs::srv::FindRoadShoulderGoal;

class UrgentAvoidancePlanner : public rclcpp::Node
{
    public:
        UrgentAvoidancePlanner() : Node("urgent_avoidance_planner")
        {
            pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/input/pose", 10, std::bind(&UrgentAvoidancePlanner::messageCallback, this, std::placeholders::_1));

            map_bin_subscription_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
            "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
            std::bind(&UrgentAvoidancePlanner::onMapBin, this, std::placeholders::_1));

            service_ = this->create_service<FindRoadShoulderGoal>(
                "/provoke_avoidance",
                std::bind(&UrgentAvoidancePlanner::calc_goal_and_publish, this, std::placeholders::_1, std::placeholders::_2));

            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/goal", 10);
        }

    private:
        void messageCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // 受信したメッセージを一時的に保存
            last_received_msg_ = msg;
        }

        void onMapBin(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg){
            lanelet::LaneletMapPtr ll_map(new lanelet::LaneletMap);
            lanelet::utils::conversion::fromBinMsg(*msg, ll_map);
            RCLCPP_INFO(this->get_logger(), "Map is loaded\n");

            // get Lanelets
            all_lanelets = lanelet::utils::query::laneletLayer(ll_map);
            shoulder_lanelets = lanelet::utils::query::shoulderLanelets(all_lanelets);
        }

        void calc_goal_and_publish(
            const std::shared_ptr<FindRoadShoulderGoal::Request> request,
            std::shared_ptr<FindRoadShoulderGoal::Response> response)
        {
            if(last_received_msg_)
            {
                request->current_pose = *last_received_msg_;
                //response->goal_pose = advance_one_meter(request->current_pose);
                response->goal_pose = set_goal_to_closest_road_shoulder(request->current_pose);
                RCLCPP_INFO(this->get_logger(), "Received pose, calculating next goal");
                publisher_->publish(response->goal_pose);
                RCLCPP_INFO(this->get_logger(), "Objective goal calculated and sent");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No message (PoseStamped) received yet");
            }
        }

        geometry_msgs::msg::PoseStamped advance_one_meter(const geometry_msgs::msg::PoseStamped &pose_stamped)
        {
            // クォータニオンからオイラー角に変換
            tf2::Quaternion quaternion(
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w);
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);

            // 新しい座標を計算
            double new_x = pose_stamped.pose.position.x + 1.0 * cos(yaw);
            double new_y = pose_stamped.pose.position.y + 1.0 * sin(yaw);

            // 新しいPoseStampedオブジェクトを作成
            geometry_msgs::msg::PoseStamped new_pose_stamped;
            new_pose_stamped.header = pose_stamped.header; // 同じフレームIDとタイムスタンプを使用
            new_pose_stamped.pose.position.x = new_x;
            new_pose_stamped.pose.position.y = new_y;
            new_pose_stamped.pose.position.z = pose_stamped.pose.position.z;
            new_pose_stamped.pose.orientation = pose_stamped.pose.orientation; // 同じ向きを使用

            return new_pose_stamped;
        }

        geometry_msgs::msg::PoseStamped set_goal_to_closest_road_shoulder(const geometry_msgs::msg::PoseStamped &current_pose_stamped)
        {
            // remove stamp
            geometry_msgs::msg::Pose current_pose;
            geometry_msgs::msg::PoseStamped shoulder_pose_stamped;
            current_pose.position = current_pose_stamped.pose.position;
            current_pose.orientation = current_pose_stamped.pose.orientation;

            // get closest shoulder lanelet
            lanelet::ConstLanelet closest_shoulder_lanelet;
            if(lanelet::utils::query::getClosestLanelet(shoulder_lanelets, current_pose, &closest_shoulder_lanelet)){
                shoulder_pose_stamped.pose = lanelet::utils::getClosestCenterPose(closest_shoulder_lanelet, current_pose.position);
                shoulder_pose_stamped.header = current_pose_stamped.header;
            }

            return shoulder_pose_stamped;
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
        rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_bin_subscription_;
        rclcpp::Service<FindRoadShoulderGoal>::SharedPtr service_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_received_msg_;
        lanelet::ConstLanelets all_lanelets;
        lanelet::ConstLanelets shoulder_lanelets;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrgentAvoidancePlanner>());
    rclcpp::shutdown();
    return 0;
}