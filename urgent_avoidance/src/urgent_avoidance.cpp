#include <iostream>
#include <memory>

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
            subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/input/pose", 10, std::bind(&UrgentAvoidancePlanner::messageCallback, this, std::placeholders::_1));

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

        void calc_goal_and_publish(
            const std::shared_ptr<FindRoadShoulderGoal::Request> request,
            std::shared_ptr<FindRoadShoulderGoal::Response> response)
        {
            if(last_received_msg_)
            {
                request->current_pose = *last_received_msg_;
                response->goal_pose = advance_one_meter(request->current_pose);
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

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
        rclcpp::Service<FindRoadShoulderGoal>::SharedPtr service_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_received_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrgentAvoidancePlanner>());
    rclcpp::shutdown();
    return 0;
}