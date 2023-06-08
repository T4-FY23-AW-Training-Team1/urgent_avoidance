#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <motion_utils/motion_utils.hpp>
#include <acoustics_msgs/msg/sound_source_direction.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "goal_update_srvs/srv/find_road_shoulder_goal.hpp"
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

// These are defined in /api/operation_mode/state topic
#define STOP 1
#define AUTONOMOUS 2

// These are defined only in here
enum class AvoidanceState{
    BEAWARE = -1,
    SEARCHING_GOAL,
    REQUESTING_AVOIDANCE,
    AVOIDING,
    WAITING,
    REQUESTING_RECOVER
};

using FindRoadShoulderGoal = goal_update_srvs::srv::FindRoadShoulderGoal;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState; 
using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using motion_utils::findNearestIndex;

class UrgentAvoidancePlanner : public rclcpp::Node
{
    public:
        UrgentAvoidancePlanner() : Node("urgent_avoidance_planner")
        {
            // Initialize parameters
            is_manual_emergency_vehicle_approaching = false;
            is_emergency_vehicle_approaching = false;
            is_arrived = false;
            node_state = AvoidanceState::BEAWARE;
            last_current_pose_stamped_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
            goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
            if(goal_pose == nullptr){
                RCLCPP_INFO(this->get_logger(), "goal_pose is nullptr from the start");
            }

            this->declare_parameter<double>("search_starting_distance", 20.0);
            this->declare_parameter<double>("rear_angle_range_min", 120.0);
            this->declare_parameter<double>("rear_angle_range_max", 240.0);
            this->declare_parameter<double>("direction_duration_threshold", 3.0);

            this->get_parameter("search_starting_distance", search_starting_distance);
            this->get_parameter("rear_angle_range_min", rear_angle_range_min);
            this->get_parameter("rear_angle_range_max", rear_angle_range_max);
            this->get_parameter("direction_duration_threshold", direction_duration_threshold);

            // Initialize publisher and subscriber
            tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "~/input/tf", 10, std::bind(&UrgentAvoidancePlanner::tf_callback, this, std::placeholders::_1));

            //pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            //"~/input/pose", 10, std::bind(&UrgentAvoidancePlanner::onCurrentPoseStamped, this, std::placeholders::_1));

            map_bin_subscription_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
            "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
            std::bind(&UrgentAvoidancePlanner::onMapBin, this, std::placeholders::_1));

            trajectory_subscription_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "~/input/trajectory", 1,
            std::bind(&UrgentAvoidancePlanner::onTrajectory, this, std::placeholders::_1));

            service_ = this->create_service<FindRoadShoulderGoal>(
                "/provoke_avoidance",
                std::bind(&UrgentAvoidancePlanner::calc_goal_and_publish, this, std::placeholders::_1, std::placeholders::_2));

            sound_source_direction_subscription_ = this->create_subscription<acoustics_msgs::msg::SoundSourceDirection>(
            "~/input/sound_source_direction", 10, std::bind(&UrgentAvoidancePlanner::onSoundSourceDirection, this, std::placeholders::_1));

            operation_mode_subscription_ = this->create_subscription<OperationModeState>("/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
                std::bind(&UrgentAvoidancePlanner::onOperationMode, this, std::placeholders::_1));

            routing_state_subscription_ = this->create_subscription<RouteState>("/api/routing/state", rclcpp::QoS{1}.transient_local(),
            std::bind(&UrgentAvoidancePlanner::onRoutingState, this, std::placeholders::_1));

            expected_goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/input/goal", 3, std::bind(&UrgentAvoidancePlanner::onExpectedGoal, this, std::placeholders::_1));
            
            client_change_to_autonomous_ = this->create_client<ChangeOperationMode>(
                "/api/operation_mode/change_to_autonomous", rmw_qos_profile_services_default);

            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/goal", 10);

            // set timer callback
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UrgentAvoidancePlanner::timer_callback, this));
        }

    private:
        void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
            //RCLCPP_INFO(this->get_logger(), "TF is loaded\n");
            for (const auto& transform : msg->transforms){
                // Check if child_frame_id is "base_link"
                if (transform.child_frame_id == "base_link"){
                    // Extract header
                    last_current_pose_stamped_->header = transform.header;

                    // Extract transform
                    last_current_pose_stamped_->pose.position.x = transform.transform.translation.x;
                    last_current_pose_stamped_->pose.position.y = transform.transform.translation.y;
                    last_current_pose_stamped_->pose.position.z = transform.transform.translation.z;
                    last_current_pose_stamped_->pose.orientation = transform.transform.rotation;
                }
            }
        }
        /*
        void onCurrentPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            // Save the latest vehicle pose
            last_current_pose_stamped_ = msg;
        }
        */
        void onMapBin(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg){
            // get lanelets (in binary)
            lanelet::LaneletMapPtr ll_map(new lanelet::LaneletMap);
            lanelet::utils::conversion::fromBinMsg(*msg, ll_map);
            RCLCPP_INFO(this->get_logger(), "Map is loaded\n");

            // get shoulder lanelets
            all_lanelets = lanelet::utils::query::laneletLayer(ll_map);
            shoulder_lanelets = lanelet::utils::query::shoulderLanelets(all_lanelets);
            if(shoulder_lanelets.empty()){
                RCLCPP_WARN(this->get_logger(), "No road shoulder is found!!");
            }
        }

        void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg){
            // get latest trajectory plan
            last_trajectory_ = msg;
        }

        void onOperationMode(const OperationModeState::SharedPtr msg){
            // get latest operation mode state
            last_operation_mode_state_ = msg;
        }

        void onRoutingState(const RouteState::SharedPtr msg){
            // check whether the vehicle is arrived
            is_arrived = (msg->state == 3); // 3 means ARRIVED
        }

        void onSoundSourceDirection(const acoustics_msgs::msg::SoundSourceDirection::SharedPtr msg){
            // get latest sound source direction
            double angle = std::atan2(msg->unit_direction_y, msg->unit_direction_x) * 180 / M_PI;
            if(angle < 0)
                angle = angle + 360;

            // If the condition matches, set is_emergency_vehicle_approaching to true and set a new goal to the road shoulder
            if(angle >= rear_angle_range_min && angle <= rear_angle_range_max && msg->duration_time >= direction_duration_threshold){
                RCLCPP_INFO(this->get_logger(), "Emegency vehicle detected!!");
                is_emergency_vehicle_approaching = true;
                /*if(last_current_pose_stamped_){
                    geometry_msgs::msg::PoseStamped goal_pose;
                    if(set_goal_to_road_shoulder_ahead(*last_current_pose_stamped_, &goal_pose)){
                        RCLCPP_INFO(this->get_logger(), "Set goal to road shoulder ahead");
                        publisher_->publish(goal_pose);
                    }else{
                        RCLCPP_INFO(this->get_logger(), "No road shoulder found");
                    }
                }*/
            }
            else{
                is_emergency_vehicle_approaching = false;
            }
        }

        void onExpectedGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            // Save the latest goal pose (not the goal pose altered in here)
            // **Not watching modified goal in goal planner**
            if(goal_pose){
                if(goal_pose->header.stamp == msg->header.stamp){
                    RCLCPP_INFO(this->get_logger(), "Ignored goal publsihed from this node.");
                }else{
                    RCLCPP_INFO(this->get_logger(), "Received normal goal.");
                    last_expected_goal_pose_stamped_ = msg;
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "Received normal goal.");
                last_expected_goal_pose_stamped_ = msg;
            }
        }
        
        void timer_callback(){
            switch(node_state){
                case AvoidanceState::BEAWARE:
                    if(is_emergency_vehicle_approaching == true || is_manual_emergency_vehicle_approaching == true){
                        node_state = AvoidanceState::SEARCHING_GOAL;
                    }else{
                        break; // If the condition is met, the procedure can go next immediately
                    }

                case AvoidanceState::SEARCHING_GOAL:
                    if(is_emergency_vehicle_approaching == false && is_manual_emergency_vehicle_approaching == false){
                        node_state = AvoidanceState::BEAWARE; // Go back to BEAWARE if the emergency vehicle is gone
                        break;
                    }
                    if(last_current_pose_stamped_){
                        if(goal_pose){
                            if(set_goal_to_road_shoulder_ahead(*last_current_pose_stamped_, goal_pose)){
                                RCLCPP_INFO(this->get_logger(), "Set goal to road shoulder ahead");
                                publisher_->publish(*goal_pose);
                                node_state = AvoidanceState::REQUESTING_AVOIDANCE;
                            }else{
                                RCLCPP_INFO(this->get_logger(), "No road shoulder found");
                            }
                        }else{
                            RCLCPP_WARN(this->get_logger(), "goal_pose is empty");
                        }
                    }else{
                        RCLCPP_WARN(this->get_logger(), "No message (of current PoseStamped) received yet");
                    }
                    break; // Even if the condition is met, this node should wait a bit to let the vehicle stop. Hence no passing through like case BEAWARE.

                case AvoidanceState::REQUESTING_AVOIDANCE:
                    if(is_emergency_vehicle_approaching == false && is_manual_emergency_vehicle_approaching == false){
                        RCLCPP_INFO(this->get_logger(), "Emegency vehicle is gone. Go back to the original goal.");
                        node_state = AvoidanceState::REQUESTING_RECOVER; // Go back to REQUESTING_RECOVER if the emergency vehicle is gone
                        publisher_->publish(*last_expected_goal_pose_stamped_); // Set back the goal to the original goal
                        break;
                    }
                    if(last_operation_mode_state_->mode == STOP && last_operation_mode_state_->is_in_transition == false && last_operation_mode_state_->is_autonomous_mode_available == true){
                        RCLCPP_INFO(this->get_logger(), "Request mode change (avoidance)");
                        if (!client_change_to_autonomous_->service_is_ready()) {
                            RCLCPP_INFO(this->get_logger(), "client is unavailable");
                        }
                        else{
                            RCLCPP_INFO(this->get_logger(), "Changed to autonomous mode (avoidance)");
                            auto request = std::make_shared<ChangeOperationMode::Request>();
                            client_change_to_autonomous_->async_send_request(request);
                            node_state = AvoidanceState::AVOIDING;
                        }
                    }
                    break;

                case AvoidanceState::AVOIDING:
                    if(is_emergency_vehicle_approaching == false && is_manual_emergency_vehicle_approaching == false){
                        RCLCPP_INFO(this->get_logger(), "Emegency vehicle is gone. Go back to the original goal.");
                        node_state = AvoidanceState::REQUESTING_RECOVER; // Go back to REQUESTING_RECOVER if the emergency vehicle is gone
                        publisher_->publish(*last_expected_goal_pose_stamped_); // Set back the goal to the original goal
                        break;
                    }
                    if(is_arrived == true){
                        RCLCPP_INFO(this->get_logger(), "Pull over seems to be finished");
                        node_state = AvoidanceState::WAITING;
                        is_manual_emergency_vehicle_approaching = false;
                        publisher_->publish(*last_expected_goal_pose_stamped_); // Set back the goal to the original goal
                    }
                    break;

                case AvoidanceState::WAITING:
                    if(is_emergency_vehicle_approaching == false && is_manual_emergency_vehicle_approaching == false){
                        RCLCPP_INFO(this->get_logger(), "Emegency vehicle is gone. Go back to the original goal.");
                        node_state = AvoidanceState::REQUESTING_RECOVER;
                    }else{
                        RCLCPP_INFO(this->get_logger(), "Waiting for emergnecy vehicle to be gone...");
                    }
                    break;

                case AvoidanceState::REQUESTING_RECOVER:
                    if(is_emergency_vehicle_approaching == true || is_manual_emergency_vehicle_approaching == true){
                        RCLCPP_INFO(this->get_logger(), "Emegency vehicle detected. Wait for passing through again.");
                        node_state = AvoidanceState::WAITING;
                        break;
                    }
                    if(last_operation_mode_state_->mode == STOP && last_operation_mode_state_->is_in_transition == false && last_operation_mode_state_->is_autonomous_mode_available == true){
                        RCLCPP_INFO(this->get_logger(), "Request mode change (recover)");
                        if (!client_change_to_autonomous_->service_is_ready()) {
                            RCLCPP_INFO(this->get_logger(), "client is unavailable");
                        }
                        else{
                            RCLCPP_INFO(this->get_logger(), "Changed to autonomous mode (recover)");
                            auto request = std::make_shared<ChangeOperationMode::Request>();
                            client_change_to_autonomous_->async_send_request(request);
                            node_state = AvoidanceState::BEAWARE;
                        }
                    }
                    break;
            }
        }

        void calc_goal_and_publish(
            const std::shared_ptr<FindRoadShoulderGoal::Request> request,
            std::shared_ptr<FindRoadShoulderGoal::Response> response)
        {
            is_manual_emergency_vehicle_approaching = true;
            /*if(last_current_pose_stamped_)
            {
                request->current_pose = *last_current_pose_stamped_;
                //response->goal_pose = advance_one_meter(request->current_pose);
                //response->goal_pose = set_goal_to_closest_road_shoulder(request->current_pose);
                is_manual_emergency_vehicle_approaching = true;
                response->is_road_shoulder_found = set_goal_to_road_shoulder_ahead(request->current_pose, &(response->goal_pose));
                if(response->is_road_shoulder_found){
                    RCLCPP_INFO(this->get_logger(), "Received pose, calculating next goal");
                    publisher_->publish(response->goal_pose);
                    RCLCPP_INFO(this->get_logger(), "Objective goal calculated and sent");
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "No road shoulder was found around the vehicle trajectory");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No message (of current PoseStamped) received yet");
            }*/
        }

        geometry_msgs::msg::PoseStamped advance_one_meter(const geometry_msgs::msg::PoseStamped &pose_stamped) // To be removed
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

        geometry_msgs::msg::PoseStamped set_goal_to_closest_road_shoulder(const geometry_msgs::msg::PoseStamped &current_pose_stamped) // To be removed
        {
            // remove stamp
            //geometry_msgs::msg::Pose current_pose;
            geometry_msgs::msg::PoseStamped shoulder_pose_stamped;
            //current_pose.position = current_pose_stamped.pose.position;
            //current_pose.orientation = current_pose_stamped.pose.orientation;

            // get closest shoulder lanelet
            lanelet::ConstLanelet closest_shoulder_lanelet;
            if(lanelet::utils::query::getClosestLanelet(shoulder_lanelets, current_pose_stamped.pose, &closest_shoulder_lanelet)){
                shoulder_pose_stamped.pose = lanelet::utils::getClosestCenterPose(closest_shoulder_lanelet, current_pose_stamped.pose.position);
                shoulder_pose_stamped.header = current_pose_stamped.header;
            }

            return shoulder_pose_stamped;
        }

        bool set_goal_to_road_shoulder_ahead(const geometry_msgs::msg::PoseStamped &current_pose_stamped, geometry_msgs::msg::PoseStamped::SharedPtr shoulder_pose_stamped){
            double total_length = 0.0;

            shoulder_pose_stamped->header = current_pose_stamped.header;
            
            lanelet::ConstLanelet target_shoulder_lanelet;
            bool shoulderFound = false;

            RCLCPP_INFO(this->get_logger(), "i starts from %ld and ends at %ld", findNearestIndex(last_trajectory_->points, last_current_pose_stamped_->pose.position), last_trajectory_->points.size()-1);
            for (size_t i = findNearestIndex(last_trajectory_->points, last_current_pose_stamped_->pose.position); !shoulderFound && (i < last_trajectory_->points.size()-1); ++i) {
                const auto& point1 = last_trajectory_->points[i];
                const auto& point2 = last_trajectory_->points[i+1];

                double dx = point1.pose.position.x - point2.pose.position.x;
                double dy = point1.pose.position.y - point2.pose.position.y;
                double dz = point1.pose.position.z - point2.pose.position.z;

                total_length += std::sqrt(dx*dx + dy*dy + dz*dz);

                RCLCPP_INFO(this->get_logger(), "Current total_length has %lf meters (i = %ld out of %ld), against search_starting_distance %lf meters", total_length, i, last_trajectory_->points.size(), search_starting_distance);
                if(total_length >= search_starting_distance){
                    shoulderFound = lanelet::utils::query::getClosestLanelet(shoulder_lanelets, point2.pose, &target_shoulder_lanelet);
                    shoulder_pose_stamped->pose = lanelet::utils::getClosestCenterPose(target_shoulder_lanelet, point2.pose.position);
                }
            }
            return shoulderFound;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
        //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
        rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_bin_subscription_;
        rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscription_;
        rclcpp::Subscription<acoustics_msgs::msg::SoundSourceDirection>::SharedPtr sound_source_direction_subscription_;
        rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_subscription_;
        rclcpp::Subscription<RouteState>::SharedPtr routing_state_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr expected_goal_subscription_;
        rclcpp::Service<FindRoadShoulderGoal>::SharedPtr service_;
        rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_current_pose_stamped_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_expected_goal_pose_stamped_;
        geometry_msgs::msg::PoseStamped::SharedPtr goal_pose;
        OperationModeState::SharedPtr last_operation_mode_state_;
        autoware_auto_planning_msgs::msg::Trajectory::SharedPtr last_trajectory_;
        lanelet::ConstLanelets all_lanelets;
        lanelet::ConstLanelets shoulder_lanelets;
        bool is_emergency_vehicle_approaching;
        bool is_manual_emergency_vehicle_approaching;
        bool is_arrived;
        AvoidanceState node_state;
        // Defined in yaml
        double search_starting_distance;
        double rear_angle_range_min;
        double rear_angle_range_max;
        double direction_duration_threshold;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrgentAvoidancePlanner>());
    rclcpp::shutdown();
    return 0;
}