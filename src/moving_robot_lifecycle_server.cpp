#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_moving_robot_interfaces/action/location_speed.hpp"

using MovingRobot= my_moving_robot_interfaces::action::LocationSpeed;
using namespace std::placeholders;
using MovingRobotGoalHandle= rclcpp_action::ServerGoalHandle<MovingRobot>;
using MovingRobotLifecycle =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class MovingRobotServerNode : public rclcpp_lifecycle::LifecycleNode 
{

public:
    MovingRobotServerNode() :LifecycleNode("moving_robot_server")
    {
        RCLCPP_INFO(this->get_logger(),"IN on_constructor");

        robot_position_ = 50;
        robot_name_="";
        server_activated_=false;
        cb_group_=this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);

    }

    MovingRobotLifecycle on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"IN on_configure");
        this->declare_parameter("robot_name",rclcpp::PARAMETER_STRING);
        robot_name_=this->get_parameter("robot_name").as_string();
        moving_robot_server_=rclcpp_action::create_server<MovingRobot>(
            this,
            "moving_robot"+robot_name_,
            std::bind(&MovingRobotServerNode::goal_callback,this, _1,_2),
            std::bind(&MovingRobotServerNode::cancel_callback,this,_1),
            std::bind(&MovingRobotServerNode::handle_accepted_callback,this,_1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        return MovingRobotLifecycle::SUCCESS;

    }


    MovingRobotLifecycle on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(),"IN on_activate");
        server_activated_=true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return MovingRobotLifecycle::SUCCESS;
    }

    MovingRobotLifecycle on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(),"IN on_deactivate");
        server_activated_=false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_handle_)
            {
                if(goal_handle_->is_active())
                {
                    preempted_goal_id_=goal_handle_->get_goal_id();
                }
            }
        }
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return MovingRobotLifecycle::SUCCESS;
    }

    MovingRobotLifecycle on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(),"IN on_cleanup");
        (void)previous_state;
        robot_name_="";
        this->undeclare_parameter("robot_name");
        moving_robot_server_.reset();
        return MovingRobotLifecycle::SUCCESS;
    }

    MovingRobotLifecycle on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(),"IN on_shutdown");
        (void)previous_state;
        robot_name_="";
        this->undeclare_parameter("robot_name");
        moving_robot_server_.reset();
        return MovingRobotLifecycle::SUCCESS;

    }

private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MovingRobot::Goal> goal)
    {
        (void)uuid;
        //(void)goal;
        RCLCPP_INFO(this->get_logger(),"Received a goal");

        //check if activation is true
        if(!server_activated_)
        {
            RCLCPP_WARN(this->get_logger(),"Node is not activated yet");
        }

        // goal validation
        if ( (goal->position<0)||(goal->position>100)||(goal->velocity<=0))
        {
            RCLCPP_INFO(this->get_logger(), "Invalid position/velocity, reject goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        //preemtping after getting a valid goal
        {
            std::lock_guard<std::mutex>lock(mutex_);
            if(goal_handle_)
            {
                if(goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(),"Abort current goal and accept new goal");
                    preempted_goal_id_=goal_handle_->get_goal_id();
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),"Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MovingRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(),"Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(
        const std::shared_ptr<MovingRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MovingRobotGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex>lock(mutex_);
            this->goal_handle_=goal_handle;
        }
        //main logic here

        //get request from goal
        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;
        
        // execute the action
  
        auto result = std::make_shared<MovingRobot::Result>();
        auto feedback = std::make_shared<MovingRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        RCLCPP_INFO(this->get_logger(), "Execute goal");
        while (rclcpp::ok()) {
            // Check if needs to preempt goal
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position = robot_position_;
                    result->message = "Preempted by another goal";
                    goal_handle->abort(result);
                    return;
                }
            }

            // Check if cancel request
            if (goal_handle->is_canceling()) {
                result->position = robot_position_;
                if (goal_position == robot_position_) {
                   result->message = "Success";
                   goal_handle->succeed(result); 
                }
                else {
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                }
                return;
            }

            int diff = goal_position - robot_position_;

            if (diff == 0) {
                result->position = robot_position_;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            }
            else if (diff > 0) {
                if (diff < velocity) {
                    robot_position_ += diff;
                }
                else {
                    robot_position_ += velocity;
                }
            }
            else if (diff < 0) {
                if (abs(diff) < velocity) {
                    robot_position_ -= abs(diff);
                }
                else {
                    robot_position_ -= velocity;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
          
    }

    rclcpp_action::Server<MovingRobot>::SharedPtr moving_robot_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MovingRobotGoalHandle> goal_handle_;
    int robot_position_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    bool server_activated_;
    std::string robot_name_;
};


int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MovingRobotServerNode>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    //rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
