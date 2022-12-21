#include "isl2_pioneer_joy/pioneer_joy.h"

// Class Constructor
PioneerTeleop::PioneerTeleop() : nh_("~"),
                                 axis_linear_(1),
                                 axis_angular_(2),
                                 axis_linear_multiplier_(1.0),
                                 axis_angular_multiplier_(1.0),
                                 cur_cmd_vel_mux_idx_(0),
                                 should_off_teleop_(false)
{
    // ROS Param Retrieval
    nh_.param("axis_linear", axis_linear_, axis_linear_);
    nh_.param("axis_angular", axis_angular_, axis_angular_);
    nh_.param("scale_linear", axis_linear_multiplier_, axis_linear_multiplier_);
    nh_.param("scale_angular", axis_angular_multiplier_, axis_angular_multiplier_);

    // Publisher
    twist_pub_              = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    move_base_cancel_pub_   = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    // Subscriber
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
        "/joy",
        10,
        &PioneerTeleop::joy_callback,
        this
    );

    // Service Client
    enable_charge_client_   = nh_.serviceClient<std_srvs::Empty>("/RosAria/enable_charge_pin");
    disable_charge_client_  = nh_.serviceClient<std_srvs::Empty>("/RosAria/disable_charge_pin");
    enable_motor_client_    = nh_.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
    disable_motor_client_   = nh_.serviceClient<std_srvs::Empty>("/RosAria/disable_motors");
    cmd_vel_mux_switch_     = nh_.serviceClient<topic_tools::MuxSelect>("/mux_cmd_vel/select");

    // Initialize
    initialize();
}

PioneerTeleop::~PioneerTeleop()
{
    ROS_INFO("Send Stop Command");
    stop();
}

/**
 * Write an image to disk.
 */
void PioneerTeleop::initialize()
{
    ROS_INFO("[PioneerJoy] : Begin Initialization !");
    // Joy Mode Topic Definition
    cmd_vel_mux_topics[NONE]        = "__none";
    cmd_vel_mux_topics[JOY]         = "/joy/cmd_vel";
    cmd_vel_mux_topics[MOVE_BASE]   = "/move_base/cmd_vel";
    // Create Request to swap mux
    topic_tools::MuxSelect mux_select;
    mux_select.request.topic = cmd_vel_mux_topics[NONE];
    cmd_vel_mux_switch_.call(mux_select);
    // Report
    ROS_INFO_STREAM(
        "[PioneerJoy] Setup List - - -" << std::endl <<
        "- Axis Linear             : " << axis_linear_ << std::endl <<
        "- Axis Linear Multiplier  : " << axis_linear_multiplier_ << std::endl <<
        "- Axis Angular            : " << axis_angular_ << std::endl <<
        "- Axis Angular Multiplier : " << axis_angular_multiplier_ << std::endl <<
        "- Current Nav Idx         : " << cur_cmd_vel_mux_idx_ << std::endl <<
        "-----------------------------" << std::endl
    );
    ROS_INFO("[PioneerJoy] : Initialize Completed !");
}

void PioneerTeleop::stop()
{
    ROS_INFO("[PioneerJoy] : Stop Robot");
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twist_pub_.publish(twist);
}

void PioneerTeleop::cmd_vel_mux_select()
{
    cur_cmd_vel_mux_idx_ += 1;
    cur_cmd_vel_mux_idx_ %= MUX_CMD_VEL_COUNT;

    // Create Request to swap mux
    topic_tools::MuxSelect mux_select;
    switch (cur_cmd_vel_mux_idx_)
    {
        case NONE:
            mux_select.request.topic = cmd_vel_mux_topics[NONE];
            break;
        case JOY:
            mux_select.request.topic = cmd_vel_mux_topics[JOY];
            break;
        case MOVE_BASE:
            mux_select.request.topic = cmd_vel_mux_topics[MOVE_BASE];
            break;
    }
    // Call Service with this request
    if(cmd_vel_mux_switch_.call(mux_select))
    {
        ROS_INFO("[PioneerJoy] : Switched cmd_vel topic to \"%s\"", mux_select.request.topic.c_str());
    }
    else
    {
        ROS_ERROR("[PioneerJoy] : Attempt to switch cmd_vel topic to \"%s\" failed!", mux_select.request.topic.c_str());
    }
}

void PioneerTeleop::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    // Get the Button Data
    deadman_triggered_         = msg->buttons[4];
    cancel_button_             = msg->buttons[6];
    cmd_vel_mux_select_button_ = msg->buttons[2];
    deploy_charger_button_     = msg->buttons[8];
    enable_motor_button_       = msg->buttons[7];

    // Behavior [Nav Mode]
    if(cmd_vel_mux_select_button_)
    {
        stop();
        cmd_vel_mux_select();
        return;
    }

    // Behavior [Charger]
    if(deploy_charger_button_ == 1)
    {
        stop();
        deploy_charger(true);
    }

    // Behavior [Cancel]
    if(cancel_button_ == 1)
    {
        ROS_INFO("[PioneerJoy] : Cancelling via /move_base/cancel ");
        stop();
        cancel();
        return;
    }

    // Behavior [Motor Enabling]
    if (enable_motor_button_ == 1)
    {
        enable_motor(true);
    }

    // Behavior [Driving]
    geometry_msgs::Twist twist;
    if(deadman_triggered_ == 1)
    {
        ROS_INFO("[PioneerJoy] : Teleop On!");
        should_off_teleop_ = false;
        // Scaling
        twist.angular.z = axis_angular_multiplier_ * float(msg->axes[axis_angular_]);
        twist.linear.x  = axis_linear_multiplier_  * float(msg->axes[axis_linear_]);
        twist_pub_.publish(twist);
    }
    else if (deadman_triggered_ == 0)
    {
        // Off Teleop when deadman is not trigger
        if(!should_off_teleop_){
            stop();
            should_off_teleop_ = true;
        }
        else{
            ROS_INFO("[PioneerJoy] : Teleop Off !");
            stop();
        }
    }
}


void PioneerTeleop::deploy_charger(bool active)
{
  std_srvs::Empty payload;
  bool result = (active) ? enable_charge_client_.call(payload) : disable_charge_client_.call(payload);
  ROS_INFO("Call Pin : %d", active);
  if (result)
  {
    ROS_INFO("Call Pin %d Service success", active);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
  }
}

void PioneerTeleop::enable_motor(bool active)
{
  std_srvs::Empty payload;
  bool result = (active) ? enable_motor_client_.call(payload) : disable_motor_client_.call(payload);
  ROS_INFO("Call Motor : %d", active);
  if (result)
  {
    ROS_INFO("Call Motor %d Service success", active);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
  }
}

void PioneerTeleop::cancel()
{
  deploy_charger(false);
  enable_motor(false);
  move_base_cancel_pub_.publish(*new actionlib_msgs::GoalID());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_teleop");
  PioneerTeleop pioneer_teleop;
  ros::spin();
}
