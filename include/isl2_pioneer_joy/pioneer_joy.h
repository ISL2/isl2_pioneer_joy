#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <topic_tools/MuxSelect.h>

#define MUX_CMD_VEL_COUNT 3 // __NONE, JOY, MOVE_BASE
enum CmdMuxEnum
{
    NONE=0,
    JOY=1,
    MOVE_BASE=2
};

class PioneerTeleop
{
    public:
        // Constructor
        PioneerTeleop();
        std::map<CmdMuxEnum,std::string> cmd_vel_mux_topics;
        void initialize();
        // Destructor
        ~PioneerTeleop();

    private:
        void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
        void enable_motor(bool active);
        void deploy_charger(bool active);
        void cmd_vel_mux_select();
        void stop();
        void cancel();

    private:
        ros::NodeHandle     nh_;
        ros::Publisher      twist_pub_;
        ros::Publisher      move_base_cancel_pub_;
        ros::Subscriber     joy_sub_;
        ros::ServiceClient  enable_charge_client_;
        ros::ServiceClient  disable_charge_client_;
        ros::ServiceClient  enable_motor_client_;
        ros::ServiceClient  disable_motor_client_;
        ros::ServiceClient  cmd_vel_mux_switch_;

    private:
        uint    deadman_triggered_;
        uint    cancel_button_;
        uint    deploy_charger_button_;
        uint    enable_motor_button_;
        uint    cmd_vel_mux_select_button_;

    private:
        int     axis_linear_;
        int     axis_angular_;
        double  axis_linear_multiplier_;
        double  axis_angular_multiplier_;
        int     cur_cmd_vel_mux_idx_;
        bool    should_off_teleop_;

};