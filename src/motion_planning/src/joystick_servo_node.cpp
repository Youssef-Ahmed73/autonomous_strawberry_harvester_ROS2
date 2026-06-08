/**
 * @file joystick_servo_node.cpp
 * @brief Xbox 360 joystick → MoveIt Servo teleoperation node for the ASHR project.
 *
 * ── BUTTON / AXIS MAPPING (Xbox 360) ─────────────────────────────────────────
 *
 * AXES  (joy.axes[])
 * 0  Left  stick  horizontal  (left = +1, right = −1)
 * 1  Left  stick  vertical    (up   = +1, down  = −1)
 * 2  Left  trigger            (released = +1, pressed = −1)
 * 3  Right stick  horizontal  (left = +1, right = −1)
 * 4  Right stick  vertical    (up   = +1, down  = −1)
 * 5  Right trigger            (released = +1, pressed = −1)
 *
 * BUTTONS (joy.buttons[])
 * 0  A     (Toggle Iris)
 * 1  B     (Toggle Door)
 * 2  X     (Toggle Scissor)
 * 3  Y     (Toggle Frame)
 * 4  LB    (Joint 6 / Yaw +)
 * 5  RB    (Joint 6 / Yaw -)
 * 6  Back  (Toggle Mode)
 * 7  Start (Toggle Controllers & Start Servo)
 * 8  Xbox (guide)
 * 9  Left  stick press (Deadman)
 * 10 Right stick press
 *
 * @author  ASHR team
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

using namespace std::chrono_literals;

static constexpr bool DEADMAN_ENABLED = false;

static constexpr int AX_LEFT_X  = 0;
static constexpr int AX_LEFT_Y  = 1;
static constexpr int AX_LT      = 2;   
static constexpr int AX_RIGHT_X = 3;
static constexpr int AX_RIGHT_Y = 4;
static constexpr int AX_RT      = 5;   

static constexpr int BTN_A       = 0;
static constexpr int BTN_B       = 1;   
static constexpr int BTN_X       = 2;   
static constexpr int BTN_Y       = 3;
static constexpr int BTN_LB      = 4;   
static constexpr int BTN_RB      = 5;   
static constexpr int BTN_BACK    = 6;   
static constexpr int BTN_START   = 7;   
static constexpr int BTN_DEADMAN = 9;   

static const std::vector<std::string> JOINT_NAMES = {
  "joint_1", "joint_2", "joint_3",
  "joint_4", "joint_5", "joint_6"
};

static constexpr double SCISSOR_OPEN  = 0.5;
static constexpr double SCISSOR_CLOSE = 0.0;
static constexpr double DOOR_OPEN     = -1.57;
static constexpr double DOOR_CLOSE    = 0.0;
static constexpr double IRIS_OPEN     = 1.57;
static constexpr double IRIS_CLOSE    = 0.0;

static double deadzone(double value, double threshold = 0.10)
{
  if (std::abs(value) < threshold) return 0.0;
  return (value > 0.0)
    ? (value - threshold) / (1.0 - threshold)
    : (value + threshold) / (1.0 - threshold);
}

static double trigger_value(double raw)
{
  return (1.0 - raw) / 2.0;
}

class JoystickServoNode : public rclcpp::Node
{
public:
  JoystickServoNode()
  : Node("joystick_servo_node"),
    cartesian_mode_(true),
    use_base_frame_(true),
    servo_active_(false),
    scissor_open_(false),
    door_open_(false),
    iris_open_(false),
    prev_back_btn_(0),
    prev_y_btn_(0),
    prev_start_btn_(0),
    prev_x_btn_(0),
    prev_b_btn_(0),
    prev_a_btn_(0)
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

    ee_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ee_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SystemDefaultsQoS(),
      std::bind(&JoystickServoNode::joy_callback, this, std::placeholders::_1));

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/servo_node/start_servo");

    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    RCLCPP_INFO(this->get_logger(),
      "\n"
      "================================================\n"
      "  ASHR Joystick Servo Node ready\n"
      "  Mode  : CARTESIAN  |  Frame: BASE_LINK\n"
      "  Back  : toggle Cartesian / Joint mode\n"
      "  Y     : toggle command frame (base ↔ EEF)\n"
      "  X     : toggle Scissor\n"
      "  B     : toggle Door\n"
      "  A     : toggle Iris\n"
      "  LB/RB : Yaw (Cartesian) / Joint 6 (Jog)\n"
      "  Start : toggle servo/arm controllers & start\n"
      "%s"
      "================================================",
      DEADMAN_ENABLED ? "  LS    : DEADMAN – hold to send commands\n" : "");
  }

private:
  bool cartesian_mode_;    
  bool use_base_frame_;    
  bool servo_active_;

  bool scissor_open_;
  bool door_open_;
  bool iris_open_;

  int  prev_back_btn_;
  int  prev_y_btn_;
  int  prev_start_btn_;
  int  prev_x_btn_;
  int  prev_b_btn_;
  int  prev_a_btn_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ee_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr              servo_start_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (static_cast<int>(msg->axes.size())   <= AX_RT ||
        static_cast<int>(msg->buttons.size()) <= BTN_START)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Joy message has fewer axes/buttons than expected.");
      return;
    }

    if (DEADMAN_ENABLED && msg->buttons[BTN_DEADMAN] == 0)
    {
      return;  
    }

    if (msg->buttons[BTN_BACK] == 1 && prev_back_btn_ == 0)
    {
      cartesian_mode_ = !cartesian_mode_;
      RCLCPP_INFO(this->get_logger(),
        "Mode switched to: %s",
        cartesian_mode_ ? "CARTESIAN" : "JOINT JOG");
    }
    prev_back_btn_ = msg->buttons[BTN_BACK];

    if (msg->buttons[BTN_Y] == 1 && prev_y_btn_ == 0)
    {
      use_base_frame_ = !use_base_frame_;
      RCLCPP_INFO(this->get_logger(),
        "Cartesian frame switched to: %s",
        use_base_frame_ ? "base_link" : "link_6 (EEF)");
    }
    prev_y_btn_ = msg->buttons[BTN_Y];

    if (msg->buttons[BTN_START] == 1 && prev_start_btn_ == 0)
    {
      toggle_controllers();
    }
    prev_start_btn_ = msg->buttons[BTN_START];

    bool ee_changed = false;

    if (msg->buttons[BTN_X] == 1 && prev_x_btn_ == 0)
    {
      scissor_open_ = !scissor_open_;
      ee_changed = true;
    }
    prev_x_btn_ = msg->buttons[BTN_X];

    if (msg->buttons[BTN_B] == 1 && prev_b_btn_ == 0)
    {
      door_open_ = !door_open_;
      ee_changed = true;
    }
    prev_b_btn_ = msg->buttons[BTN_B];

    if (msg->buttons[BTN_A] == 1 && prev_a_btn_ == 0)
    {
      iris_open_ = !iris_open_;
      ee_changed = true;
    }
    prev_a_btn_ = msg->buttons[BTN_A];

    if (ee_changed)
    {
      publish_ee_state();
    }

    if (cartesian_mode_)
    {
      publish_twist(msg);
    }
    else
    {
      publish_joint_jog(msg);
    }
  }

  void toggle_controllers()
  {
    servo_active_ = !servo_active_;

    if (!switch_controller_client_->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(),
        "/controller_manager/switch_controller service not available.");
    }
    else
    {
      auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      
      if (servo_active_)
      {
        req->activate_controllers = {"servo_controller"};
        req->deactivate_controllers = {"arm_controller"};
      }
      else
      {
        req->activate_controllers = {"arm_controller"};
        req->deactivate_controllers = {"servo_controller"};
      }
      
      req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

      auto future = switch_controller_client_->async_send_request(req,
        [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture f)
        {
          auto res = f.get();
          if (res->ok)
          {
            RCLCPP_INFO(this->get_logger(), "Controllers switched successfully. Servo active: %d", servo_active_);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Failed to switch controllers.");
          }
        });
        
      (void)future;
    }

    call_start_servo();
  }

  void publish_ee_state()
  {
    auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    traj_msg->header.stamp = this->now();
    traj_msg->joint_names = {
      "Active_Scissors_Gear_Joint", 
      "Door_Joint", 
      "Iris_Active_Gear_Joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(3);
    
    point.positions[0] = scissor_open_ ? SCISSOR_OPEN : SCISSOR_CLOSE;
    point.positions[1] = door_open_    ? DOOR_OPEN    : DOOR_CLOSE;
    point.positions[2] = iris_open_    ? IRIS_OPEN    : IRIS_CLOSE;

    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 500000000; 

    traj_msg->points.push_back(point);
    ee_pub_->publish(std::move(traj_msg));
  }

  void publish_twist(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    twist_msg->header.stamp    = this->now();
    twist_msg->header.frame_id = use_base_frame_ ? "base_link" : "link_6";

    twist_msg->twist.linear.x =  deadzone(msg->axes[AX_LEFT_X]);
    twist_msg->twist.linear.y =  deadzone(msg->axes[AX_LEFT_Y]);
    twist_msg->twist.linear.z =  deadzone(msg->axes[AX_RIGHT_Y]);

    double lt = trigger_value(msg->axes[AX_LT]);
    double rt = trigger_value(msg->axes[AX_RT]);
    double pitch = deadzone(lt - rt);   

    double yaw_cmd = static_cast<double>(msg->buttons[BTN_LB] - msg->buttons[BTN_RB]);

    twist_msg->twist.angular.x =  deadzone(msg->axes[AX_RIGHT_X]);  
    twist_msg->twist.angular.y =  pitch;                             
    twist_msg->twist.angular.z =  yaw_cmd;   

    twist_pub_->publish(std::move(twist_msg));
  }

  void publish_joint_jog(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    auto jog_msg = std::make_unique<control_msgs::msg::JointJog>();

    jog_msg->header.stamp    = this->now();
    jog_msg->header.frame_id = "base_link";  

    double lt = trigger_value(msg->axes[AX_LT]);
    double rt = trigger_value(msg->axes[AX_RT]);
    
    double joint_6_cmd = static_cast<double>(msg->buttons[BTN_LB] - msg->buttons[BTN_RB]);

    std::vector<double> velocities = {
       deadzone(msg->axes[AX_LEFT_X]),   
       deadzone(msg->axes[AX_LEFT_Y]),   
       deadzone(msg->axes[AX_RIGHT_X]),  
       deadzone(msg->axes[AX_RIGHT_Y]),  
       deadzone(lt - rt),               
       joint_6_cmd,  
    };

    jog_msg->joint_names = JOINT_NAMES;
    jog_msg->velocities  = velocities;

    joint_pub_->publish(std::move(jog_msg));
  }

  void call_start_servo()
  {
    if (!servo_start_client_->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(),
        "/servo_node/start_servo service not available yet. "
        "Make sure the servo node is running.");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future  = servo_start_client_->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f)
      {
        auto response = f.get();
        if (response->success)
        {
          RCLCPP_INFO(this->get_logger(), "Servo service triggered successfully.");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
            "start_servo returned false: %s", response->message.c_str());
        }
      });

    (void)future;  
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node     = std::make_shared<JoystickServoNode>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}