/**
 * @file joystick_servo_node.cpp
 * @brief Xbox 360 joystick → MoveIt Servo teleoperation node for the ASHR project.
 *
 * ── BUTTON / AXIS MAPPING (Xbox 360) ─────────────────────────────────────────
 *
 *  AXES  (joy.axes[])
 *    0  Left  stick  horizontal  (left = +1, right = −1)
 *    1  Left  stick  vertical    (up   = +1, down  = −1)
 *    2  Left  trigger            (released = +1, pressed = −1)
 *    3  Right stick  horizontal  (left = +1, right = −1)
 *    4  Right stick  vertical    (up   = +1, down  = −1)
 *    5  Right trigger            (released = +1, pressed = −1)
 *    6  D-pad horizontal         (left = +1, right = −1)
 *    7  D-pad vertical           (up   = +1, down  = −1)
 *
 *  BUTTONS (joy.buttons[])
 *    0  A
 *    1  B
 *    2  X
 *    3  Y
 *    4  LB
 *    5  RB
 *    6  Back
 *    7  Start
 *    8  Xbox (guide)
 *    9  Left  stick press
 *   10  Right stick press
 *
 * ── CONTROL SCHEME ───────────────────────────────────────────────────────────
 *
 *  Cartesian mode (default):
 *    Left  stick  X/Y → EEF linear  X / Y
 *    Right stick  Y   → EEF linear  Z
 *    Right stick  X   → EEF roll  (rotate around X)
 *    LT / RT          → EEF pitch  (LT = +, RT = −)   [triggers, mapped 0-1]
 *    D-pad  X         → EEF yaw   (rotate around Z)
 *    Frame toggle     → Y button   (base_link  ↔  link_6)
 *
 *  Joint mode:
 *    Left  stick  X   → joint_1
 *    Left  stick  Y   → joint_2
 *    Right stick  X   → joint_3
 *    Right stick  Y   → joint_4
 *    LT / RT          → joint_5   (LT = +, RT = −)
 *    D-pad  X         → joint_6
 *
 *  Both modes:
 *    A button  → toggle Cartesian / Joint mode   (prints to terminal)
 *    Y button  → toggle command frame: base_link ↔ link_6  (Cartesian only)
 *    Start     → call /servo_node/start_servo service  (convenience)
 *    Back      → deadman: hold while moving for an extra safety layer (optional,
 *                         see DEADMAN_ENABLED below)
 *
 * ── DEADMAN SWITCH ────────────────────────────────────────────────────────────
 *  Set DEADMAN_ENABLED = true to require the Back button to be held before any
 *  command is published.  Useful on real hardware.  Disabled by default so that
 *  simulation is easier to use.
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

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
//  Compile-time options
// ─────────────────────────────────────────────────────────────────────────────
static constexpr bool DEADMAN_ENABLED = false;   // set true for real hardware

// ─────────────────────────────────────────────────────────────────────────────
//  Xbox 360 axis indices
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int AX_LEFT_X  = 0;
static constexpr int AX_LEFT_Y  = 1;
static constexpr int AX_LT      = 2;   // released=+1, fully pressed=−1
static constexpr int AX_RIGHT_X = 3;
static constexpr int AX_RIGHT_Y = 4;
static constexpr int AX_RT      = 5;   // released=+1, fully pressed=−1
static constexpr int AX_DPAD_X  = 6;
// static constexpr int AX_DPAD_Y  = 7;  // reserved for future use

// ─────────────────────────────────────────────────────────────────────────────
//  Xbox 360 button indices
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int BTN_A     = 0;
static constexpr int BTN_B     = 1;   // reserved / future
static constexpr int BTN_X     = 2;   // reserved / future
static constexpr int BTN_Y     = 3;
static constexpr int BTN_LB    = 4;   // reserved / future
static constexpr int BTN_RB    = 5;   // reserved / future
static constexpr int BTN_BACK  = 6;   // deadman
static constexpr int BTN_START = 7;   // start servo service

// ─────────────────────────────────────────────────────────────────────────────
//  Joint names for the Probot arm (must match moveit_config SRDF / URDF)
// ─────────────────────────────────────────────────────────────────────────────
static const std::vector<std::string> JOINT_NAMES = {
  "joint_1", "joint_2", "joint_3",
  "joint_4", "joint_5", "joint_6"
};

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: apply a small dead-zone to an axis value
// ─────────────────────────────────────────────────────────────────────────────
static double deadzone(double value, double threshold = 0.10)
{
  if (std::abs(value) < threshold) return 0.0;
  // rescale so the output starts from 0 right at the threshold
  return (value > 0.0)
    ? (value - threshold) / (1.0 - threshold)
    : (value + threshold) / (1.0 - threshold);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: convert a trigger axis (released=+1, pressed=−1) → [0, 1]
// ─────────────────────────────────────────────────────────────────────────────
static double trigger_value(double raw)
{
  // raw ∈ [−1, +1]  →  pressed=1, released=0
  return (1.0 - raw) / 2.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Node class
// ─────────────────────────────────────────────────────────────────────────────
class JoystickServoNode : public rclcpp::Node
{
public:
  JoystickServoNode()
  : Node("joystick_servo_node"),
    cartesian_mode_(true),
    use_base_frame_(true),
    prev_a_btn_(0),
    prev_y_btn_(0),
    prev_start_btn_(0)
  {
    // ── Publishers ───────────────────────────────────────────────────────────
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());

    // ── Subscriber ───────────────────────────────────────────────────────────
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SystemDefaultsQoS(),
      std::bind(&JoystickServoNode::joy_callback, this, std::placeholders::_1));

    // ── Service client (to start servo on demand via Start button) ───────────
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/servo_node/start_servo");

    RCLCPP_INFO(this->get_logger(),
      "\n"
      "================================================\n"
      "  ASHR Joystick Servo Node ready\n"
      "  Mode  : CARTESIAN  |  Frame: BASE_LINK\n"
      "  A     : toggle Cartesian / Joint mode\n"
      "  Y     : toggle command frame (base ↔ EEF)\n"
      "  Start : call /servo_node/start_servo\n"
      "%s"
      "================================================",
      DEADMAN_ENABLED ? "  Back  : DEADMAN – hold to send commands\n" : "");
  }

private:
  // ── State ─────────────────────────────────────────────────────────────────
  bool cartesian_mode_;    // true = Cartesian, false = joint jog
  bool use_base_frame_;    // true = base_link,  false = link_6 (EEF)

  int  prev_a_btn_;
  int  prev_y_btn_;
  int  prev_start_btn_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr              servo_start_client_;

  // ─────────────────────────────────────────────────────────────────────────
  //  Main callback
  // ─────────────────────────────────────────────────────────────────────────
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Guard: make sure we have enough axes and buttons
    if (static_cast<int>(msg->axes.size())   <= AX_DPAD_X ||
        static_cast<int>(msg->buttons.size()) <= BTN_START)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Joy message has fewer axes/buttons than expected. "
        "Check that joy_node is publishing Xbox 360 data.");
      return;
    }

    // ── Deadman check ───────────────────────────────────────────────────────
    if (DEADMAN_ENABLED && msg->buttons[BTN_BACK] == 0)
    {
      return;  // do not publish anything unless deadman is held
    }

    // ── Toggle: A button → switch Cartesian / Joint mode (rising edge) ──────
    if (msg->buttons[BTN_A] == 1 && prev_a_btn_ == 0)
    {
      cartesian_mode_ = !cartesian_mode_;
      RCLCPP_INFO(this->get_logger(),
        "Mode switched to: %s",
        cartesian_mode_ ? "CARTESIAN" : "JOINT JOG");
    }
    prev_a_btn_ = msg->buttons[BTN_A];

    // ── Toggle: Y button → switch command frame (rising edge) ───────────────
    if (msg->buttons[BTN_Y] == 1 && prev_y_btn_ == 0)
    {
      use_base_frame_ = !use_base_frame_;
      RCLCPP_INFO(this->get_logger(),
        "Cartesian frame switched to: %s",
        use_base_frame_ ? "base_link" : "link_6 (EEF)");
    }
    prev_y_btn_ = msg->buttons[BTN_Y];

    // ── Start button → call start_servo service (rising edge) ───────────────
    if (msg->buttons[BTN_START] == 1 && prev_start_btn_ == 0)
    {
      call_start_servo();
    }
    prev_start_btn_ = msg->buttons[BTN_START];

    // ── Dispatch to Cartesian or Joint publisher ─────────────────────────────
    if (cartesian_mode_)
    {
      publish_twist(msg);
    }
    else
    {
      publish_joint_jog(msg);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Publish a TwistStamped (Cartesian servo)
  // ─────────────────────────────────────────────────────────────────────────
  void publish_twist(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // A fresh timestamp is required by Servo on every message.
    twist_msg->header.stamp    = this->now();
    twist_msg->header.frame_id = use_base_frame_ ? "base_link" : "link_6";

    // ── Linear velocities ──────────────────────────────────────────────────
    //  Left stick:  X → EEF X,  Y → EEF Y
    //  Right stick: Y → EEF Z
    twist_msg->twist.linear.x =  deadzone(msg->axes[AX_LEFT_X]);
    twist_msg->twist.linear.y =  deadzone(msg->axes[AX_LEFT_Y]);
    twist_msg->twist.linear.z =  deadzone(msg->axes[AX_RIGHT_Y]);

    // ── Angular velocities ─────────────────────────────────────────────────
    //  LT / RT      → pitch  (LT = positive, RT = negative)
    //  Right stick X → roll
    //  D-pad X       → yaw
    double lt = trigger_value(msg->axes[AX_LT]);
    double rt = trigger_value(msg->axes[AX_RT]);
    double pitch = deadzone(lt - rt);   // net pitch command

    twist_msg->twist.angular.x =  deadzone(msg->axes[AX_RIGHT_X]);  // roll
    twist_msg->twist.angular.y =  pitch;                             // pitch
    twist_msg->twist.angular.z =  deadzone(msg->axes[AX_DPAD_X]);   // yaw

    twist_pub_->publish(std::move(twist_msg));
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Publish a JointJog (joint-space servo)
  // ─────────────────────────────────────────────────────────────────────────
  void publish_joint_jog(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    auto jog_msg = std::make_unique<control_msgs::msg::JointJog>();

    // A fresh timestamp is required by Servo on every message.
    jog_msg->header.stamp    = this->now();
    jog_msg->header.frame_id = "base_link";  // not used by Servo for joint cmds

    // Build velocity commands for all 6 joints.
    //
    //  joint_1 ← left  stick X
    //  joint_2 ← left  stick Y
    //  joint_3 ← right stick X
    //  joint_4 ← right stick Y
    //  joint_5 ← LT(+) / RT(−)
    //  joint_6 ← D-pad X
    double lt = trigger_value(msg->axes[AX_LT]);
    double rt = trigger_value(msg->axes[AX_RT]);

    std::vector<double> velocities = {
       deadzone(msg->axes[AX_LEFT_X]),   // joint_1
       deadzone(msg->axes[AX_LEFT_Y]),   // joint_2
       deadzone(msg->axes[AX_RIGHT_X]),  // joint_3
       deadzone(msg->axes[AX_RIGHT_Y]),  // joint_4
       deadzone(lt - rt),               // joint_5
       deadzone(msg->axes[AX_DPAD_X]),  // joint_6
    };

    jog_msg->joint_names = JOINT_NAMES;
    jog_msg->velocities  = velocities;

    joint_pub_->publish(std::move(jog_msg));
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Call /servo_node/start_servo
  // ─────────────────────────────────────────────────────────────────────────
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
          RCLCPP_INFO(this->get_logger(), "Servo started successfully.");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
            "start_servo returned false: %s", response->message.c_str());
        }
      });

    (void)future;  // suppress nodiscard warning
  }
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor lets the service callback and the joy subscriber
  // run concurrently without blocking each other.
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node     = std::make_shared<JoystickServoNode>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
