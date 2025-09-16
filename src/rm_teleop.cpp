#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

class JoyToTwist {
public:
  JoyToTwist(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // パラメータ
    pnh.param<std::string>("joy_topic",   joy_topic_,   "/joy");
    pnh.param<std::string>("twist_topic", twist_topic_, "/twist_cmd");
    pnh.param<std::string>("enable_topic", enable_topic_, "/twist_enable");

    pnh.param<double>("linear_scale", linear_scale_, 0.05);  // m/s
    pnh.param<double>("deadzone",     deadzone_,     0.1);
    pnh.param<int>("deadman_button",  deadman_button_, 4);   // L1

    // /twist_enable を送るボタン設定（-1で無効）
    pnh.param<int>("enable_button",   enable_button_,   1);  // 〇
    pnh.param<int>("disable_button",  disable_button_,  0);  // ×
    pnh.param<int>("toggle_button",   toggle_button_,   2);  // Start

    pub_twist_  = nh.advertise<geometry_msgs::Twist>(twist_topic_, 10);
    pub_enable_ = nh.advertise<std_msgs::Bool>(enable_topic_, 10);

    sub_ = nh.subscribe(joy_topic_, 10, &JoyToTwist::joyCb, this);
    rate_ = ros::Rate(100.0);  // 100Hz
  }

  void spin() {
    while (ros::ok()) {
      ros::spinOnce();
      handleEnableButtons();  // 先にON/OFF処理
      publishCmd();           // 次に速度コマンド
      rate_.sleep();
    }
  }

private:
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    last_joy_ = *msg;
    have_joy_ = true;

    // 初回だけ前回状態バッファを初期化（ボタン数に合わせる）
    if (!prev_buttons_initialized_) {
      prev_buttons_ = last_joy_.buttons;
      prev_buttons_initialized_ = true;
    }
  }

  // 立ち上がりエッジ検出（押していなかった→押した）
  bool edgePressed(int idx) {
    if (idx < 0) return false;
    if (!have_joy_) return false;
    if (idx >= (int)last_joy_.buttons.size()) return false;
    int cur = last_joy_.buttons[idx];
    int prev = (idx < (int)prev_buttons_.size()) ? prev_buttons_[idx] : 0;
    bool pressed = (cur == 1 && prev == 0);
    // prev を更新
    if (idx >= (int)prev_buttons_.size()) prev_buttons_.resize(idx+1, 0);
    prev_buttons_[idx] = cur;
    return pressed;
  }

  void handleEnableButtons() {
    if (!have_joy_) return;

    // トグル
    if (edgePressed(toggle_button_)) {
      enable_state_ = !enable_state_;
      std_msgs::Bool b; b.data = enable_state_;
      pub_enable_.publish(b);
      ROS_INFO_STREAM("twist_enable TOGGLE -> " << (enable_state_ ? "true" : "false"));
      return; // 同フレームで他のON/OFFは無視
    }

    // 明示ON
    if (edgePressed(enable_button_)) {
      enable_state_ = true;
      std_msgs::Bool b; b.data = true;
      pub_enable_.publish(b);
      ROS_INFO("twist_enable true");
    }

    // 明示OFF
    if (edgePressed(disable_button_)) {
      enable_state_ = false;
      std_msgs::Bool b; b.data = false;
      pub_enable_.publish(b);
      ROS_INFO("twist_enable false");
    }
  }

  void publishCmd() {
    geometry_msgs::Twist cmd;
    if (!have_joy_) {
      pub_twist_.publish(cmd);
      return;
    }

    // デッドマンが離されていたら停止（ゼロ送信）
    if (deadman_button_ >= 0 &&
        deadman_button_ < (int)last_joy_.buttons.size() &&
        last_joy_.buttons[deadman_button_] == 0) {
      pub_twist_.publish(cmd);
      return;
    }

    // 軸読み取り（コントローラに合わせて調整）
    double lx = axisValue(last_joy_, 0);  // 左stick左右
    double ly = axisValue(last_joy_, 1);  // 左stick上下
    double ry = axisValue(last_joy_, 4);  // 右stick上下

    cmd.linear.x =  linear_scale_ * ly;   // 前後
    cmd.linear.y =  linear_scale_ * lx;   // 左右
    cmd.linear.z =  linear_scale_ * ry;   // 上下
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    pub_twist_.publish(cmd);
  }

  double axisValue(const sensor_msgs::Joy& joy, int idx) const {
    if (idx >= 0 && idx < (int)joy.axes.size()) {
      double v = joy.axes[idx];
      return (std::fabs(v) > deadzone_) ? v : 0.0;
    }
    return 0.0;
  }

  ros::Publisher pub_twist_, pub_enable_;
  ros::Subscriber sub_;
  ros::Rate rate_{100.0};

  std::string joy_topic_{"/joy"}, twist_topic_{"/twist_cmd"}, enable_topic_{"/twist_enable"};
  double linear_scale_{0.05}, deadzone_{0.1};
  int deadman_button_{4};

  int enable_button_{5};   // RB
  int disable_button_{1};  // B / 〇
  int toggle_button_{7};   // Start

  bool enable_state_{false};

  sensor_msgs::Joy last_joy_;
  bool have_joy_{false};
  bool prev_buttons_initialized_{false};
  std::vector<int> prev_buttons_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_teleop");
  ros::NodeHandle nh, pnh("~");
  JoyToTwist node(nh, pnh);
  node.spin();
  return 0;
}
