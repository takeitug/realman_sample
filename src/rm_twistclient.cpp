// rm_twistclient.cpp (velocity enable + watchdog no-publish + init on enable + param mutex)
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <rm_msgs/JointPos.h>
#include <std_msgs/Bool.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>
#include <map>
#include <fstream>
#include <iomanip>
#include <limits>
#include <algorithm>

#define PI 3.14159265358979323846

class TwistToJointPos {
public:
  TwistToJointPos(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // params
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
    pnh_.param<std::string>("twist_topic",        twist_topic_,        "/twist_cmd");
    pnh_.param<std::string>("out_topic",          out_topic_,          "/rm_driver/JointPos");
    pnh_.param<std::string>("enable_topic",       enable_topic_,       "/twist_enable");
    pnh_.param<std::string>("base_link", base_link_, "base_link");
    pnh_.param<std::string>("tip_link",  tip_link_,  "Link6");
    pnh_.param<double>("rate_hz",      rate_hz_,      100.0);
    pnh_.param<double>("watchdog_sec", watchdog_sec_, 0.15);
    pnh_.param<double>("damping",      lambda2_,      1e-6);

    // 予測（一次遅れの時定数[s]）と位相リード（初期0で無効）
    pnh_.param<double>("tau_pred",   tau_pred_,   0.38);
    pnh_.param<double>("alpha_lead", alpha_lead_, 0.0);

    // 速度モード有効化の初期値
    pnh_.param<bool>("enable_on_start", vel_enabled_, false);

    // 競合回避（パラメータ・ミューテックス）
    pnh_.param<bool>("use_param_mutex", use_param_mutex_, false);
    pnh_.param<std::string>("mutex_param", mutex_param_, "/rm_cmd/owner");
    node_id_ = ros::this_node::getName();  // 例: /rm_twistclient

    if (!pnh_.getParam("joint_names", joint_names_)) {
      throw std::runtime_error("~joint_names is required.");
    }
    pnh_.getParam("joint_vel_limits", joint_vel_limits_); // 任意

    // URDF -> KDL
    urdf::Model model;
    if (!model.initParam("robot_description"))
      throw std::runtime_error("Failed to load /robot_description.");
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
      throw std::runtime_error("Failed to build KDL tree.");
    if (!tree.getChain(base_link_, tip_link_, chain_))
      throw std::runtime_error("Failed to get KDL chain from " + base_link_ + " to " + tip_link_);

    const unsigned int nj = chain_.getNrOfJoints();
    if (nj != 6) ROS_WARN("This node expects 6-DOF chain. Current: %u", nj);
    if (nj != joint_names_.size())
      throw std::runtime_error("joint_names size != chain DOF.");

    jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    // 上下限・速度上限
    pos_lower_.resize(nj); pos_upper_.resize(nj);
    urdf_vel_lim_.resize(nj, 0.0);
    for (size_t i=0;i<nj;++i) {
      auto uj = model.getJoint(joint_names_[i]);
      if (!uj || !uj->limits) {
        pos_lower_[i] = -1e9; pos_upper_[i] = 1e9;
        urdf_vel_lim_[i] = 0.0;
      } else {
        pos_lower_[i] = uj->limits->lower;
        pos_upper_[i] = uj->limits->upper;
        urdf_vel_lim_[i] = uj->limits->velocity;
      }
    }
    eff_vel_lim_.resize(nj);
    for (size_t i=0;i<nj;++i) {
      double cfg = (i<joint_vel_limits_.size()? joint_vel_limits_[i]:0.0);
      double urd = urdf_vel_lim_[i];
      if (cfg>0.0 && urd>0.0) eff_vel_lim_[i] = std::min(cfg,urd);
      else if (cfg>0.0)       eff_vel_lim_[i] = cfg;
      else if (urd>0.0)       eff_vel_lim_[i] = urd;
      else                    eff_vel_lim_[i] = 1.5; // fallback
    }

    // 内部状態
    q_est_.resize(nj);  q_est_.data.setZero();
    dq_last_.setZero();
    v_prev_.setZero();
    have_initial_q_ = false;

    last_js_vals_.assign(nj, std::numeric_limits<double>::quiet_NaN());
    last_js_stamp_ = ros::Time(0);

    // pub/sub
    pub_cmd_   = nh_.advertise<rm_msgs::JointPos>(out_topic_, 10);
    sub_js_    = nh_.subscribe(joint_states_topic_, 100, &TwistToJointPos::jointStateCb, this);
    sub_twist_ = nh_.subscribe(twist_topic_,        50,  &TwistToJointPos::twistCb,      this);
    sub_enable_= nh_.subscribe(enable_topic_,       10,  &TwistToJointPos::enableCb,     this);

    timer_ = nh_.createTimer(ros::Duration(1.0/rate_hz_), &TwistToJointPos::onTimer, this);

    // 起動時にenableなら取得を試みる
    if (vel_enabled_) {
      if (acquireMutexIfNeeded()) {
        reinitFromLastJointStates(/*force_wait=*/false);
        ROS_INFO("Velocity mode ENABLED at startup.");
      } else {
        vel_enabled_ = false;
        ROS_WARN("Velocity mode requested at startup but mutex not acquired. Staying DISABLED.");
      }
    }

    ROS_INFO_STREAM("TwistToJointPos+gate base="<<base_link_
      <<" tip="<<tip_link_<<" rate="<<rate_hz_<<"Hz tau_pred="<<tau_pred_
      <<" alpha_lead="<<alpha_lead_<<" watchdog="<<watchdog_sec_
      <<" enable_topic="<<enable_topic_
      <<" mutex_param="<< (use_param_mutex_? mutex_param_ : std::string("DISABLED")));
  }

  ~TwistToJointPos() {
    // 退出時に占有解除
    releaseMutexIfNeeded();
  }

private:
  // ---- Callbacks ----
  void jointStateCb(const sensor_msgs::JointState& msg) {
    // 最新の実測を joint_names_ の順で引き当て
    for (size_t i=0;i<joint_names_.size();++i) {
      double val = std::numeric_limits<double>::quiet_NaN();
      for (size_t k=0;k<msg.name.size();++k) {
        if (msg.name[k] == joint_names_[i]) {
          if (k < msg.position.size()) val = msg.position[k];
          break;
        }
      }
      last_js_vals_[i] = val;
    }
    last_js_stamp_ = msg.header.stamp.isZero()? ros::Time::now() : msg.header.stamp;

    if (have_initial_q_) return;

    // 初期化（初回のみ）
    bool all_found = true;
    for (size_t i=0;i<joint_names_.size();++i) {
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
      if (it == msg.name.end()) { all_found=false; break; }
      size_t k = std::distance(msg.name.begin(), it);
      if (k >= msg.position.size()) { all_found=false; break; }
    }
    if (!all_found) return;

    for (size_t i=0;i<joint_names_.size();++i) {
      size_t k = std::distance(msg.name.begin(),
                   std::find(msg.name.begin(), msg.name.end(), joint_names_[i]));
      q_est_(i)  = msg.position[k];
    }
    have_initial_q_ = true;
    ROS_INFO("Initialized internal states q_est_ from /joint_states once.");
  }

  void twistCb(const geometry_msgs::Twist& msg) {
    last_twist_ = msg;
    last_twist_stamp_ = ros::Time::now();
    have_twist_ = true;
  }

  void enableCb(const std_msgs::Bool& msg) {
    if (msg.data == vel_enabled_) return; // 状態変化なし

    if (msg.data) { // OFF -> ON
      if (!acquireMutexIfNeeded()) {
        ROS_WARN("Enable requested but mutex is held by another owner. Staying DISABLED.");
        return;
      }
      vel_enabled_ = true;
      // オンに切り替え時、毎回初期化（内部推定を保持しない）
      reinitFromLastJointStates(/*force_wait=*/false);
      have_twist_ = false; // 直前のTwistは使わない
      ROS_INFO("Velocity mode ENABLED (reinitialized from joint_states).");
    } else { // ON -> OFF
      vel_enabled_ = false;
      releaseMutexIfNeeded();
      ROS_INFO("Velocity mode DISABLED.");
    }
  }

  // ---- Mutex helpers (parameter-based) ----
  bool acquireMutexIfNeeded() {
    if (!use_param_mutex_) return true;
    std::string owner;
    if (!nh_.getParam(mutex_param_, owner)) owner.clear();
    if (!owner.empty() && owner != node_id_) return false;
    nh_.setParam(mutex_param_, node_id_);
    return true;
  }
  void releaseMutexIfNeeded() {
    if (!use_param_mutex_) return;
    std::string owner;
    if (!nh_.getParam(mutex_param_, owner)) return;
    if (owner == node_id_) {
      nh_.deleteParam(mutex_param_);
    }
  }

  // ---- Reinit from latest /joint_states ----
  void reinitFromLastJointStates(bool force_wait) {
    bool ok = true;
    for (double v : last_js_vals_) {
      if (!std::isfinite(v)) { ok = false; break; }
    }
    if (!ok) {
      if (force_wait) {
        ROS_INFO("Waiting for valid /joint_states to initialize...");
        ros::Rate r(200);
        ros::Time t0 = ros::Time::now();
        while (ros::ok()) {
          bool all = true;
          for (double v : last_js_vals_) if (!std::isfinite(v)) { all=false; break; }
          if (all) break;
          if ((ros::Time::now() - t0).toSec() > 1.0) break; // 1sタイムアウト
          ros::spinOnce(); r.sleep();
        }
        ok = true;
        for (double v : last_js_vals_) if (!std::isfinite(v)) { ok=false; break; }
      }
    }
    if (ok) {
      for (size_t i=0;i<joint_names_.size() && i<6;++i) {
        q_est_(i)  = last_js_vals_[i];
      }
      have_initial_q_ = true;
      ROS_INFO("Reinitialized internal states from cached /joint_states.");
    } else {
      have_initial_q_ = false; // 次のjoint_statesで初期化待ち
      ROS_WARN("No valid /joint_states yet; will initialize when it arrives.");
    }
  }

  // ---- Control loop ----
  void onTimer(const ros::TimerEvent&) {
    if (!have_initial_q_) return;

    const double dt = 1.0 / rate_hz_;

    // 速度モード無効、またはミューテックス不保持時は送信しない
    if (!vel_enabled_) return;
    if (use_param_mutex_) {
      std::string owner;
      if (!nh_.getParam(mutex_param_, owner) || owner != node_id_) {
        // 別ノードが所有権を取得した/失った場合は安全側で停止
        vel_enabled_ = false;
        ROS_WARN_THROTTLE(2.0, "Lost mutex or not owner. Auto DISABLE.");
        return;
      }
    }

    // watchdog：古い/無い Twist は「送信しない」
    bool twist_fresh = (have_twist_ &&
      (ros::Time::now() - last_twist_stamp_).toSec() <= watchdog_sec_);
    if (!twist_fresh) {
      return; // ★ publishしない（他制御と競合しない）
    }

    // 並進のみ使用（角速度は常に0）
    Eigen::Matrix<double,6,1> v6_raw;
    v6_raw << last_twist_.linear.x, last_twist_.linear.y, last_twist_.linear.z, last_twist_.angular.x, last_twist_.angular.y, last_twist_.angular.z;

    // 位相リード
    Eigen::Matrix<double,6,1> v6 = v6_raw;
    if (alpha_lead_ > 1e-12 && dt > 1e-6) {
      Eigen::Matrix<double,6,1> dv = (v6_raw - v_prev_) / dt;
      v6 = v6_raw + alpha_lead_ * dv;
    }
    v_prev_ = v6_raw;

    // 速度逆運動学（q_for_IKは内部推定q_est_）
    KDL::JntArray q_for_IK = q_est_;

    KDL::Jacobian J(chain_.getNrOfJoints());
    jac_solver_->JntToJac(q_for_IK, J);
    Eigen::Matrix<double,6,6> J6 = J.data.leftCols<6>();

    Eigen::Matrix<double,6,1> dq;
    Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu(J6);
    if (lu.rank() == 6) {
      dq = lu.solve(v6);
    } else {
      // damped least squares
      Eigen::Matrix<double,6,6> JJt = J6 * J6.transpose();
      JJt.diagonal().array() += lambda2_;
      Eigen::Matrix<double,6,1> a = JJt.ldlt().solve(v6);
      dq = J6.transpose() * a;
      ROS_WARN_THROTTLE(1.0, "Near singular: damped fallback used.");
    }

    // 速度上限
    for (int i=0;i<6;++i) {
      double lim = eff_vel_lim_[i];
      if (lim > 0.0) {
        if (dq(i) >  lim) dq(i) =  lim;
        if (dq(i) < -lim) dq(i) = -lim;
      }
    }

    // 積分
    KDL::JntArray q_act = q_est_;
    for (int i=0;i<6;++i) q_act(i) += dq(i) * dt;

    // 一次遅れ逆モデルから q_cmd を計算
    KDL::JntArray q_cmd = q_for_IK;
    if (tau_pred_ >= dt) {
      const double coef_pred = dt / tau_pred_;
      for (int i=0;i<6;++i) q_cmd(i) = (q_act(i)-(1-coef_pred)*q_for_IK(i))/coef_pred;
    } else {
      for (int i=0;i<6;++i) q_cmd(i) = q_act(i);
    }

    // 角度上限クリップ
    for (int i=0;i<6;++i) {
      if (q_cmd(i) < pos_lower_[i]) q_cmd(i) = pos_lower_[i];
      if (q_cmd(i) > pos_upper_[i]) q_cmd(i) = pos_upper_[i];
    }

    // 送信（ここまで来たらvelocity許可＆Twist新鮮＆ミューテックス保持）
    rm_msgs::JointPos out;
    out.joint.resize(6);
    for (int i=0;i<6;++i) out.joint[i] = q_cmd(i);
    pub_cmd_.publish(out);

    // 内部状態更新
    q_est_ = q_act;
    dq_last_ = dq;
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_js_, sub_twist_, sub_enable_;
  ros::Publisher  pub_cmd_;
  ros::Timer timer_;

  std::string joint_states_topic_, twist_topic_, out_topic_, enable_topic_;
  std::string base_link_, tip_link_;
  double rate_hz_{100.0}, watchdog_sec_{0.15}, lambda2_{1e-6};

  // 予測・位相リード
  double tau_pred_{0.05};
  double alpha_lead_{0.0};
  Eigen::Matrix<double,6,1> dq_last_;
  Eigen::Matrix<double,6,1> v_prev_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_vel_limits_, urdf_vel_lim_, eff_vel_lim_;
  std::vector<double> pos_lower_, pos_upper_;

  // 内部状態
  KDL::JntArray q_est_;
  bool have_initial_q_{false};

  // 最新の実測 joint_states を保持
  std::vector<double> last_js_vals_;
  ros::Time last_js_stamp_;

  geometry_msgs::Twist last_twist_;
  ros::Time last_twist_stamp_;
  bool have_twist_{false};

  // enable & mutex
  bool vel_enabled_{false};
  bool use_param_mutex_{true};
  std::string mutex_param_{"/rm_cmd/owner"};
  std::string node_id_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_twistclient");
  ros::NodeHandle nh, pnh("~");
  try {
    TwistToJointPos node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception: %s", e.what());
  }
  return 0;
}
