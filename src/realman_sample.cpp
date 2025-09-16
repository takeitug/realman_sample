#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <vector>
#include <rm_msgs/Joint_Speed.h> // 速度制御に必要なヘッダーファイルを追加

// 円周率の定義
#define PI 3.14159265358979323846

int main(int argc, char** argv) {
    ros::init(argc, argv, "realman_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ロボットのグループ名を指定（環境に合わせて変更）
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    ROS_INFO_STREAM("コントロールモードの選択:");
    ROS_INFO_STREAM("1: 位置制御（関節）");
    ROS_INFO_STREAM("2: 速度制御（関節）");
    
    int judge;
    std::cin >> judge;

    if (judge == 1) {
        // 位置制御（関節）ブロック
        ROS_INFO_STREAM("位置制御（関節）モードを選択しました。");
        std::vector<double> joints = {
            -37 * PI / 180,
            45 * PI / 180,
            -103 * PI / 180,
            31 * PI / 180,
            -85 * PI / 180,
            -95 * PI / 180
        };

        move_group.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            ROS_INFO_STREAM("動作を計画・実行します。");
            move_group.move();
        } else {
            ROS_ERROR_STREAM("動作計画に失敗しました。");
        }
    } else if (judge == 2) {
        // 速度制御ブロック
        ROS_INFO_STREAM("速度制御（関節）モードを選択しました。");
        
        // パブリッシャーを初期化
        ros::Publisher joint_speed_pub = nh.advertise<rm_msgs::Joint_Speed>("/rm_driver/Udp_Joint_Speed", 10);
        
        ros::Rate rate(100); // 制御周期を100Hzに設定

        ROS_INFO("関節速度制御を開始します。5秒間動作します。");

        // 関節速度メッセージを作成
        rm_msgs::Joint_Speed speed_msg;
        speed_msg.joint_speed.resize(6); // 6つの関節分にリサイズ

        // 5秒間 (100Hz * 500回) 速度指令を送信するループ
        for (int count = 0; count <= 200; ++count) {
            // 各関節の速度を設定 (ラジアン/秒)
            speed_msg.joint_speed[0] = 0.0;
            speed_msg.joint_speed[1] = 0.0;
            speed_msg.joint_speed[2] = 0.05; // 関節3をゆっくり動かす
            speed_msg.joint_speed[3] = 0.0;
            speed_msg.joint_speed[4] = 0.0;
            speed_msg.joint_speed[5] = 0.0;

            // メッセージをパブリッシュ（送信）
            joint_speed_pub.publish(speed_msg);
            
            ros::spinOnce();
            rate.sleep();
        }
        
        // プログラム終了時に速度をゼロにして停止させる
        for (int i = 0; i < 6; ++i) {
            speed_msg.joint_speed[i] = 0.0;
        }
        joint_speed_pub.publish(speed_msg);

        // 停止指令が確実に届くよう1秒待機
        ros::Duration(1.0).sleep();

    } else {
        ROS_WARN_STREAM("無効な入力です。プログラムを終了します。");
    }

    ros::shutdown();
    return 0;
}