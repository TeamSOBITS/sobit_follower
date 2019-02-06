#include <stdio.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <time.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <iomanip>
#include <limits>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <float.h>
#include <list>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Core>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef struct {         /* 構造体の型枠を定義する */
	geometry_msgs::Point observe_point;
	geometry_msgs::Point current_pre_point;
	geometry_msgs::Point after_pre_point;
	geometry_msgs::Point filtered_point;
	geometry_msgs::Vector3 filtered_vel;
} info_set;
class kalman_filter_class
{
public:
	ros::NodeHandle nh;
	//SUB
	ros::Subscriber sub_target_point;
	ros::Subscriber sub_target_flag;
	//PUB
	ros::Publisher pub_point_marker;
	ros::Publisher pub_pre_target_point;
	ros::Publisher pub_kalman_point;
	//ros::Publisher cost_grant_point;
	ros::Publisher pub_cost_marker;
	tf::TransformListener listener;
	
	geometry_msgs::Point observe_target_point;//観測値
	
	int dimension_size = 4;
	int seq = 0;
	ros::Duration duration;
	ros::Time before_time;//以前の時刻
	float pre_target_info[4] = {};
	float current_error_dispersion[4][4] = {};
	double dt = 0.0;
	info_set states;
	std::string base_frame_name;

	Eigen::MatrixXf pre_dispersion_states_of_rear;
	Eigen::MatrixXf current_observe_states;//現在の観測値を用いて観測方程式から求めた値
	Eigen::MatrixXf current_states;//現在の観測値
	Eigen::MatrixXf H;//単位変換行列
	Eigen::MatrixXf H_TRANS;//Hの転置行列
	Eigen::MatrixXf pre_error_dispersion_of_current;//現在における事前誤差共分散
	Eigen::MatrixXf S;//H * pre_error_dispersion_of_current * H_TRANSの逆行列
	Eigen::MatrixXf S_TRANS;//H * pre_error_dispersion_of_current * H_TRANSの逆行列
	Eigen::MatrixXf pre_error_dispersion;//1ステップ前の事後誤差共分散の初期化
	Eigen::MatrixXf F;//状態遷移
	Eigen::MatrixXf F_TRANS;//Fの転置行列
	Eigen::MatrixXf E;//単位行列
	Eigen::MatrixXf kalman_gain;	//カルマンゲイン
	Eigen::MatrixXf exception;	//カルマンゲインの例外処理用
	Eigen::MatrixXf W;//状態方程式におけるノイズ
	Eigen::MatrixXf R;//観測方程式におけるノイズ
	Eigen::MatrixXf filtered_target_states;//補正後の状態(現在における推定値)
	Eigen::MatrixXf filtered_dispersion;//現在における事後誤差共分散

	float kansoku_x[7] = {0,6,7,8,9,10,11};
	float kansoku_y[7] = {0,17,15,14,13,12,11};

	kalman_filter_class(){
		base_frame_name = "base_footprint";
		this->current_states.setZero(2,1);
		this->pre_dispersion_states_of_rear.setZero(4,1);
		this->current_observe_states.setZero(4,1);
		this->H.setZero(2,4);
		this->H_TRANS.setZero(4,2);
		this->pre_error_dispersion_of_current.setZero(4,4);
		//std::cout << "" << this->pre_error_dispersion_of_current << std::endl;
		this->S.setZero(2,2);
		this->S_TRANS.setZero(2,2);
		this->pre_error_dispersion.setZero(4,4);
		this->F.setZero(4,4);
		this->F_TRANS.setZero(4,4);
		this->E.setZero(4,4);
		this->kalman_gain.setZero(4,2);
		this->exception.setZero(4,2);
		this->W.setZero(4,1);
		this->R.setZero(2,2);
		this->filtered_target_states.setZero(4,1);
		this->filtered_dispersion.setZero(4,4);
		
		this->sub_target_point = nh.subscribe("/target_states",1,&kalman_filter_class::target_point,this);
		this->pub_point_marker = nh.advertise<visualization_msgs::MarkerArray>( "/point_marker", 1 );
		this->sub_target_flag = nh.subscribe("/target_judge",1,&kalman_filter_class::target_judge,this);
		this->pub_pre_target_point = nh.advertise<geometry_msgs::Point>( "/pre_point", 1 );
		this->pub_kalman_point = nh.advertise<geometry_msgs::Point>( "/kalman_point", 1 );
		//this->sub_info = nh.subscribe("target_info",1,&kalman_filter_class::pp);
		//this->pub_point_marker = nh.advertise<geometry_msgs::Point>("/publish_state",1);
	}//public

~kalman_filter_class(){}

void target_point(const geometry_msgs::Point& msg)
{
	//printf("aaaaaaaaaaa\n");
	this->dt = 0.1;//時間間隔
	this->seq++;

	printf("laser基準");
	std::cout << "laser_base" << msg << std::endl;
	/*geometry_msgs::PointStamped odom_base_target_point;//変換後のtargetの座標を格納する
	geometry_msgs::PointStamped robot_base_target_point;
	try{
		robot_base_target_point.header.frame_id = base_frame_name;
		robot_base_target_point.header.stamp = ros::Time(0);
		robot_base_target_point.point = msg;
		this->listener.transformPoint("/odom", robot_base_target_point, odom_base_target_point );
	}//try
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
		return;
	}//catch*/
	this->current_states(0,0) = msg.x;
	this->current_states(1,0) = msg.y;
	//this->current_states(0,0) = odom_base_target_point.point.x;
	//this->current_states(1,0) = odom_base_target_point.point.y;
	//観測値の更新
	//this->current_states(0,0) = this->kansoku_x[0] + i * 0.1;
	//this->current_states(1,0) = this->kansoku_y[0] + i * 0.02;
	//printf("odom基準");
	//std::cout << "this->current_states(0,0)" << this->current_states(0,0) << std::endl;
	//std::cout << "this->current_states(1,0)" << this->current_states(1,0) << std::endl;

	filtering();

}//target_point

void filtering()
{
	Eigen::MatrixXf observation_pre_error_of_current(4,1);//観測値の予測誤差
	//printf("過去から現在の予測値\n");
	//std::cout << "" << this->pre_dispersion_states_of_rear << std::endl;
	//初期化関数
	init();
	//現在における事前予測誤差の分散と観測方程式から観測値を求める・・・①
	this->current_observe_states = this->H * this->current_states;//現在における観測方程式(Zt)

	/*printf("現在の観測値　前\n");
	std::cout << "" << this->current_states << std::endl;
	printf("this->H\n");
	std::cout << "" << this->H << std::endl;*/
	//観測した値の予測誤差を求める・・・②
	observation_pre_error_of_current = this->current_observe_states - (this->H * this->pre_dispersion_states_of_rear);
	//カルマンゲインの計算・・・③
	kalman();
	//②，③を用いて，予測と観測から補正された状態の算出(事後推定値)
	this->filtered_target_states =  this->pre_dispersion_states_of_rear + this->kalman_gain * observation_pre_error_of_current;


	/*printf("現在の観測値\n");
	std::cout << "" << this->current_observe_states << std::endl;
	printf("状態遷移\n");
	std::cout << "" << this->F << std::endl;
	printf("現在における分散\n");
	std::cout << "" << this->pre_error_dispersion_of_current << std::endl;
	printf("観測値の予測誤差\n");
	std::cout << "" << observation_pre_error_of_current << std::endl;
	printf("過去から現在の予測値\n");
	std::cout << "" << this->pre_dispersion_states_of_rear << std::endl;*/
	printf("補正値\n");
	std::cout << "" << this->filtered_target_states << std::endl;

	//現在における事後誤差共分散←これは1ステップ後の事前共分散として扱う
	this->filtered_dispersion = this->pre_error_dispersion_of_current - this->kalman_gain * this->H * this->pre_error_dispersion_of_current;
	//printf("補正値の分散\n");
	//std::cout << "" << this->filtered_dispersion << std::endl;

	//現在の補正値から1ステップ後の状態と分散を求める
	predict_target_and_error_update();

	update();

}//filtering

void init()
{
	geometry_msgs::Point init_target_point;//人の位置の初期化
	geometry_msgs::Vector3 init_vel;//人の前進速度の初期化
	//FとF_TRANS，1ステップ前の事後誤差共分散行列の更新(初期化時のdtは0として扱う)
	for (int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++) {
			if(i == j)	{
				this->F(i,j) = 1.0;
				this->F_TRANS(i,j) = 1.0;
				this->E(i,j) = 1.0;
			}//if
			else if(i == j - 2 && j == i + 2) {this->F(i,j) = this->dt;}
			else if(j == i - 2 && i == j + 2) {this->F_TRANS(i,j) = this->dt;}
			else
			{
				this->F(i,j) = 0.0;
				this->F_TRANS(i,j) = 0.0;
				this->E(i,j) = 0.0;
			}//else
		}//for
	}//for
	//printf("seq ======== %d\n",this->seq);
	//printf("初期化\n");
	//HとH_TRANSの初期化(単位変換時に別処理)
	if(this->seq == 1) {
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 4; j++) {
			if(i == j) {
			this->H(i,j) = 1.0;
			this->H_TRANS(j,i) = 1.0;
			this->exception(j,i) = 1.0;
			}
			else {
			this->H(i,j) = 0.0;
			this->H_TRANS(j,i) = 0.0;
			this->exception(j,i) = 0.0;
			}
		}//for
	}//for
	//printf("this->H\n");
	//std::cout << "" << this->H << std::endl;
	this->pre_error_dispersion_of_current(2,2) = 100.0;
	this->pre_error_dispersion_of_current(3,3) = 100.0;
	//1ステップ前の状態の初期化
	this->pre_dispersion_states_of_rear(0,0) = init_target_point.x;
	this->pre_dispersion_states_of_rear(1,0) = init_target_point.y;
	this->pre_dispersion_states_of_rear(2,0) = init_vel.x;
	this->pre_dispersion_states_of_rear(3,0) = init_vel.y;
	this->pre_error_dispersion_of_current = this->F * this->pre_error_dispersion_of_current * this->F_TRANS;
	}
}//init

void predict_target_and_error_update()
{
	//1ステップ後の状態を予測
	this->pre_dispersion_states_of_rear(0,0) = this->filtered_target_states(0,0) + this->dt * this->filtered_target_states(2,0);
	this->pre_dispersion_states_of_rear(1,0) = this->filtered_target_states(1,0) + this->dt * this->filtered_target_states(3,0);
	this->pre_dispersion_states_of_rear(2,0) = this->filtered_target_states(2,0);
	this->pre_dispersion_states_of_rear(3,0) = this->filtered_target_states(3,0);
	//printf("現在から未来の予測値\n");
	//std::cout << "" << this->pre_dispersion_states_of_rear << std::endl;

	//1ステップ後における事前誤差共分散行列
	this->pre_error_dispersion_of_current = this->E + this->F * this->filtered_dispersion * this->F_TRANS;

	//printf("未来の予測分散\n");
	//std::cout << "" << this->pre_error_dispersion_of_current << std::endl;
}//predict_target_and_error_update

void update()//視覚化用に格納
{
	this->states.observe_point.x = this->current_observe_states(0,0);
	this->states.observe_point.y = this->current_observe_states(1,0);
	this->states.current_pre_point.x = this->pre_dispersion_states_of_rear(0,0);
	this->states.current_pre_point.y = this->pre_dispersion_states_of_rear(1,0);
	this->states.filtered_point.x = this->filtered_target_states(0,0);
	this->states.filtered_point.y = this->filtered_target_states(1,0);
	this->states.filtered_vel.x = this->filtered_target_states(2,0);
	this->states.filtered_vel.y = this->filtered_target_states(3,0);
	this->states.after_pre_point.x = this->pre_dispersion_states_of_rear(0,0);
	this->states.after_pre_point.y = this->pre_dispersion_states_of_rear(1,0);
	this->pub_kalman_point.publish(this->states.filtered_point);
	//printf("予測値\n");
	//std::cout << "" << robot_base_target_point.point << std::endl;
	//target_point_marker(robot_base_target_point.point);
}//update

void kalman()
{
	this->S = this->H * this->pre_error_dispersion_of_current * this->H_TRANS + this->R;//後に逆行列を求める
	//std::cout << "" << this->H << std::endl;
	//std::cout << "" << this->H_TRANS << std::endl;
	//std::cout << "" << this->pre_error_dispersion_of_current << std::endl;
	double TRNS_value = this->S(0,0) * this->S(1,1) - this->S(0,1) * this->S(1,0);
	Eigen::MatrixXf E_2(2,2);//単位行列(2 * 2)
	E_2(0,0) = E_2(1,1) = 1.0;
	E_2(0,1) = E_2(1,0) = 0.0;
	this->R(0,0) = 2.0;
	this->R(0,1) = 0.0;
	this->R(1,1) = 2.0;
	this->R(1,0) = 0.0;
	//std::cout << "" << E_2 << std::endl;
	double buf = 0.0;//一時的な格納変数

	//printf("Sの逆行列（前）\n");
	//std::cout << "" << this->S_TRANS << std::endl;
	//printf("S行列（前）\n");
	//std::cout << "" << this->S << std::endl;
	if(TRNS_value == 0)//例外処理→分散性がなくなった時
	{
		this->kalman_gain = this->exception;
	}//if
	else	//掃き出し法→ノイズを含めていないので，分散性がなくなっていく
	{
		for(int i = 0; i < 2; i++)
		{
			buf = 1 / this->S(i,i);
			for(int j = 0; j < 2; j++)
			{
				this->S(i,j) *= buf;
				E_2(i,j) *= buf;
				//this->S_TRANS(i,j) = E_2(i,j);
			}//for
			for(int j = 0; j < 2; j++)
			{
				if(i != j)
				{
					buf = this->S(j,i);
					for(int k = 0; k < 2; k++)
					{
						this->S(j,k) -= this->S(i,k) * buf;
						E_2(j,k) -= E_2(i,k) * buf;
						//this->S_TRANS(j,k) -= this->S_TRANS(i,k) * buf;
					}//for
				}//if
			}//for
		}//for
		this->S_TRANS = E_2;
		this->kalman_gain = (this->pre_error_dispersion_of_current * this->H_TRANS) * this->S_TRANS;	
	}//else

	//printf("S\n");
	//std::cout << "" << this->S * this->S_TRANS << std::endl;
	//printf("カルマンゲイン\n");
	//std::cout << "" << this->kalman_gain << std::endl;
}//kalman

void target_judge(const std_msgs::Bool& msg)
{
	if(msg.data == true) 
	{
		geometry_msgs::PointStamped odom_base_target_point;//変換後のtargetの座標を格納する
		geometry_msgs::PointStamped robot_base_target_point;
		try{
		odom_base_target_point.header.frame_id = "/odom";
		odom_base_target_point.header.stamp = ros::Time(0);
		odom_base_target_point.point.x = this->states.after_pre_point.x;
		odom_base_target_point.point.y = this->states.after_pre_point.y;
		this->listener.transformPoint(base_frame_name, odom_base_target_point, robot_base_target_point );
		}//try
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
		}//catch
		this->pub_pre_target_point.publish(robot_base_target_point.point);
		printf("targetを見失ったので予測値をパブします.\n");
		std::cout << "" << robot_base_target_point.point << std::endl;
	}
}//target_judge

/*void target_point_marker(geometry_msgs::Point msg)
{
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(2);
	marker.markers[0].header.frame_id = base_frame_name;
	marker.markers[0].header.stamp = ros::Time::now();
	marker.markers[0].action = visualization_msgs::Marker::ADD;
	marker.markers[0].type = visualization_msgs::Marker::SPHERE;
	marker.markers[0].ns = "pre_point";
	marker.markers[0].id = 1;
	marker.markers[0].color.a = 1.0;
	marker.markers[0].color.r = 0.0;
	marker.markers[0].color.g = 0.0;
	marker.markers[0].color.b = 1.0;
	marker.markers[0].scale.x = 0.02;//[m]
	marker.markers[0].scale.y = 0.02;//[m]
	marker.markers[0].scale.z = 1.0;//[m]
	marker.markers[0].pose.position.x = msg.x;
	marker.markers[0].pose.position.y = msg.y;
	marker.markers[0].pose.position.z = 0.0;
	marker.markers[0].pose.orientation.x = 0;
	marker.markers[0].pose.orientation.y = 0;
	marker.markers[0].pose.orientation.z = 0;
	marker.markers[0].pose.orientation.w = 1;

	marker.markers[1].header.frame_id = base_frame_name;
	marker.markers[1].header.stamp = ros::Time::now();
	marker.markers[1].action = visualization_msgs::Marker::ADD;
	marker.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker.markers[1].ns = "observe_point";
	marker.markers[1].id = 2;
	marker.markers[1].color.a = 1.0;
	marker.markers[1].color.r = 1.0;
	marker.markers[1].color.g = 0.0;
	marker.markers[1].color.b = 0.0;
	marker.markers[1].scale.x = 0.02;//[m]
	marker.markers[1].scale.y = 0.02;//[m]
	marker.markers[1].scale.z = 1.0;//[m]
	marker.markers[1].pose.position.x = this->current_states(0,0);
	marker.markers[1].pose.position.y = this->current_states(1,0);
	marker.markers[1].pose.position.z = 0.0;
	marker.markers[1].pose.orientation.x = 0;
	marker.markers[1].pose.orientation.y = 0;
	marker.markers[1].pose.orientation.z = 0;
	marker.markers[1].pose.orientation.w = 1;

	/*marker.markers[2].header.frame_id = "base_laser_link";
	marker.markers[2].header.stamp = ros::Time::now();
	marker.markers[2].action = visualization_msgs::Marker::ADD;
	marker.markers[2].type = visualization_msgs::Marker::SPHERE;
	marker.markers[2].ns = "observe_point";
	marker.markers[2].id = 3;
	marker.markers[2].color.a = 1.0;
	marker.markers[2].color.r = 0.0;
	marker.markers[2].color.g = 1.0;
	marker.markers[2].color.b = 0.0;
	marker.markers[2].scale.x = 0.02;//[m]
	marker.markers[2].scale.y = 0.02;//[m]
	marker.markers[2].scale.z = 1.0;//[m]
	marker.markers[2].pose.position.x = this->current_states(0,0);
	marker.markers[2].pose.position.y = this->current_states(1,0);
	marker.markers[2].pose.position.z = 0.0;
	marker.markers[2].pose.orientation.x = 0;
	marker.markers[2].pose.orientation.y = 0;
	marker.markers[2].pose.orientation.z = 0;
	marker.markers[2].pose.orientation.w = 1;

	marker.markers[3].header.frame_id = "odom";
	marker.markers[3].header.stamp = ros::Time::now();
	marker.markers[3].action = visualization_msgs::Marker::ADD;
	marker.markers[3].type = visualization_msgs::Marker::SPHERE;
	marker.markers[3].ns = "pre_point";
	marker.markers[3].id = 4;
	marker.markers[3].color.a = 1.0;
	marker.markers[3].color.r = 0.5;
	marker.markers[3].color.g = 0.0;
	marker.markers[3].color.b = 0.5;
	marker.markers[3].scale.x = 0.02;//[m]
	marker.markers[3].scale.y = 0.02;//[m]
	marker.markers[3].scale.z = 1.0;//[m]
	marker.markers[3].pose.position.x = msg.x;
	marker.markers[3].pose.position.y = msg.y;
	marker.markers[3].pose.position.z = 0.0;
	marker.markers[3].pose.orientation.x = 0;
	marker.markers[3].pose.orientation.y = 0;
	marker.markers[3].pose.orientation.z = 0;
	marker.markers[3].pose.orientation.w = 1;
	this->pub_point_marker.publish( marker );

}//target_point_marker*/


};//Points_grouping_class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kalman_check_node");
	kalman_filter_class tf_class;
	ros::spin();
	return 0;
}//main
