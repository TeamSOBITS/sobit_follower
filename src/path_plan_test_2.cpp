#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <float.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <sobit_follower/grouped_points_array.h>
#include <sobit_follower/grouped_points.h>
#include <sobit_follower/point_states.h>

typedef struct {         /* 構造体の型枠を定義する */
	double score;
	double norm_score;
	double min_score = DBL_MAX;
    double max_score = DBL_MIN;
} score_states;//スコア

typedef struct {         /* 構造体の型枠を定義する */
	score_states goal_;
	score_states obs_;
    score_states angle_; 
    score_states vel_;
	bool collision; 
} eval_states;//評価ステート

typedef struct {         /* 構造体の型枠を定義する */
	double canditate_velo;
	double canditate_th;
    double next_x; 
    double next_y;
	double next_th;
} pre_states;//予測ステート

typedef struct {         /* 構造体の型枠を定義する */
	double score;
	std::list<pre_states> path;
} optimal_state;//最適?なパスと評価値


class route_planning_class
{
	//nodehandle
	ros::NodeHandle nh;
	//pub
	ros::Publisher pub_path_marker;
	ros::Publisher pub_path_marker_all;
	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_goal_flag;
	ros::Publisher pub_point_states;
	//sub
	ros::Subscriber sub_goal;
	ros::Subscriber sub_obstacle_states;
	ros::Subscriber sub_pre_goal;
	//tf
	tf::TransformListener listener;
	//geometry
	geometry_msgs::Point goal_point;//base_laser_link基準
	geometry_msgs::Point obstacle_point;//障害物の位置座標
	geometry_msgs::Twist vel;
	geometry_msgs::PointStamped base_laser_point;//変換前のワールド基準の座標を格納する
	geometry_msgs::PointStamped base_odom_point;//変換後のロボットの座標を格納する
	//obstacle_states
	sobit_follower::grouped_points_array points_states;	//障害物のステート
	//パス配列
	std::list<pre_states> path_list;	//サンプリングごとの予測パスのステートリスト
	std::list<std::list<pre_states>> all_paths_list;	//予測した全てのパス(保存用)
	std::list<eval_states> path_eval_list;	//予測パスの各評価リスト
	std::list<optimal_state> optimal_path_list;//正直必要ない
	eval_states es;//各pathの評価値を格納
	//std::list<eval_states> es_list;//各pathの評価値を格納
	pre_states ph;
	//ロボットの位置と速度
	double current_robot_x = 0.0;
	double current_robot_y = 0.0;
	double current_robot_ang = 0.0;//現在のロボットの角度
	double current_robot_velo = 0.0;//現在のロボットの速度

	double stop_distance = 0.0;
 	double target_distance = 0.0;
	double max_velo_range = 0.0;
	double min_velo_range = 0.0;
	//ロボットの速度情報をパブリッシュするかの判定フラグ
	bool pre_flag = false;
	std::string base_frame_name;

	int old_vel_vec_size = 5;
	std::vector<double> old_vel_vec;

public:
	route_planning_class(){

	//試験用
	//this->goal_point.x = 2.0;//ワールド基準
	//this->goal_point.y = 0.0;

	base_frame_name = "base_footprint";
	//目的値をサブスクライブ
	//this->sub_goal = nh.subscribe("/target_states",1,&route_planning_class::target_OK,this);
	//targetとobstacle情報をサブスクライブ
	this->sub_obstacle_states = nh.subscribe( "/target_and_obstacle_states", 1, &route_planning_class::target_and_obstacle_states,this);
	//予測位置をサブスクライブ
	//this->sub_pre_goal = nh.subscribe( "/pre_point", 1, &route_planning_class::target_NG, this );
	//決定パスをパブリッシュ
	this->pub_path_marker = nh.advertise<visualization_msgs::MarkerArray>( "/path_marker", 1 );
	//全てのパスをパブリッシュ
	this->pub_path_marker_all = nh.advertise<visualization_msgs::MarkerArray>( "/path_marker_all", 1 );
	//車輪の速度をパブリッシュ
	this->pub_cmd_vel = nh.advertise<geometry_msgs::Twist>( "/cmd_vel_mux/input/teleop", 1 );
	//ロボットがゴール付近に到達したかのフラグをパブリッシュ
	this->pub_goal_flag = nh.advertise<std_msgs::Bool>( "/goal_flag", 1 );
	//ロボットとtarget情報をパブリッシュ
	this->pub_point_states = nh.advertise<sobit_follower::point_states>( "/pub_point_states", 1 );

	}//public

	~route_planning_class(){}

void target_and_obstacle_states(const sobit_follower::grouped_points_arrayPtr input)
{
	ros::Time begin = ros::Time::now();
	if( input->grouped_points_array.size() == 0 ) {return;}
	this->points_states = *input;
	this->target_distance = hypotf(input->target_point.x,input->target_point.y);
	std_msgs::Bool goal_flag;
	bool cmd_vel_flag;
	//速度制限
	double max_forward_velo_def = 0.8;//[m/s]	 最高前進速度(def:0.4)
	double min_forward_velo_def = 0.2;//[m/s]	 最低前進速度(def:0.0)
	double max_back_velo_def = -0.6;//[m/s]最高後退速度(def:-0.4)
	double min_back_velo_def = -0.1;//[m/s]	 最低後進速度(def:0.0)
	//laser則域距離(多分違う)
	double max_laser_range = 4.0;
	double min_laser_range = 0.2;
	//追従対象者とロボットの保つ距離 ※ max_keep_distance > min_keep_distanceである事
	double max_keep_distance = 1.3;
	double min_keep_distance = 0.9;
	ROS_INFO("target_position");
	std::cout << "" << input->target_point << std::endl;

	if(this->target_distance > max_keep_distance)//前進
	{
		printf("前進します\n");
		this->max_velo_range = (input->target_point.x - max_keep_distance) * max_forward_velo_def / (max_laser_range - max_keep_distance) + min_forward_velo_def;
		this->min_velo_range = 0.0;
		printf("最高前進速度:%f\n",this->max_velo_range);
		cmd_vel_flag = true;
	}
	else if(this->target_distance >= min_keep_distance && this->target_distance <= max_keep_distance)//停止
	{
		printf("targetに近いです\n");
		this->vel.linear.x = 0.0;
		this->vel.angular.z = 0.0;
		cmd_vel_flag = false;
		goal_flag.data = true;
	}
	else//後退
	{
		printf("後退します\n");
		this->max_velo_range = 0.0;
		this->min_velo_range = (input->target_point.x - min_keep_distance) * max_back_velo_def / (min_laser_range - min_keep_distance) + min_back_velo_def;
		printf("最高後退速度:%f\n",this->min_velo_range);
		cmd_vel_flag = true;
	}
	if(cmd_vel_flag == true)	//パス生成
	{
		dwa();	//dwaによるパス生成
		goal_flag.data = false;
		//odom基準から見たbase_laser_linkの座標系
		tf::StampedTransform transform;
		try{
	  		this->listener.lookupTransform("/odom", base_frame_name, ros::Time(0), transform);
		}//try
		catch (tf::TransformException ex){
	  		ROS_ERROR("%s",ex.what());
	  		ros::Duration(1.0).sleep();
		}//try
		geometry_msgs::PointStamped odom_base_point;//変換後のtargetの座標を格納する
		geometry_msgs::PointStamped laser_base_point;
		try{
			laser_base_point.header.frame_id = base_frame_name;
			laser_base_point.header.stamp = ros::Time(0);
			laser_base_point.point = this->points_states.target_point;
			this->listener.transformPoint("/odom", laser_base_point, odom_base_point );
		}//try
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("Received an exception trying to transform a point.: \n%s", ex.what());
		}//catch
		//odom基準のロボットの位置座標をPoint型に変換
		geometry_msgs::Point robot_point;
		robot_point.x = transform.getOrigin().x();
		robot_point.y = transform.getOrigin().y();
		robot_point.z = 0.0;
		//odom基準のロボットとtargetの位置をpoint_states型に変換→txtデータ用
		sobit_follower::point_states states;
		states.robot_point = robot_point;
		states.target_point = odom_base_point;
		this->pub_point_states.publish(states);
	}//if

	//std::cout << "this->vel.linear.x: " << this->vel.linear.x << std::endl;
	//速度の平滑処理
	old_vel_vec.push_back(this->vel.linear.x);//新しい値を入れる
	if(old_vel_vec.size() > old_vel_vec_size){
		old_vel_vec.erase(old_vel_vec.begin());//古い値を削除
	}
	this->vel.linear.x = 0.0;//一旦０にする
	for(int temp_count = 0; temp_count < old_vel_vec.size(); temp_count++){
		this->vel.linear.x += old_vel_vec[temp_count];//過去の値を足しこむ
	}
	this->vel.linear.x = this->vel.linear.x / double(old_vel_vec.size());//平均値で書き換え
	this->current_robot_velo = this->vel.linear.x;

	this->vel.linear.x = 0;
	this->vel.angular.z = 0;
	this->pub_cmd_vel.publish(this->vel);
	this->pub_goal_flag.publish(goal_flag);
}//obstacle_states_cb

void dwa()
{
	//------------------車輪のパラメータ--------------------
	//多次元の予測回数
	int pre_step = 30;//(def:30)
	//int collision_step = pre_step / 10;
	//サンプリングタイム
	double samplingtime = 0.2;//[s]

	//各速度の細分化回数
	double velo_step = 10.0;	//あまり大きくすると処理が重くなる(def:5)
	double ang_velo_step = 10.0;	//あまり大きくすると処理が重くなる(def:5)

	//回転速度制限
	double max_ang_def = 20.0;//[rad/s]	最高回転速度(def:20)
	double min_ang_def = -20.0;//[rad/s]	最低回転速度(def:0)
	//各加速度制御
	double max_acceration = 0.1;	//最高加速度(def:0.1)
	double max_ang_acceration = 0.2;	//最高回転加速度(57....度)(def:1.0)
	
	//回転加速度を考慮した範囲	
	double range_ang_velo = samplingtime * max_ang_acceration;
	double min_ang_velo = this->current_robot_ang - range_ang_velo;
	double max_ang_velo = this->current_robot_ang + range_ang_velo;

	//前進加速度を考慮した範囲
	double range_velo = samplingtime * max_acceration;
	double min_velo = this->current_robot_velo - range_velo;
	double max_velo = this->current_robot_velo + range_velo;

	//重み付け(モデルや環境によって調整する必要あり)
	double weighting_goal = 0.1;
	double weighting_obs = 0.0;//障害物の距離による評価はなし
	double weighting_angle = 0.04;
	double weighting_vel = 0.2;

	//パス番号
	int es_num = 0;

	//現在の各速度範囲の更新
	if(min_ang_velo < min_ang_def)	{ min_ang_velo = min_ang_def; }
	if(max_ang_velo > max_ang_def)	{ max_ang_velo = max_ang_def; }
	if(min_velo < this->min_velo_range)	{ min_velo = this->min_velo_range; }
	if(max_velo > this->max_velo_range)	{ max_velo = this->max_velo_range; }

	//　※path_eval_list.size() = velo_step * ang_velo_step
	double delta_velo = (max_velo - min_velo) / velo_step;
	double delta_ang_velo = (max_ang_velo - min_ang_velo) / ang_velo_step;

	//複数パスの生成
	for(double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += delta_ang_velo)//for_2
	{
		for(double velo = min_velo; velo < max_velo; velo += delta_velo)//for_3
		{
			this->ph.canditate_velo = velo;
			this->ph.canditate_th = ang_velo;
			predict_state(ang_velo,velo,current_robot_x,current_robot_y,current_robot_ang,samplingtime,pre_step,es_num);
			es_num++;
			this->all_paths_list.push_back(this->path_list);	//生成したpathを追加
			path_list.clear();	//path追加後，path_listを空にする→これをfor_2&for_3分だけループさせる
		}//for_3
	}//for_2

	normalization(); //正規化

	auto path = this->path_eval_list.begin();//scoreが入ったlist
	optimal_state optimal;
	optimal_state option;
	int collision_num = 0;
	int path_num = 0;
	//一つのパスを決定
	for(auto pn = this->all_paths_list.begin(); pn != this->all_paths_list.end(); pn++)//for_5
	{
		auto pa = *pn;
		option.path = pa;
		/*if(path->collision == true) {
			//std::cout << "collision_num :: " << collision_num << std::endl;
			collision_num++;
			path++;
			continue; 
		}//障害物判定されたパスは飛ばす */
		//else {
			auto paths = *pn;//複数pathの先頭アドレス
			double sum_score = weighting_goal * path->goal_.norm_score + weighting_obs * path->obs_.norm_score + weighting_angle * path->angle_.norm_score + weighting_vel * path->vel_.norm_score;
			//printf("path_num%d\n",path_num);
			//std::cout << "sum_score :: " << sum_score << std::endl;
			if(optimal.score < sum_score)
			{
				optimal.path = paths;
				optimal.score = sum_score;
			}//if
			path_num++;
			path++;
		//}//else
	}//for_5
	auto states = optimal.path.begin();
	this->optimal_path_list.push_back(optimal);//正直必要ない

	//マーカー表示(確認用)
	path_marker_array_2(collision_num);
	path_marker_array(optimal);

	//ロボットの速度と各パススコアの更新
	update(states);

	//パスリストの初期化
	this->all_paths_list.clear();	//全てのpath情報を空にする
	this->optimal_path_list.clear();//最適pathを空にする(正直必要ない)
	this->path_eval_list.clear();//各pathの評価ステートを空にする
}//dwa

void normalization()
{
	for(auto data = this->path_eval_list.begin(); data != this->path_eval_list.end(); data++)//for_4
	{
		//if(data->collision == true) { continue; }
		if(this->es.goal_.max_score - this->es.goal_.min_score != 0) {
		data->goal_.norm_score = (data->goal_.score - this->es.goal_.min_score) / (this->es.goal_.max_score - this->es.goal_.min_score);
		}
		else { data->goal_.norm_score = 0; }
		if(this->es.obs_.max_score - this->es.obs_.min_score != 0) {
		data->obs_.norm_score = (data->obs_.score - this->es.obs_.min_score) / (this->es.obs_.max_score - this->es.obs_.min_score);
		}
		else { data->obs_.norm_score = 0; }
		if(this->es.angle_.max_score - this->es.angle_.min_score != 0) {
		data->angle_.norm_score = (data->angle_.score -this->es.angle_.min_score) / (this->es.angle_.max_score - this->es.angle_.min_score);
		}
		else { data->angle_.norm_score = 0; }
		if(this->es.vel_.max_score - this->es.vel_.min_score != 0) {
		data->vel_.norm_score = (data->vel_.score - this->es.vel_.min_score) / (this->es.vel_.max_score - this->es.vel_.min_score);
		}
		else { data->vel_.norm_score = 0; }
	}//for_4
}//normalization

void predict_state(double ang_velo,double velo,double x,double y,double th,double dt,int st,int es_num)	//pathを予測する
{
	//eval_states es_list[st] = {};//パスリスト
	bool ph_flag;
	//printf("パス%d\n",es_num);
	for(int i = 0; i < st; i++)
	{
		pre_states temp;
		temp.next_x = velo * cos(th) * dt + x;
		temp.next_y = velo * sin(th) * dt + y;
		temp.next_th = ang_velo * dt + th;
		ph_flag = evaluate(st,es_num,temp,i);
		if(ph_flag == true) { break; }
		this->ph.next_x = temp.next_x;
		this->ph.next_y = temp.next_y;
		this->ph.next_th = temp.next_th;
		this->path_list.push_back(this->ph);
		x = temp.next_x;
		y = temp.next_y;
		th = temp.next_th;
	}//for
	//std::cout << "this->es.goal_.score :: " << this->es.goal_.score << std::endl;
}//predict_state

bool evaluate(int st,int es_num,pre_states temp,int i)
{
	this->es.collision = false;//初期化
	double min_obs_distance_score = DBL_MAX;
	pre_states base_temp;
	for( int i = 0; i < this->points_states.grouped_points_array.size(); i ++ )//for_1
	{
		//if(this->points_states.grouped_points_array[i].center_radius == 0) { continue; }
		double center_radius = this->points_states.grouped_points_array[i].center_radius;
		double center_point_x = this->points_states.grouped_points_array[i].center_x;
		double center_point_y = this->points_states.grouped_points_array[i].center_y;
		double ob_point_radius = this->points_states.grouped_points_array[i].particle_radius;
		double distance_from_center_pre_point = hypotf(temp.next_x - center_point_x , temp.next_y - center_point_y);
		for(int j = 0; j < this->points_states.grouped_points_array[i].particle_x.size(); j++)//for_2
		{
			double obstacle_point_x = this->points_states.grouped_points_array[i].particle_x[j];
			double obstacle_point_y = this->points_states.grouped_points_array[i].particle_y[j];
			double distance_from_ob_point_pre_point = hypotf(temp.next_x - obstacle_point_x , temp.next_y - obstacle_point_y);
			if(distance_from_ob_point_pre_point <= ob_point_radius) //if pathの位置がobstacleと接触していた場合
			{
				this->es.collision = true;	//パス評価可否基準
				//this->path_eval_list.push_back(this->es);//※衝突判定前の情報(es)もpush
				//printf("衝突\n");
				//printf("パス位置%d番目\n",i);
				break;
			}//if
		}//for_2
		if(this->es.collision == true) { break; }
		double obs_to_point_difference = fabs(center_radius - distance_from_center_pre_point);
		if(min_obs_distance_score > obs_to_point_difference) { min_obs_distance_score = obs_to_point_difference; }
	}//for_1
	if(this->es.collision == true) { 
	base_temp = this->ph;
	//printf("衝突判定\n");
	}//以下の処理は行わない
	else if(this->es.collision == false && i == st - 1) { base_temp = temp; }
	else { return this->es.collision; }
	this->es.obs_.score = min_obs_distance_score;//score評価1::障害物との距離
	double posi_from_pre_to_goal_x = this->points_states.target_point.x - base_temp.next_x;
	double posi_from_pre_to_goal_y = this->points_states.target_point.y - base_temp.next_y;

	double frame_from_pre_to_goal_x = posi_from_pre_to_goal_x * cos(base_temp.next_th) - posi_from_pre_to_goal_y * sin(base_temp.next_th);
	double frame_from_pre_to_goal_y = posi_from_pre_to_goal_x * sin(base_temp.next_th) + posi_from_pre_to_goal_y * cos(base_temp.next_th);
	double theta_from_pre_to_goal = fabs(atan2(frame_from_pre_to_goal_y,frame_from_pre_to_goal_x));

	this->es.goal_.score = 10.0 - hypotf(posi_from_pre_to_goal_x , posi_from_pre_to_goal_y);	//score評価2::ゴールとpathの距離
	this->es.angle_.score = (M_PI / 2) - theta_from_pre_to_goal; //score評価3::goalとpathの角度間隔
	this->es.vel_.score = fabs(base_temp.canditate_velo);//score評価4::車輪の前進速度がどの程度かを判定
	this->path_eval_list.push_back(this->es);//push

	//全パスにおける最大値と最小値を更新(正直書き方が下手)
	if(this->es.obs_.min_score > this->es.obs_.score) { this->es.obs_.min_score = this->es.obs_.score; }
	if(this->es.obs_.max_score < this->es.obs_.score) { this->es.obs_.max_score = this->es.obs_.score; }
	if(this->es.goal_.min_score > this->es.goal_.score) { this->es.goal_.min_score = this->es.goal_.score; }
	if(this->es.goal_.max_score < this->es.goal_.score) { this->es.goal_.max_score = this->es.goal_.score; }
	if(this->es.angle_.min_score > this->es.angle_.score) { this->es.angle_.min_score = this->es.angle_.score; }
	if(this->es.angle_.max_score < this->es.angle_.score) { this->es.angle_.max_score = this->es.angle_.score; }
	if(this->es.vel_.min_score > this->es.vel_.score) { this->es.vel_.min_score = this->es.vel_.score; }
	if(this->es.vel_.max_score < this->es.vel_.score) { this->es.vel_.max_score = this->es.vel_.score; }
	/*std::cout << "this->es.vel_.max_score :: " << this->es.vel_.max_score << std::endl;
	std::cout << "this->es.vel_.min_score :: " << this->es.vel_.min_score << std::endl;
	std::cout << "this->es.vel_.score :: " << this->es.vel_.score << std::endl;*/
	return this->es.collision;
}//evaluate

void update(std::list<pre_states>::iterator states)
{
	//if(this->tf_flag == false) {return false;}
	//double stop_distance_max = this->base_laser_point.point.x - this->keep_distance;
	//double stop_distance_min = this->keep_distance;

	//速度の選択
	this->vel.linear.x = states->canditate_velo;
	this->vel.angular.z = states->canditate_th;

	//各基準スコアの更新→書き方汚い
	this->es.obs_.min_score = DBL_MAX;
	this->es.obs_.max_score = DBL_MIN;
	this->es.goal_.min_score = DBL_MAX;
	this->es.goal_.max_score = DBL_MIN;
	this->es.angle_.min_score = DBL_MAX;
	this->es.angle_.max_score = DBL_MIN;
	this->es.vel_.min_score = DBL_MAX;
	this->es.vel_.max_score = DBL_MIN;
}//update

void path_marker_array(optimal_state data)
{
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(3);
	marker.markers[0].points.resize(10);
	marker.markers[0].header.frame_id = base_frame_name;
	marker.markers[0].header.stamp = ros::Time::now() + ros::Duration(20);
	marker.markers[0].action = visualization_msgs::Marker::ADD;
	marker.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
	marker.markers[0].ns = "optimum_path";
	marker.markers[0].id = 1;
	marker.markers[0].scale.x = 0.03;
	marker.markers[0].color.a = 1.0;
	marker.markers[0].color.r = 0.0;
	marker.markers[0].color.g = 0.0;
	marker.markers[0].color.b = 1.0;
	geometry_msgs::Point test;
	for(auto state = data.path.begin(); state != data.path.end(); state++)//for_5
	{
		test.x = state->next_x;
		test.y = state->next_y;
		geometry_msgs::Point p;
		p.x = state->next_x;
		p.y = state->next_y;
		p.z = 0.0;
		marker.markers[0].points.push_back(p);
	}//for_5
	marker.markers[1].header.frame_id = base_frame_name;
	marker.markers[1].header.stamp = ros::Time::now();
	marker.markers[1].action = visualization_msgs::Marker::ADD;
	marker.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker.markers[1].ns = "last_path_point";
	marker.markers[1].id = 2;
	marker.markers[1].color.a = 1.0;
	marker.markers[1].color.r = 0.0;
	marker.markers[1].color.g = 0.0;
	marker.markers[1].color.b = 1.0;
	marker.markers[1].scale.x = 0.02;//[m]
	marker.markers[1].scale.y = 0.02;//[m]
	marker.markers[1].scale.z = 1.0;//[m]
	marker.markers[1].pose.position.x = test.x;
	marker.markers[1].pose.position.y = test.y;
	marker.markers[1].pose.position.z = 0.0;
	marker.markers[1].pose.orientation.x = 0;
	marker.markers[1].pose.orientation.y = 0;
	marker.markers[1].pose.orientation.z = 0;
	marker.markers[1].pose.orientation.w = 1;

	marker.markers[2].header.frame_id = base_frame_name;
	marker.markers[2].header.stamp = ros::Time::now();
	marker.markers[2].action = visualization_msgs::Marker::ADD;
	marker.markers[2].type = visualization_msgs::Marker::SPHERE;
	marker.markers[2].ns = "goal_point";
	marker.markers[2].id = 3;
	marker.markers[2].color.a = 1.0;
	marker.markers[2].color.r = 0.0;
	marker.markers[2].color.g = 1.0;
	marker.markers[2].color.b = 0.0;
	marker.markers[2].scale.x = 0.02;//[m]
	marker.markers[2].scale.y = 0.02;//[m]
	marker.markers[2].scale.z = 1.0;//[m]
	marker.markers[2].pose.position.x = this->goal_point.x;
	marker.markers[2].pose.position.y = this->goal_point.y;
	marker.markers[2].pose.position.z = 0.0;
	marker.markers[2].pose.orientation.x = 0;
	marker.markers[2].pose.orientation.y = 0;
	marker.markers[2].pose.orientation.z = 0;
	marker.markers[2].pose.orientation.w = 1;

	this->pub_path_marker.publish( marker );
}//path_marker_array

void path_marker_array_2(int collision_size)
{
	int k = 0;
	int s = 0;
	auto path = this->path_eval_list.begin();
	visualization_msgs::MarkerArray marker_all;
	marker_all.markers.resize(this->all_paths_list.size());
	for(auto pn = this->all_paths_list.begin(); pn != this->all_paths_list.end(); pn++)//for_6
	{
		auto pa = *pn;
		optimal_state option;
		option.path = pa;
		//std::cout << "this->path_eval_list.size() :: " << this->path_eval_list.size() << std::endl;
		//std::cout << "option.path.size() :: " << option.path.size() << std::endl;
		marker_all.markers[k].header.frame_id = base_frame_name;
		marker_all.markers[k].header.stamp = ros::Time::now();
		marker_all.markers[k].action = visualization_msgs::Marker::ADD;
		marker_all.markers[k].type = visualization_msgs::Marker::LINE_STRIP;
		marker_all.markers[k].ns = "all_path";
		marker_all.markers[k].id = k;
		marker_all.markers[k].scale.x = 0.03;
		marker_all.markers[k].color.a = 0.5;
		if(path->collision == true) {//衝突
			marker_all.markers[k].color.r = 1.0;//1.0
			marker_all.markers[k].color.g = 0.0;
			marker_all.markers[k].color.b = 0.0;
		}//if
		else {
			marker_all.markers[k].color.r = 0.0;
			marker_all.markers[k].color.g = 1.0;//1.0
			marker_all.markers[k].color.b = 0.0;
		}//else
		for(auto state = option.path.begin(); state != option.path.end(); state++)//for_7
		{
			geometry_msgs::Point p;
			p.x = state->next_x;
			p.y = state->next_y;
			p.z = 0.0;
			marker_all.markers[k].points.push_back(p);
		}//for_7
		k++;
		path++;
	}//for_6
	this->pub_path_marker_all.publish( marker_all );
}//path_marker_array_2

};//route_planning_class

int main(int argc,char **argv)
{

	ros::init(argc,argv,"route_planning_node");
	route_planning_class psc;
	ros::spin();
	return 0;
}
