#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp_lm{

public:

    icp_lm(ros::NodeHandle &n);
    ~icp_lm();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // sensor states = robot_x_y_theta
    Vector3d sensor_sta;

    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;
    // min match_cnt
    int min_match;

    // main process
    void process(visualization_msgs::MarkerArray input);
    // landMarks to Eigen::Matrix
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);
    // fint the nearest points & filter
    void findNearest(Eigen::Matrix2d R, Eigen::Vector2d t, Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2, Eigen::MatrixXd &outPoints, vector<float> &error);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    void findNearest_B(Eigen::Matrix2d RR, Eigen::Vector2d tt, Eigen::MatrixXd &pts1pts1, Eigen::MatrixXd &pts2pts2, Eigen::MatrixXd &outPointsoutPoints, vector<float> &errorerror);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp_lm::~icp_lm()
{}

icp_lm::icp_lm(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/icp_lm/robot_x", robot_x);
	n.getParam("/icp_lm/robot_y", robot_y);
	n.getParam("/icp_lm/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

    n.getParam("/icp_lm/max_iter", max_iter);
	n.getParam("/icp_lm/tolerance", tolerance);
	n.getParam("/icp_lm/dis_th", dis_th);
    n.getParam("/icp_lm/min_match", min_match);

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &icp_lm::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

void icp_lm::process(visualization_msgs::MarkerArray input)
{   
    cout<<"------Time:  "<<input.markers[0].header.stamp<<endl;
    
    double time_0 = (double)ros::Time::now().toSec();

    if(isFirstScan)
    {
        tar_pc = this->landMarksToMatrix(input);
        isFirstScan =false;
        return;
    }

    src_pc = this->landMarksToMatrix(input);
    if(src_pc.cols() < min_match)
    {
        src_pc = tar_pc;
        cout<<"LANDMARKS TO FEW"<<endl;
        return;
    }
    
    // 明天修改思路：正常匹配，多余的点舍去   ONLY modify findnearest, give back two equal matrix
    /*if(src_pc.cols() != tar_pc.cols())      // at that stage, src_pc is current and tar_pc is previous
    {
        src_pc = tar_pc;
        cout<<"LANDMARKS NUM NOT EQUAL"<<endl;
        return;
    }*/
    
    src_pc = tar_pc; // src_pc = previous tar_pc
    tar_pc = this->landMarksToMatrix(input);
    
    // init some variables
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);

    Eigen::Matrix2d R_12;
	R_12 << 1, 0, 0, 1;
	Eigen::Vector2d T_12;
	T_12[0] = T_12[1] = 0;
    Eigen::Matrix3d H_12;
    H_12 << R_12(0, 0), R_12(0, 1), T_12[0],
		    R_12(1, 0), R_12(1, 1), T_12[1],
		    0, 0, 1;
    
    int NUM = src_pc.cols();
    int NUM_tar = tar_pc.cols();
    Eigen::MatrixXd pts1_mid = Eigen::MatrixXd::Zero(2,NUM);    // tar_pc's match
    Eigen::MatrixXd pts2_mid = Eigen::MatrixXd::Zero(2,NUM_tar);    // src_pc's match
    vector<float> error;
	Eigen::Matrix2d R_final = R_12;
	Eigen::Vector2d T_final = T_12;
    float err = 0;

    cout<<"SRC_POINTS NUM: "<<NUM<<endl;
    cout<<"TAR_POINTS NUM: "<<NUM_tar<<endl;
    cout<<"src_pc size: "<<src_pc.size()<<endl;
    cout<<"tar_pc size: "<<tar_pc.size()<<endl;

    // main LOOP
    for(int i=0; i<max_iter; i++)
    {
        // TODO: please code by yourself

        error.clear();
        pts1_mid = Eigen::MatrixXd::Zero(2,NUM);     //clear
        pts2_mid = Eigen::MatrixXd::Zero(2,NUM_tar);

        if (NUM <= NUM_tar)
        {
            this->findNearest(R_12, T_12, tar_pc, src_pc, pts1_mid, error);
            H_12 = this->getTransform(pts1_mid, src_pc);
        }
        else
        {   //num_pc > num_tar
            this->findNearest_B(R_12, T_12, tar_pc, src_pc, pts2_mid, error);
            H_12 = this->getTransform(tar_pc, pts2_mid);
        }

        R_12 << H_12(0,0), H_12(0,1), H_12(1,0), H_12(1,1);
        T_12[0] = H_12(0,2);
        T_12[1] = H_12(1,2);
        Transform_acc =  H_12*Transform_acc;     // update final H
        
        // exit condition
        for(int j=0; j<NUM; j++)
            err += error[j];
        err /= NUM;
        if(err < tolerance)
            break;
    }

    tar_pc = this->landMarksToMatrix(input);
    cout<<"error: "<<err<<endl;
    cout<<"FINAL T: "<<endl<<Transform_acc<<endl;
    this->publishResult(Transform_acc);

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

Eigen::MatrixXd icp_lm::landMarksToMatrix(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();
    cout<<markerSize<<" markers received !"<<endl;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void icp_lm::findNearest(Eigen::Matrix2d R, Eigen::Vector2d t, Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2, Eigen::MatrixXd &outPoints, vector<float> &error)
{   // num_pc <= num_tar
    // TODO: please code by yourself
    error.clear();
    int num = pts2.cols();      // if src_pc are fewer, use src_pc.cols()
    int num_tar = pts1.cols();

    //对目标点云pts2进行变换 previous src
	for (int i = 0; i < num; i++)
	{
		double x = Eigen::Vector2f(R(0, 0), R(0, 1)).transpose() * Eigen::Vector2f(pts2(0,i), pts2(1,i)) + t[0];
		double y = Eigen::Vector2f(R(1, 0), R(1, 1)).transpose() * Eigen::Vector2f(pts2(0,i), pts2(1,i)) + t[1];
		
		pts2(0,i) = x;
        pts2(1,i) = y;
	}
	//计算最邻近点对
	for (int i = 0; i < num; i++)
	{
		float min_dist = std::numeric_limits<float>::max();
		int index = 0;
		for (int j = 0; j < num_tar; j++)
		{
			float dist = sqrt(pow(pts2(0,i) - pts1(0,j), 2) + pow(pts2(1,i) - pts1(1,j), 2));
			if (dist < min_dist)
			{
				min_dist = dist;
				index = j;
			}
		}
        //error.push_back(min_dist);
        if (min_dist > dis_th && min_dist < 0.3)
        {
            error.push_back(0.0005);
            outPoints(0,i) = pts1(0,index);
            outPoints(1,i) = pts1(1,index);     // pts2 change, and outpoints follow the index in pts2, pts1 stay same
        }  
        else
        {   
            error.push_back(min_dist);
            outPoints(0,i) = pts2(0,i);
            outPoints(1,i) = pts2(1,i);     // if < dis_th , stay same
            cout<<"TOO NEAR OR TOO FAR"<<endl;
        }
	}
}

void icp_lm::findNearest_B(Eigen::Matrix2d R, Eigen::Vector2d t, Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2, Eigen::MatrixXd &outPoints, vector<float> &error)
{   // num_src > num_tar
    // R, T, tar_pc, src_pc, pts2_mid(src_pc rematch), error
    error.clear();
    int num_pc = pts2.cols();      // if src_pc are fewer, use src_pc.cols()
    int num_tar = pts1.cols();

    //对目标点云pts2进行变换 previous src
	for (int i = 0; i < num_pc; i++)
	{
		double x = Eigen::Vector2f(R(0, 0), R(0, 1)).transpose() * Eigen::Vector2f(pts2(0,i), pts2(1,i)) + t[0];
		double y = Eigen::Vector2f(R(1, 0), R(1, 1)).transpose() * Eigen::Vector2f(pts2(0,i), pts2(1,i)) + t[1];
		
		pts2(0,i) = x;
        pts2(1,i) = y;
	}

	for (int i = 0; i < num_tar; i++)
	{
		float min_dist = std::numeric_limits<float>::max();
		int index = 0;
		for (int j = 0; j < num_pc; j++)
		{
			float dist = sqrt(pow(pts1(0,i) - pts2(0,j), 2) + pow(pts1(1,i) - pts2(1,j), 2));
			if (dist < min_dist)
			{
				min_dist = dist;
				index = j;
			}
		}
        //error.push_back(min_dist);

        if (min_dist > dis_th && min_dist < 0.3)
        {
            error.push_back(0.0005);
            outPoints(0,i) = pts2(0,index);
            outPoints(1,i) = pts2(1,index);     // pts1 change, and outpoints follow the index in pts1, pts2 stay same
        }
        else
        {   
            error.push_back(min_dist);
            outPoints(0,i) = pts1(0,i);
            outPoints(1,i) = pts1(1,i);          // if < dis_th , stay same
            cout<<"TOO NEAR OR TOO FAR"<<endl;
        }  
        
	}

}

Eigen::Matrix3d icp_lm::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{   // src -> tar ; tar -> src
    // TODO: please code by yourself
    int N = src.cols();     // size
    // R , P, AND T
    Eigen::Matrix2d R_;
	R_ << 1, 0, 0, 1;
	Eigen::Vector2d P_;
	P_[0] = P_[1] = 0;
    Eigen::Matrix3d T_;
    T_ << R_(0, 0), R_(0, 1), P_[0],
		    R_(1, 0), R_(1, 1), P_[1],
		    0, 0, 1;
    
    //去中心化
    //计算点云中心坐标
    Eigen::Matrix2d p1, p2;
	p1(0,0) = 0;
	p1(0,1) = 0;
	p2(0,0) = 0;
	p2(0,1) = 0;

	for (int i = 0; i < N; i++)
	{
		p1(0,0) += src(0,i);
        p1(0,1) += src(1,i);
		p2(0,0) += tar(0,i);
        p2(0,1) += tar(1,i);
	}
    p1 /= N;
    p2 /= N;

	//去中心化
	Eigen::MatrixXd q1 = Eigen::MatrixXd::Zero(2,N);
    Eigen::MatrixXd q2 = Eigen::MatrixXd::Zero(2,N);
	for (int i = 0; i < N; i++)
	{
		q1(0,i) = src(0,i) - p1(0,0);
        q1(1,i) = src(1,i) - p1(0,1);
		q2(0,i) = tar(0,i) - p2(0,0);
        q2(1,i) = tar(1,i) - p2(0,1);
	}

    // cal pc1*pc2^T
	Eigen::Matrix2d W = Eigen::Matrix2d::Zero();    // then cal W

    for (int i=0; i<N; i++)
    {
        W += Eigen::Vector2d(q1(0,i), q1(1,i)) * Eigen::Vector2d(q2(0,i),q2(1,i)).transpose();
        //cout << "DEBUG1" << endl;
    }
    cout << "W = " << W << endl;

    // SVD on W
	Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix2d U = svd.matrixU();
	Eigen::Matrix2d V = svd.matrixV();

    R_ = U* (V.transpose());
	P_ = Eigen::Vector2d(p1(0,0), p1(0,1)) - R_ * Eigen::Vector2d(p2(0,0), p2(0,1));
    T_ << R_(0, 0), R_(0, 1), P_[0],
		    R_(1, 0), R_(1, 1), P_[1],
		    0, 0, 1;

    return T_;
}

float icp_lm::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
    float dist = sqrt(pow(pta[0] - ptb[0], 2) + pow(pta[1] - ptb[1], 2));
    return dist;
}

Eigen::Matrix3d icp_lm::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp_lm::publishResult(Eigen::Matrix3d T)
{	
    float delta_yaw = atan2(T(1,0), T(0,0));
    cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

    // tf
    double temp = -sensor_sta(2);   // try
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(temp);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "world_base";      // world_base
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = -sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    geometry_msgs::TransformStamped ept;
    ept.header.stamp = ros::Time::now();
    ept.header.frame_id = "map";     
    ept.child_frame_id = "world_base";
    ept.transform.translation.x = 0;
    ept.transform.translation.y = 0;
    ept.transform.translation.z = 0;
    geometry_msgs::Quaternion ept_quat = tf::createQuaternionMsgFromYaw(0);
    ept.transform.rotation = ept_quat;
    odom_broadcaster.sendTransform(ept);

    odom_broadcaster.sendTransform(odom_trans);     // SUSPEND FOR EKF
    
    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";    // world_base

    odom.pose.pose.position.x = -sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_landMark");
    ros::NodeHandle n;

    icp_lm icp_lm_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}