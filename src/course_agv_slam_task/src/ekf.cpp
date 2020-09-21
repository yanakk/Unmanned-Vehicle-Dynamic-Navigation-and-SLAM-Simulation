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
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta; 
    // match threshold  最近邻匹配阈值
    float match_th;
    float same_th;
    // bool
    bool isFirstScan;
    // status   状态量 final publish status(0,0),status(1,0),status(2,0)
    MatrixXd status;
    // covariance   协方差
    MatrixXd covariance;
    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num     input by human in launch files
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
    // count the non-zero elements in status
    int nonZero_cnt;    // bottom index+1, initial = 3
    //int start = 0;
    
    // init all 
    void initAll();
    // predict phase    laser
    void predict(nav_msgs::Odometry odom);
    // update phase     mag
    void update(visualization_msgs::MarkerArray input);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // landMarks to r-phi matrix
    Vector2d cartesianToPolar(double x, double y);  // 笛卡尔坐标转极坐标，得到距离和角度
    // update feature map
    void updateFeatureMap(Eigen::MatrixXd newFeatures);
    // get motion Jacobian (F?)
    MatrixXd getMotionJacobian();
    // get observation Jacobian (H?)
    MatrixXd getObservJacobian(double q, double sqrt_q, double delta_x, double delta_y);
    // angle normalization
    double angleNorm(double angle);
    // calc 2D distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // find nearest map points
    int findNearestMap(Vector2d point);
    int IsInPool(float x, float y);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
    tf::TransformBroadcaster ekf_broadcaster;
    ros::Publisher odom_pub;
    void publishResult();
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/same_th", same_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    this->initAll();

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this);    // Publish is in the update
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_slam", 1);
}

void ekf::predict(nav_msgs::Odometry odom)
{
    // TODO: Please complete the predict phase or motion model
    // odom-Quaternion  四元数储存角度; 可以考虑利用一个空的直接存角度?
    //status = Eigen::VectorXd::Zero(3+2*landMark_num);
    status(0,0) = odom.pose.pose.position.x;
    status(1,0) = odom.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    double roll, pitch, yaw;    //定义存储r,p,y的变量
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);   //进行转换
    yaw = angleNorm(yaw);
    status(2,0) = yaw;  // *
    
    int N = status.rows();
    Eigen::MatrixXd F(N, 3);    // Nx3
    F.setZero();
    F.block(0,0, 3, 3) = Eigen::Matrix3d::Identity();
    covariance = covariance + F *noise_R* F.transpose();    

}

void ekf::update(visualization_msgs::MarkerArray input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    MatrixXd landMarkFeatures = this->landMarksToXY(input);
    cout<<"-------------New LM Cnt:    "<<landMarkFeatures.cols()<<endl;

    // TODO: Please complete the update phase or observation model
    // landMarkFeatures(0,i)-x  landMarkFeatures(1,i)-y
    // initial

    if(isFirstScan)
    {
        cout<<"FIRST SCAN"<<endl;
        this->updateFeatureMap(landMarkFeatures);
        return;
    }
    
    for (int i=0; i<landMarkFeatures.cols(); i++)
    {
        double x = status(0,0);
        double y = status(1,0);
        double zx = landMarkFeatures(0,i);  // in robo coord
        double zy = landMarkFeatures(1,i);
        Eigen::Vector2d z = this->cartesianToPolar(zx,zy); //(0)-r, (1)-phi
        double angle = status(2,0) + z(1);    // angle to robopole
        angle = angleNorm(angle);
        double mx = z(0)* cos(angle) + status(0,0);   // world coord
        double my = z(0)* sin(angle) + status(1,0);
        noise_Q << 0.0001*z(0)*z(0),0.0, 0.0, 0.00005*z(1)*z(1); // noise Q relates to pos
        

        int index = this->IsInPool(mx, my); // index of landmark in status

        if (fabs(mx)>8.5 || fabs(my)>9.1 || index == 1)   // 地图边缘为路标判定错误的情况，直接不算;在两个阈值中间，也不算
            continue;
        
        if (index == 0)     // not in pool, new landmark
        {
            // cal jacobian
            Eigen::Matrix3d sigma_xi = covariance.block(0,0,3,3);   //点的协方差
            Eigen::Matrix<double, 2, 3> Gp;     // 关于机器人位姿的雅克比
            Gp <<1, 0, -z(0)*sin(angle),
                 0, 1, z(0)*cos(angle);
            Eigen::Matrix2d Gz;     // 关于观测的雅克比
            Gz << cos(angle), -z(0)*sin(angle),
                  sin(angle), -z(0)*cos(angle);
            Eigen::Matrix2d sigma_m = Gp*sigma_xi*Gp.transpose() + Gz*noise_Q*Gz.transpose();   //新地图的协方差
            Eigen::MatrixXd Gmx;
            // this part of jacobian calculation REFERRED TO  https://zhuanlan.zhihu.com/p/45207081 
            Gmx.resize (2, status.rows());  // 关于原状态的雅可比
            Gmx.setZero();
            Gmx.block (0, 0, 2, 3) = Gp;
            Eigen::MatrixXd sigma_mx;
            sigma_mx.resize(2, status.rows());
            sigma_mx.setZero();
            sigma_mx = Gmx*covariance;  // 新地图点相对已有状态的协方差

            // add to status
            nonZero_cnt +=2;
            Eigen::MatrixXd tmp_st (nonZero_cnt, 1 );
            tmp_st.setZero();
            tmp_st << status, mx, my;   // resize will clear matrix, so make a temp
            status.resize (nonZero_cnt, 1);
            status= tmp_st;
            
            // expand covariance
            Eigen::MatrixXd tmp_sigma (nonZero_cnt, nonZero_cnt);
            tmp_sigma.setZero();
            tmp_sigma.block ( 0, 0, nonZero_cnt-2, nonZero_cnt-2 ) = covariance;
            tmp_sigma.block ( nonZero_cnt-2, nonZero_cnt-2, 2, 2 ) = sigma_m;
            tmp_sigma.block ( nonZero_cnt-2, 0, 2, nonZero_cnt-2 ) = sigma_mx;
            tmp_sigma.block ( 0, nonZero_cnt-2, nonZero_cnt-2, 2 ) = sigma_mx.transpose();

            covariance.resize ( nonZero_cnt, nonZero_cnt);
            covariance = tmp_sigma;

        }

        else    // in pool, old landmark, return index; status(index) = x coord, status(index+1) = y coord;         UPDATE
        {   
            Eigen::MatrixXd F(5, nonZero_cnt);
            F.setZero();
            F.block(0,0,3,3) = Eigen::Matrix3d::Identity();
            F(3, index) = 1;
            F(4, index+1) = 1;

            double mx_hat = status(index, 0);
            double my_hat = status(index+1, 0);
            double theta = status(2,0);
            double delta_x = mx_hat - x;
            double delta_y = my_hat - y;
            double phi_hat = atan2(delta_y, delta_x)- theta;
            phi_hat=angleNorm(phi_hat);
            double q = delta_x * delta_x + delta_y * delta_y;
            double sqrt_q = sqrt(q);

            Eigen::MatrixXd Hv;
            Hv = this->getObservJacobian(q, sqrt_q, delta_x, delta_y);
            Eigen::MatrixXd Ht = Hv * F;

            Eigen::Vector2d z_hat(sqrt_q, phi_hat);

            Eigen::MatrixXd K = covariance * Ht.transpose()*( Ht * covariance * Ht.transpose() + noise_Q ).inverse();   // K:Nx2

            status = status + K * (z - z_hat);
            //cout<<"K: "<<endl<<K<<endl;
            cout<<" z - z_hat: "<<z - z_hat<<endl;
            //cout<<" GAIN: "<<endl<< K * (z - z_hat)<<endl;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nonZero_cnt, nonZero_cnt);
            covariance = ( I - K * Ht) * covariance;

        }

    }
    

    this->publishResult();      // publish status

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

void ekf::initAll()
{   
    // TODO: You can initial here if you need
    status.resize(3,1);
    status.setZero();
    nonZero_cnt = 3;
    //noise_R = Eigen::Matrix3d::Zero();
    noise_R = 0.001*Matrix3d::Identity();
    noise_Q = Eigen::Matrix2d::Zero();
    covariance.resize(3,3);
    covariance.setZero();
    
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void ekf::updateFeatureMap(Eigen::MatrixXd newFeatures)     // UPDATE MAP, ONLY FIRST TIME
{   
    // TODO:  Please complete this function if you need
    if(isFirstScan)
    {   
        // initial the map by landmarks in first scan
        // newFeatures(0,i),(1,i)
        for (int i = 0; i < newFeatures.cols();i++)
        {
            nonZero_cnt +=2;
            Eigen::MatrixXd tmp_st (nonZero_cnt, 1 );
            tmp_st.setZero();
            tmp_st << status , newFeatures(0,i),newFeatures(1,i);   // resize will clear matrix, so make a temp
            status.resize (nonZero_cnt, 1);
            status= tmp_st;

            // expand covariance

        }
        covariance.resize(nonZero_cnt,nonZero_cnt);
        covariance.setZero();
        isFirstScan = false;
    }
    else
    {   
    }
}

/*int ekf::findNearestMap(Vector2d point)
{   
    // TODO: Please complete the NN search
}*/

Eigen::MatrixXd ekf::getMotionJacobian()
{
    // TODO: Please complete the Jocobian Calculation of Motion
    int N = landMark_num*2 +3;
    MatrixXd jacob_G;
    jacob_G = Eigen::MatrixXd::Identity(N, N);
    return jacob_G;
}

Eigen::MatrixXd ekf::getObservJacobian(double q, double sqrt_q, double delta_x, double delta_y)
{
    // TODO: Please complete the Jocobian Calculation of Observation
    Eigen::MatrixXd Hv(2, 5);
    Hv << -sqrt_q * delta_x, -sqrt_q* delta_y, 0, sqrt_q*delta_x, sqrt_q*delta_y,
           delta_y, -delta_x, -q, -delta_y, delta_x;
    Hv = (1/q) * Hv;
    return Hv;

}

Vector2d ekf::cartesianToPolar(double x, double y)      // 笛卡尔坐标转极坐标，得到距离和角度
{
    float r = std::sqrt(x*x + y*y);
    float phi = angleNorm(std::atan2(y, x));
    Vector2d r_phi(r, phi);
    return r_phi;
}

float ekf::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

double ekf::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

int ekf::IsInPool(float x, float y)    // give back index
{
    if ( nonZero_cnt == 3)
        return 0;
    
    float min_dist = std::numeric_limits<float>::max();
	int index = 0;
    for (int i = 3; i < nonZero_cnt; i+=2)
    {
        float dist = sqrt(pow(x-status(i,0), 2) + pow(y-status(i+1,0), 2));
        if (dist < min_dist)
		{
			min_dist = dist;
			index = i;
		}
    }
    if (min_dist < match_th)
        return index;

    else if(min_dist < same_th)
        return 1;
    
    return 0;   // not in pool, return 0
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2,0));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    ekf_trans.header.frame_id = "world_base";       // world_base
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0,0);
    ekf_trans.transform.translation.y = status(1,0);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    cout<<"nonZero_cnt: "<<nonZero_cnt<<endl;
    //cout<<"status:"<<endl<<status<<endl;
    //cout<<"covariance:"<<endl<<covariance<<endl;
    geometry_msgs::TransformStamped ept;
    ept.header.stamp = ros::Time::now();
    ept.header.frame_id = "map";     
    ept.child_frame_id = "world_base";
    ept.transform.translation.x = 0;
    ept.transform.translation.y = 0;
    ept.transform.translation.z = 0;
    geometry_msgs::Quaternion ept_quat = tf::createQuaternionMsgFromYaw(0);
    ept.transform.rotation = ept_quat;
    ekf_broadcaster.sendTransform(ept);

    ekf_broadcaster.sendTransform(ekf_trans);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";    // world_base

    odom.pose.pose.position.x = status(0,0);
    odom.pose.pose.position.y = status(1,0);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}