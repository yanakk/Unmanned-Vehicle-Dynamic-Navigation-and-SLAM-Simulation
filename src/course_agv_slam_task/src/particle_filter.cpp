#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

#define particle_num 50
typedef struct particle {
    int id;
    float x;
    float y;
    float theta;
    float weight;
} particle;

class particle_filter{

public:
    particle_filter(ros::NodeHandle &n);
	~particle_filter();
    ros::NodeHandle& n;

    // some params
    float init_x;
    float init_y;
    float init_theta;
    float init_rand_xy;
    float init_rand_theta;
    float th; // th about resample
    string world_frame, robot_frame;
    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher particles_pub;
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // particles 
    particle particles[particle_num];

    // global state
    Eigen::Vector3d state;  // 定位位姿
    Eigen::Vector3d pos;
    // map
    nav_msgs::OccupancyGrid global_map;
    bool isMapSet = false;
    MatrixXd grid_points;   // map points

    // set map
    void setMap(nav_msgs::OccupancyGrid input);
    // init the state and particles 
    void init();
    // do Motion from ICP odom
    void doMotion(nav_msgs::Odometry input);
    // do Observation to weight each particle
    void doObservation(sensor_msgs::LaserScan input);
    // publish the particles and the estimated state
    void publishAll();
    // angle normalization
    double angleNorm(double angle);
    // weight normalization
    void weightNorm();
    // re-sample the particle according to the weights
    void resampling();
    // get the final pose 
    void getFinalPose();
    // gen new particle 
    particle genNewParticle();
    // gaussian
    float gaussrand(double E, double V);
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);

    double pre_x, pre_y, pre_yaw;

};

particle_filter::~particle_filter()
{}

particle_filter::particle_filter(ros::NodeHandle& n):
    n(n)
{
    n.getParam("/particle_filter/init_x", init_x);
    n.getParam("/particle_filter/init_y", init_y);
    n.getParam("/particle_filter/init_theta", init_theta);
    //n.getParam("/particle_filter/particle_num", particle_num);
    //particle particles[particle_num];

    n.getParam("/particle_filter/init_rand_xy", init_rand_xy);
    n.getParam("/particle_filter/init_rand_theta", init_rand_theta);
    n.getParam("/particle_filter/th", th);
    n.getParam("/particle_filter/world_frame", world_frame);
	n.getParam("/particle_filter/robot_frame", robot_frame);

    this->init();

    particles_pub = n.advertise<visualization_msgs::MarkerArray>("particles", 0, true);
    map_sub = n.subscribe("/map", 1, &particle_filter::setMap, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &particle_filter::doObservation, this);
    odom_sub = n.subscribe("/icp_odom", 1, &particle_filter::doMotion, this);
}

void particle_filter::setMap(nav_msgs::OccupancyGrid input)
{   
    // set once at first time
    if(!isMapSet)
    {   
        cout<<"init the global occupancy grid map"<<endl;
        this->global_map = input;
        isMapSet = true;

        const float map_height = global_map.info.height;    // 129
        const float map_width = global_map.info.width;      // 129
        const float map_res = global_map.info.resolution;   // 0.155
        cout<<map_height<<' '<<map_width<<' '<<map_res<<endl;
        float  initial = -map_height*map_res*0.5 + map_res/2;
        grid_points = Eigen::MatrixXd::Zero(map_width * map_height,2);  // x, y   initial is zero
        for (size_t i=0; i < map_height; i++)
        {
            for (size_t j=0; j < map_width; j++)
            {
                grid_points(map_width*i + j,0) = initial + j*map_res;
                grid_points(map_width*i + j,1) = initial + i*map_res;
            }
        }
        cout<<grid_points<<endl;
    }
    // if self map, update here

}

void particle_filter::init()
{   
    // set state
    state << 0, 0, 0;

    for(int i=0; i<particle_num; i++)
    {   
        particles[i].id = i;
        particles[i].x = init_x + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].y = init_y + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].theta = init_theta + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_theta - init_rand_theta;
        particles[i].weight = float(1/float(particle_num)); // same weight
    }
    publishAll();
}

void particle_filter::doMotion(nav_msgs::Odometry input)
{   
    cout<<"doing Motion"<<endl;
    // TODO: Motion Model
    /*float cur_x = input.pose.pose.position.x;
    float cur_y = input.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(input.pose.pose.orientation, quat);
    double roll, pitch, yaw; 
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);*/ // ok
    listener.waitForTransform(world_frame, robot_frame, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
    float cur_x = transform.getOrigin().x();
    float cur_y = transform.getOrigin().y();
    double roll, pitch, yaw;    // r,p,y
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);   //进行转换
    
    yaw = angleNorm(yaw);
    pos(0)=cur_x; pos(1)=cur_y; pos(2)=yaw;

    float delta_x = cur_x-pre_x;
    float delta_y = cur_y-pre_y;
    float delta_yaw = angleNorm(yaw-pre_yaw);

    for(int i=0; i<particle_num; i++)   // update
    {   
        //particles[i].id = i;
        particles[i].x += delta_x;
        particles[i].y += delta_y;
        particles[i].theta += delta_yaw;
        particles[i].theta = angleNorm(particles[i].theta);
    }
    pre_x = cur_x;
    pre_y = cur_y;
    pre_yaw = yaw;

}

void particle_filter::doObservation(sensor_msgs::LaserScan input)
{
    cout<<"doing observation"<<endl;
    cout<<"------seq:  "<<input.header.seq<<endl;
    const double ang_min = input.angle_min;
    const double ang_max = input.angle_max;
    const double ang_inc = input.angle_increment;   // angular distance between measurements [rad]
    const double range_max = input.range_max;
    const double range_min = input.range_min;
    double laser_size = input.ranges.size();    // laser nums   ranges.at(i) : distance
    
    // TODO: Measurement Model
    if(isMapSet){
    for(int i=0; i<particle_num; i++)   // 遍历粒子
    {
        float p_x = particles[i].x;
        float p_y = particles[i].y;
        Eigen::Vector2d pf(p_x, p_y);
        Eigen::Vector2d ps(pos(0), pos(1));
        float p_yaw = angleNorm(particles[i].theta);
        float rt_num = 0.0;
        float cnt = 1.0;
        for(size_t j = 0; j < laser_size; j ++) // 遍历激光
        {
            /* get laser end point dist */
            double R = input.ranges.at(j);
            if (R < range_max && R > range_min)
            {
                cnt ++; // effective laser num++
                double angle = ang_inc * j + ang_min + p_yaw;     // + yaw
                double mx = R* cos(angle) + p_x;    // pos in "global"
                double my = R* sin(angle) + p_y;
                if(fabs(mx)>=10 || fabs(my)>=10)    // out of range
                    continue;

                Eigen::Vector2d m(mx, my);
                // find nearest grid point
                int flag;   // narrow search area
                if(my<=0)
                    flag = 0;
                else
                    flag = 1;
                double min = 10;
                int id = 0;
                for (int k= flag*global_map.info.width *global_map.info.height/2; k<(flag+1)*global_map.info.width * global_map.info.height/2; k++)   // find nearest grid middle point
                {
                    Eigen::Vector2d p;
                    p(0) = grid_points(k,0);
                    p(1) = grid_points(k,1);
                    float dist = calc_dist(m,p);
                    if (dist < min)
                    {
                        min = dist;
                        id = k;
                    }
                }

                // judge if corret
                if (global_map.data[id] > 30)
                    rt_num ++;    // ob
            
            }

        }
        float ratio = rt_num/cnt;
        particles[i].weight = ratio;
        //cout<<i<<": "<<particles[i].weight<<endl;
        //particles[i].weight = 1.0/calc_dist(pf, ps);

    }
    }
    // 权重归一化
    weightNorm();

    // 重采样  注：先判断有无必要
    if(input.header.seq % 15 == 0)
        resampling();
    getFinalPose();
    publishAll();
}

void particle_filter::resampling()
{
    // TODO: Resampling Step
    cout<<"------RESAMPLING------"<<endl;
    cout<<pos<<endl;
    particle best;
    best.weight = 1.0/float(particle_num); // initial
    for(int i=0; i<particle_num; i++) // find best
    {
        if(particles[i].weight > best.weight)
            best = particles[i];
    }
    cout<<"best id:  "<<best.id<<" "<<best.x<<" "<<best.y<<" "<<best.theta<<" "<<best.weight<<endl;
    for(int i=0; i<particle_num; i++)   // detect low weight particles
    {   
        if(particles[i].weight <= (1.0/float(particle_num))*th )
        {
            float noisex = gaussrand(0, 0.4);
            //cout<<"noisex: "<<noise<<endl;
            particles[i].x = best.x + noisex;
            float noisey = gaussrand(0, 0.4);
            particles[i].y = best.y + noisey;
            float noiset = gaussrand(0, 0.1);
            particles[i].theta = best.theta + noiset/5.0;
        }
    }

    // 完成后权重全部初始化
    for(int i=0; i<particle_num; i++)
    {   
        particles[i].id = i;
        particles[i].weight = float(1/float(particle_num)); // same weight
    }

}

void particle_filter::getFinalPose()
{   
    // TODO: Final State Achieve
    float final_x, final_y, final_yaw;
    for(int i=0; i<particle_num; i++)   // update
    {   
        //particles[i].id = i;
        final_x += particles[i].x;
        final_y += particles[i].y;
        final_yaw += particles[i].theta;
    }
    final_x /= float(particle_num);
    final_y /= float(particle_num);
    final_yaw = angleNorm(final_yaw/float(particle_num));

    state(0) = final_x;
    state(1) = final_y;
    state(2) = final_yaw;

}

particle particle_filter::genNewParticle()
{
    // TODO: Generate New Particle
}

float particle_filter::gaussrand(double E, double V) // gaussian
{
	static double V1, V2, S;
	static int phase = 0;
	float X;
	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);
		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);
	phase = 1 - phase;
	return X = X * V + E;
}

void particle_filter::weightNorm()
{
    float sum;
    for(int i=0; i<particle_num; i++)
    {   
        sum += particles[i].weight;
    }

    for(int i=0; i<particle_num; i++)
    {   
        particles[i].weight /= sum;
    }

}

double particle_filter::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

float particle_filter::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

void particle_filter::publishAll()
{
    visualization_msgs::MarkerArray particle_markers_msg;
    particle_markers_msg.markers.resize(particle_num);

    for(int i=0; i<particle_num; i++)
    {
        particle_markers_msg.markers[i].header.frame_id = "world_base";    // map
        particle_markers_msg.markers[i].header.stamp = ros::Time::now();
        particle_markers_msg.markers[i].ns = "particle";
        particle_markers_msg.markers[i].id = i;
        particle_markers_msg.markers[i].type = visualization_msgs::Marker::ARROW;
        particle_markers_msg.markers[i].action = visualization_msgs::Marker::ADD;
        particle_markers_msg.markers[i].pose.position.x = particles[i].x;
        particle_markers_msg.markers[i].pose.position.y = particles[i].y;
        particle_markers_msg.markers[i].pose.position.z = 0; // add height for viz ?
        particle_markers_msg.markers[i].pose.orientation.x = 0.0;
        particle_markers_msg.markers[i].pose.orientation.y = 0.0;
        particle_markers_msg.markers[i].pose.orientation.z = sin(particles[i].theta/2);
        particle_markers_msg.markers[i].pose.orientation.w = cos(particles[i].theta/2);
        particle_markers_msg.markers[i].scale.x = 0.1;
        particle_markers_msg.markers[i].scale.y = 0.02;
        particle_markers_msg.markers[i].scale.z = 0.05;
        particle_markers_msg.markers[i].color.a = particles[i].weight * particle_num / 2; // Don't forget to set the alpha!
        //particle_markers_msg.markers[i].color.a = 0.5;
        particle_markers_msg.markers[i].color.r = 1.0;
        particle_markers_msg.markers[i].color.g = 0.0;
        particle_markers_msg.markers[i].color.b = 0.0;
    }
    cout<<"particle [0]:  "<<particles[0].x<<' '<<particles[0].y<<endl;
    particles_pub.publish(particle_markers_msg);

    // tf
    geometry_msgs::Quaternion quat_ = tf::createQuaternionMsgFromYaw(state(2));

    geometry_msgs::TransformStamped pf_trans;
    pf_trans.header.stamp = ros::Time::now();
    pf_trans.header.frame_id = "world_base";
    pf_trans.child_frame_id = "pf_loc";

    pf_trans.transform.translation.x = state(0);
    pf_trans.transform.translation.y = state(1);
    pf_trans.transform.translation.z = 0.0;
    pf_trans.transform.rotation = quat_;

    geometry_msgs::TransformStamped ept;
    ept.header.stamp = ros::Time::now();
    ept.header.frame_id = "map";     
    ept.child_frame_id = "world_base";
    ept.transform.translation.x = 0;
    ept.transform.translation.y = 0;
    ept.transform.translation.z = 0;
    geometry_msgs::Quaternion ept_quat = tf::createQuaternionMsgFromYaw(0);
    ept.transform.rotation = ept_quat;
    //tf_broadcaster.sendTransform(ept);

    tf_broadcaster.sendTransform(pf_trans);

    // OR publish others you want

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    particle_filter particle_filter_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}