#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

class mapping{

public:
    mapping(ros::NodeHandle &n);
	~mapping();
    ros::NodeHandle& n;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber landMark_sub;
    //ros::Subscriber ekfpos_sub;
    ros::Publisher map_pub;
    tf::TransformListener listener;
    // transform
    tf::StampedTransform transform;
    // global grid map
    nav_msgs::OccupancyGrid grid_map;   // map, operate and then send out
    // some variables
    string world_frame, sensor_frame;
    int map_height, map_width;
    float map_res;
    double judge_ratio;
    double P_occu, P_free;
    double th_occu, th_free;
    // grid points location
    MatrixXd grid_points;
    
    // main process
    void process(sensor_msgs::LaserScan input);
    //void update_pos(nav_msgs::Odometry odom);
    void update_landmark(visualization_msgs::MarkerArray input);
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);

    // status   状态量 save pos in ekf status(0,0),status(1,0),status(2,0)
    MatrixXd status;

    double angleNorm(double angle);
    // calc 2D distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    void update_map(const double ratio, const int id);

    const double true_x = 10;
    const double true_y = 10;

};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n):
    n(n)
{
    // get the params
    n.getParam("/bayes_mapping/world_frame", world_frame);
	n.getParam("/bayes_mapping/sensor_frame", sensor_frame);

	n.getParam("/bayes_mapping/map_height", map_height);
	n.getParam("/bayes_mapping/map_width", map_width);
	n.getParam("/bayes_mapping/map_res", map_res);
    n.getParam("/bayes_mapping/judge_ratio", judge_ratio);

    n.getParam("/bayes_mapping/P_occu", P_occu);    // occu in bayes
    n.getParam("/bayes_mapping/P_free", P_free);    // free in bayes
    n.getParam("/bayes_mapping/th_occu", th_occu); 	  // occu in th
    n.getParam("/bayes_mapping/th_free", th_free);    // free in th
    
    // iniitialization
    grid_map.info.height = map_height;
    grid_map.info.width = map_width;
    grid_map.info.resolution = map_res;
    grid_map.header.frame_id = world_frame;

    // set origin of map
    grid_map.info.origin.position.x = - float(grid_map.info.width) / 2 * grid_map.info.resolution;
    grid_map.info.origin.position.y = - float(grid_map.info.height) / 2 * grid_map.info.resolution;
    grid_map.info.origin.orientation.w = 1;

    // fill with -1 or 50  / unknown in the map
    grid_map.data.assign(map_width * map_height, 55);
    double initial = -true_x + map_res/2;
    grid_points = Eigen::MatrixXd::Zero(map_width * map_height,3);  // x, y, prob_log   initial is zero
    for (size_t i=0; i < map_height; i++)
    {
        for (size_t j=0; j < map_width; j++)
        {
            grid_points(map_width*i + j,0) = initial + j*map_res;
            grid_points(map_width*i + j,1) = initial + i*map_res;
        }
    }
    cout<<grid_points<<endl;

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::process, this);
    landMark_sub = n.subscribe("/landMarks", 1, &mapping::update_landmark, this);
    //ekfpos_sub = n.subscribe("/ekf_slam", 1, &mapping::update_pos, this);
}

void mapping::process(sensor_msgs::LaserScan input)
{
    /* LaserScan:
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges         range data [m] (Note: values < range_min or > range_max should be discarded)   ranges.at(i) : distance
float32[] intensities

OccupancyGrid:
实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[2*(x+10)+40(y+10)]
    */
    cout<<"------seq:  "<<input.header.seq<<endl;
    
    // transformation is needed
    //cout<<"frame name:  "<<world_frame<<sensor_frame<<endl;
    listener.waitForTransform(world_frame, sensor_frame, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform(world_frame, sensor_frame, ros::Time(0), transform);

    /* transform: get x,y,yaw in world base */

    double cur_x = transform.getOrigin().x();
    double cur_y = transform.getOrigin().y();

    double roll, pitch, yaw;    //定义存储r,p,y的变量
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);   //进行转换
    
    yaw = angleNorm(yaw);

    cout<<"raw_yaw:  "<< yaw <<endl;
    yaw  = angleNorm(yaw);
    cout<<"pos:  "<<cur_x<<' '<<cur_y<<' '<< yaw <<endl;

    // TODO: Please complete your mapping code
    const double ang_min = input.angle_min;
    const double ang_max = input.angle_max;
    const double ang_inc = input.angle_increment;   // angular distance between measurements [rad]
    const double range_max = input.range_max;
    const double range_min = input.range_min;
    double laser_size = input.ranges.size();    // laser nums   ranges.at(i) : distance
    //cout<<"range_max: "<<range_max<<endl;

    /* set increase step  */
    const float cell_size = map_res;
    const double inc_step = 1.0 * cell_size;
    /* for every laser beam */
    for(size_t i = 0; i < laser_size; i ++)
    {
        /* get laser end point dist */
        double R = input.ranges.at(i);
        double angle = ang_inc * i + ang_min + yaw;     // + yaw
        angle = angleNorm(angle);
        //cout<<"i:  "<<i<<" R: "<<R<<endl;

        if (R < range_max && R >= range_min)       // reach obstacles
            {
               for(double r = 0; r < R + cell_size; r += inc_step)  // follow laser beam, inc_step++
               {
                    double mx = r* cos(angle) + cur_x;
                    double my = r* sin(angle) + cur_y;     // end point, obstacle
                    int flag;   // narrow search area
                    if(my<=0)
                        flag = 0;
                    else
                        flag = 1;
                
                    Eigen::Vector2d m;
                    m(0) = mx;
                    m(1) = my;
                    double min = 10;
                    int id = 0;
                    for (int k= flag*map_width * map_height/2; k<(flag+1)*map_width * map_height/2; k++)   // find nearest grid middle point
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

                    if (r <= R- 0.5*cell_size && min < cell_size/3)  // blank
                    {
                        grid_points(id,2) += log( P_free / (1.0 - P_free) );    // update free
                    }
                        //grid_map.data[id] = 0;      // not reach end, reachable
                    else if (r >= R + 0.5*cell_size && min < cell_size/3)   // obs
                    {
                        grid_points(id,2) += log( P_occu / (1.0 - P_occu) );    // update occu
                    }

                    update_map(grid_points(id,2), id);
                }

                double mx = R* cos(angle) + cur_x;
                double my = R* sin(angle) + cur_y;     // end point, obstacle
                int flag;   // narrow search area
                if(my<=0)
                    flag = 0;
                else
                    flag = 1;
                
                Eigen::Vector2d m;
                m(0) = mx;
                m(1) = my;
                //cout<<"i:  "<<i<<" end point: "<<m<<endl;
                double min = 10;
                int id = 0;
                for (int k= flag*map_width * map_height/2; k<(flag+1)*map_width * map_height/2; k++)   // find nearest grid middle point
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
                
                if (min < cell_size/2.5){
                    grid_points(id,2) += log( P_occu / (1.0 - P_occu) );    // update occu
                    update_map(grid_points(id,2), id);
                }

            }
/*
        else if (R > range_max)   // no obs
        {
            for(double r = 0; r < range_max; r += inc_step)  // 沿着激光射线以inc_step步进，更新地图
               {
                    double mx = r* cos(angle) + cur_x;
                    double my = r* sin(angle) + cur_y;     // end point, obstacle
                    int flag;   // narrow search area
                    if(my<=0)
                        flag = 0;
                    else
                        flag = 1;
                
                    Eigen::Vector2d m;
                    m(0) = mx;
                    m(1) = my;
                    double min = 10;
                    int id = 0;
                    for (int k= flag*map_width * map_height/2; k<(flag+1)*map_width * map_height/2; k++)   // find nearest grid middle point
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
                    
                    grid_points(id,3) += 1.0;
                    grid_points(id,4) = double(grid_points(id,2)/(grid_points(id,2) + grid_points(id,3)));
                    update_map(grid_points(id,4), id);

                }

        }*/


    }

    

    // publish
    //cout<<grid_points<<endl;
    map_pub.publish(grid_map);
}

void mapping::update_landmark(visualization_msgs::MarkerArray input)
{
    Eigen::MatrixXd tar_pc;
    tar_pc = this->landMarksToMatrix(input);
    for (size_t i = 0; i < tar_pc.cols(); i++)
    {
        
        Eigen::Vector2d m;
        m(0) = tar_pc(0,i);
        m(1) = tar_pc(1,i);
        double min = 10;
        int id = 0;
        for (int k= 0; k<map_width * map_height; k++)   // find nearest grid middle point
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
        
        if (min < map_res/3){   // obs, occu
            grid_points(id,2) += log( P_occu / (1.0 - P_occu) );    // update occu
            update_map(grid_points(id,2), id);
           // update_map(grid_points(id,4), id);
        }

    }
}

Eigen::MatrixXd mapping::landMarksToMatrix(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();
    //cout<<markerSize<<" markers received !"<<endl;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void mapping::update_map(const double prob_log, const int id)
{
    /*
    if (ratio > judge_ratio && grid_points(id,2) > 30)
        grid_map.data[id] = 100;
    else if (grid_points(id,3) > 30)
        grid_map.data[id] = 0;
    else
        grid_map.data[id] = -1;*/
    
    double prob = 1.0 - 1.0 / (1 + exp(prob_log));
    //int pub_data = 100*prob;
    
    if (prob >= th_occu*P_occu)
    {
        grid_map.data[id] = 100;
    }
    else if (prob <= th_free*P_free)
    {
        grid_map.data[id] = 0;
    }
    else
    {
        grid_map.data[id] = 50; // SACN BUT TBD
    }

}

double mapping::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

float mapping::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle n;

    mapping mapping_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}