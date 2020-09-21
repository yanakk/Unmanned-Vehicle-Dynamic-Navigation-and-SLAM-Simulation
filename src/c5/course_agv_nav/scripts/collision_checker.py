#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import numpy
import math

MAX_LASER_RANGE = 30

class Checker():
    def __init__(self):
        self.threshold = 0.4 # use to check collision
        self.oblsct = 10
        self.isreplan = False

        self.path = numpy.zeros([3,0])
        self.prepath = numpy.zeros([3,0])
        # x y yaw
        self.robot_x = numpy.zeros([3,1])
        # ros topic

        self.map_height = 80
        self.map_width = 80
        self.map_res = 0.25
        self.initial = -self.map_height*self.map_res*0.5 + self.map_res/2
        self.grid_points = numpy.zeros([self.map_width * self.map_height,2])
        for i in range(self.map_height):
            for j in range(self.map_width):
                self.grid_points[self.map_width*i + j,0] = self.initial + j*self.map_res
                self.grid_points[self.map_width*i + j,1] = self.initial + i*self.map_res
        print(self.grid_points)

        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)    # can be canceled
        self.map_sub = rospy.Subscriber('/grid_map_mine', OccupancyGrid, self.mapCallback)
        self.collision_pub = rospy.Publisher('/collision_checker_result',Bool,queue_size=5)
        self.marker_pub = rospy.Publisher('/collision_marker',Marker,queue_size=1)

    def poseCallback(self,msg):
        p = msg.position
        o = msg.orientation
        e = tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])
        self.robot_x = numpy.array([[p.x,p.y,e[2]]]).T

    def pathCallback(self,msg):
        self.path = self.pathToNumpy(msg) # shape (3,pose_num)
        print("get path! shape: ",self.path.shape)

    def mapCallback(self, msg):
        #print("get map msg: ", len(msg.data))
        if self.path.shape[1] == 0:
            pass
        # elif self.prepath.all() == self.path.all():
        #     pass # path not change
        else:
            # self.prepath = self.path   # tbd
            for i in range(self.path.shape[1]):
                p_pos = self.path[0:2, i]
                if p_pos[1] <= 0:
                    flag = 0    # in a half
                else:
                    flag = 1
                
                min = 10; id = 0
                for k in range(flag*self.map_width * self.map_height/2, (flag+1)*self.map_width * self.map_height/2):
                    p = numpy.zeros([2,])
                    p[0] = self.grid_points[k,0]; p[1] = self.grid_points[k,1]
                    dist = self.cal_dist(p_pos, p)
                    if dist < min:
                        min = dist
                        id = k
                if min < self.map_res and msg.data[id]>60:
                    self.publish_collision()
                    break
                    pass # path in obs
        
        pass

    def laserCallback(self,msg):
        #np_msg = self.laserToNumpy(msg)
        #obs = self.u2T(self.robot_x).dot(np_msg)
	    ## TODO update robot global pose give msg
        if self.collision_check(msg) and self.isreplan == True:
            self.publish_collision()
        else:
            self.isreplan = True

    def collision_check(self,msg):
        if self.path.shape[1] == 0:
            return False
	res = False #problem here
	#TODO
        count = 0
        range_min = msg.range_min
        for i in range(len(msg.ranges)):
            R = msg.ranges[i]
            if R < self.threshold and R >= range_min:
                count += 1
        if count > self.oblsct:
            res = True
            print('near count: ',count)
        return res

    def publish_collision(self):
        res = Bool()
        res.data = True
        self.isreplan = False
        self.collision_pub.publish(res)

    def cal_dist(self, pta, ptb):
        return math.sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]))

    def pathToNumpy(self,msg):
        pos = numpy.ones([0,3])
        for i in range(len(msg.poses)):
            p = msg.poses[i].pose.position
            pos = numpy.append(pos,[[p.x,p.y,1]],axis=0)
        return pos.T

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = numpy.ones([3,total_num])
        range_l = numpy.array(msg.ranges)
        range_l[range_l == numpy.inf] = MAX_LASER_RANGE
        angle_l = numpy.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = numpy.vstack((numpy.multiply(numpy.cos(angle_l),range_l),numpy.multiply(numpy.sin(angle_l),range_l)))
        return pc

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = numpy.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return numpy.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

def main():
    rospy.init_node('collision_checker_node')
    n = Checker()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
