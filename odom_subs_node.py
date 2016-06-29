#!/usr/bin/env python
import roslib
import rospy
import tf
from math import atan2,asin,pi
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid 
from coordinate import coordinate
import tile
import time
import sys

class map_subscriber():
    """
    This class is responsible for fetching the data from the ROS
    and allows several other options.
    """

    _initial_location = None
    _board = None
    _direction = None
    _tile = tile.tile()
    _tileVal_obstacle = _tile.get_WallVal()
    _tileVal_free = _tile.get_FreeVal()
    _tileVal_unmapped = _tile.get_UnmappedVal()
    _tileVal_car = _tile.get_CarVal()
    _listener = None

    def __init__(self):
        print "Creating new subscriber\n"
        self._map_index = 0
        self.listener()

    def __str__(self):
        print "Current map:\n{0}".format(self._current_map)

    def refresh_board(self,odom, occu_grid):

        width = occu_grid.info.width
        height = occu_grid.info.height
        map = [[0 for x in range(width)] for x in range(height)]
        for i in range(width):
            for j in range(height):
                if occu_grid.data[j*width+i] == 0:
                    map[j][i] = self._tileVal_free
                elif occu_grid.data[j*width+i] > 0:
                    map[j][i] = self._tileVal_obstacle
                elif occu_grid.data[j*width+i] == -1:
                    map[j][i] = self._tileVal_unmapped

        grid_y = int(round((odom.pose.pose.position.x - occu_grid.info.origin.position.x) / occu_grid.info.resolution,0))
        grid_x = int(round((odom.pose.pose.position.y - occu_grid.info.origin.position.y) / occu_grid.info.resolution,0))
        map[grid_y][grid_x] = self._tileVal_car
        while True:
            try:
                (trans,rot) = self._listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        euler = tf.transformations.euler_from_quaternion(rot)
        yaw = euler[2]
        yaw = (yaw*180/pi)

        if self._initial_location is None:
            self._initial_location = coordinate(grid_x,grid_y)
        self._board = map
        self._direction = self.calcDirection(yaw)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)
        occu_grid_sub = message_filters.Subscriber('rtabmap/proj_map', OccupancyGrid)
        odom_sub = message_filters.Subscriber('rtabmap/odom', Odometry)

        ts = message_filters.TimeSynchronizer([odom_sub, occu_grid_sub], 10)
        ts.registerCallback(self.refresh_board)
        flag = False
        while not flag : 
            try:
                self._listener = tf.TransformListener()
            except:
                continue
            flag = True
        time.sleep(5)
        #rospy.spin()

    def get_initial_coordinate(self):
        """
        getter method for initial car location
        :return: coordinate instance, coordinate with 0 value upon failure
        """
        if self._initial_location is None:
            return coordinate.coordinate(0,0)
        try:
            return self._initial_location
        except:
            return coordinate.coordinate(0,0)

    def get_board(self):
        """
        getter method to access the map data
        :return: (map_instance, index) upon success, False otherwise
        """
        try:
            return self._current_map,self._map_index
        except:
            return False

    def calcDirection(self, yaw):
        if -22.5<= yaw < 22.5:
            return 1
        if 22.5<= yaw < 67.5:
            return 2
        if 67.5<= yaw < 112.5:
            return 3
        if 112.5<= yaw < 157.5:
            return 4
        if -157.5 > yaw or yaw >= 157.5:
            return 5
        if -157.5<= yaw < -112.5:
            return 6
        if -112.5<= yaw < 67.5:
            return 7
        if -67.5<= yaw < -22.5:
            return 8
        return 1


if __name__ == '__main__':
    print("this main")
    sub_instance = map_subscriber()
    print (sub_instance.get_board())
