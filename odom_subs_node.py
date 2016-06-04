#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid 

def callback(odom, occu_grid):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", Odometry.header,occu_grid.header)
    print (odom.header)
    print ("*********************************")
    print (occu_grid.header)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    #fileDes = open('/home/osher/Desktop/map_data','w')
    width = occu_grid.info.width
    height = occu_grid.info.height
    map = [[0 for x in range(width)] for x in range(height)]
    for i in range(width):
        for j in range(height):
            if occu_grid.data[j*width+i] == 0:
                map[j][i] = '0'
            elif occu_grid.data[j*width+i] > 0:
                map[j][i] = '3'
            elif occu_grid.data[j*width+i] == -1:
                map[j][i] = '1'
    #print(map)
    #print ("*************************************************")
    #print (odom.pose)
    #print ("*************************************************")
    #print (occu_grid.info.origin)
    #print (occu_grid.info.resolution)
    print ("*************************************************")
    grid_x = int(round((odom.pose.pose.position.x - occu_grid.info.origin.position.x) / occu_grid.info.resolution,0))
    grid_y = int(round((odom.pose.pose.position.y - occu_grid.info.origin.position.y) / occu_grid.info.resolution,0))
    map[grid_x][grid_y] = '9'
    print(map) 
    #print ("grid x of car:")
    #print(grid_x)
    #print ("grid y of car:")
    #print(grid_y)
    #fileDes.write(map)
    #fileDes.close()
def listener(): 

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    occu_grid_sub = message_filters.Subscriber('rtabmap/proj_map', OccupancyGrid)
    odom_sub = message_filters.Subscriber('rtabmap/odom', Odometry)

    ts = message_filters.TimeSynchronizer([odom_sub, occu_grid_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    listener()