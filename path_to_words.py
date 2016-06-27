def bestNeighbor(cost_so_far, dest):
    minVal = -1
    minTuple = (0,0)
    for i in range(-1,2):
        for j in range(-1,2):
            if ((i!=0) or (j!=0)):
                if ((dest[0]+i,dest[1]+j) in cost_so_far):
                    if ((cost_so_far[(dest[0]+i,dest[1]+j)] < minVal) or (minVal==-1)):
                        minVal = cost_so_far[(dest[0]+i,dest[1]+j)]
                        minTuple = (dest[0]+i,dest[1]+j)
    return minTuple


def calcPath(cost_so_far, dest, source, path):
    if (source == dest):
        path.insert(0,source)
        print (path)
        return path
    best = bestNeighbor(cost_so_far, dest) 
    path.insert(0, dest)
    return calcPath(cost_so_far,best,source,path)

def get_direction(curr_coor, next_coor):
  if curr_coor[0] > next_coor[0]:
    #top left corner
    if curr_coor[1] > next_coor[1]:
      return 8
    #above
    elif curr_coor[1] == next_coor[1]:
      return 1
    #top right corner
    else:
      return 2
  elif curr_coor[0] == next_coor[0]:
    #left
    if curr_coor[1] > next_coor[1]:
      return 7
    #right
    else:
      return 3
  elif curr_coor[0] < next_coor[0]:
    #bottom left corner
    if curr_coor[1] < next_coor[1]:
      return 4
    #below
    elif curr_coor[1] == next_coor[1]:
      return 5
    #bottom right corner
    else:
      return 6

def print_rotation(robot_direction,yaw):
  rotate = yaw - robot_direction
  direction = ""
  degree = 0
  if rotate >= 4 or (rotate < 0 and rotate >= -3):
    direction = "right"
    if rotate >= 4:
      degree = (8-rotate)*45
    else:
      degree = abs(rotate)*45
  else:
    direction = "left"
    if abs(rotate) >= 4:
      degree = (8-abs(rotate))*45
    else:
      degree = rotate*45
  print("turn "+str(degree)+" degrees to the "+direction)

def path(path, yaw):
  i = 0
  while i != len(path)-1:
    print_rotation(get_direction(path[i],path[i+1]),yaw)
    yaw = get_direction(path[i],path[i+1])
    i = i+1
    distance = 5
    while i != len(path)-1 and get_direction(path[i],path[i+1]) == yaw:
      distance = distance + 5
      i = i+1
    print("walk forward for " +str(distance)+" cm")


def get_path(path, yaw):
    """
    This method prints the path to the user
    :param path:
    :param yaw:
    :return: str describing the path to the destination goal
    """
    text = ''
    i = 0
    while i != len(path) - 1:
        print_rotation(get_direction(path[i], path[i + 1]), yaw)
        yaw = get_direction(path[i], path[i + 1])
        i = i + 1
        distance = 5
        while i != len(path) - 1 and get_direction(path[i], path[i + 1]) == yaw:
            distance = distance + 5
            i = i + 1
        text += "walk forward for {0} cm\n".format(str(distance))
    return text
  
#if __name__ == '__main__':
    #path([(5,5),(4,5),(3,6),(3,7),(4,8),(5,8),(6,7),(6,6),(5,5)], 1)
    #print("------------------------------------")
    #path([(5,5),(4,4),(3,3),(2,2),(1,1)], 7)
 #   i = 5
  #  for k in range(1,9):
   #   print("path([("+str(i)+","+str(i)+"),("+str(i-1)+","+str(i-1)+")],"+str(k)+")")
    #  print("path([("+str(i)+","+str(i)+"),("+str(i-1)+","+str(i)+")],"+str(k)+")")
     # print("path([("+str(i)+","+str(i)+"),("+str(i-1)+","+str(i+1)+")],"+str(k)+")")
     # print("path([("+str(i)+","+str(i)+"),("+str(i+1)+","+str(i-1)+")],"+str(k)+")")
     # print("path([("+str(i)+","+str(i)+"),("+str(i+1)+","+str(i)+")],"+str(k)+")")
     # print("path([("+str(i)+","+str(i)+"),("+str(i+1)+","+str(i+1)+")],"+str(k)+")")
      #print("path([("+str(i)+","+str(i)+"),("+str(i)+","+str(i-1)+")],"+str(k)+")")
      #print("path([("+str(i)+","+str(i)+"),("+str(i)+","+str(i+1)+")],"+str(k)+")")
