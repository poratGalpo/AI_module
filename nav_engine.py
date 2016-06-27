import sys
import AI_engine
import coordinate
from implementation import *
from path_to_words import *


class nav_interface():
    _navStack = None
    _AI_engine = None
    def AIE_NotImplemented(self, message):
        if message is None:
            print "nav engine - method not implemented"
        else:
            print "nav engine - method % not implemented" % message

        sys.exit(1)

    def receiver(self, coordinate):


        """
        This method receives a legal coordinate and push i to the navigation stack
        :param coordinate: legal coordinate instance
        :return: 1 upon success, 0 otherwise
        """
        self.AIE_NotImplemented('receiver')

    def print_stack(self):
        """
        This method prints the navigation stack content
        :return: 1 upon success, 0 otherwise
        """

        #   Naive implementation    -   need to be implemented
        #   according to the navigation stack type

        print self._navStack


class stub_navEngine(nav_interface):
    EXT_Err = 1
    _navStack = []
    _AI_engine = None       #AI_engine.engine_v1()

    def __init__(self,engine):
        """
        We need to ensure the engine being sent to the navigation component is the one that is
        compatible to all methods define in engine_interface
        :rtype: None
        :param engine: an instance of a class derived from engine_interface
        :return:
        """

        if not issubclass(engine.__class__, AI_engine.engine_interface):
            print 'navigation stack error - illegal AI_engine\n'
            sys.exit(self.EXT_Err)
        else:
            self._AI_engine = engine


    def __str__(self):
        """
        In case of printing the navigation stack,
        prints out the entire list
        :return: None
        """
        self.print_stack()
        return None

    def receiver(self, coordinateInp):
        try:
            xLoc = coordinateInp.get_x()
            yLoc = coordinateInp.get_y()
            self.push_navStack()
            print 'Got coordinate {0}\n'.format(coordinateInp)
        except:
            print sys.exc_info()
            return 0

        return 1

    def push_navStack(self,coo):
        """
        This method performs push to the navigation stack
        :param coo: Coordinate instance
        :return: True upon success, False otherwise
        """
        try:
            self._navStack += [coo]
        except:
            return False
        return True

    def get_stack(self):
        """
        This method returns the navigation stack
        :return: list
        """
        try:
            return self._navStack
        except:
            return []

    def terminate_process(self):
        """
        This method is used for terminating the scanning process.
        Will Define the nav stack to be the last coordinate and shuts down  the AI engine
        :return: True upon success, False otherwise
        """
        try:
            print "Navigating to the last coordinate"
            self._navStack = [self._AI_engine.get_initial_coordinate()]
        except:
            return False
        return True

    def print_stack(self):
        for coordInstance in self._navStack:
            print coordInstance
            print  '\n'

    def clean_atts(self):
        """
        This method is designed for a 'single-point' cleanup
        after the scanning process has been terminated
        Most of it's actions is deleting instance
        :return: True upon successful cleanup, False otherwise
        """
        flag = True
        try:
            del self._AI_engine
        except:
            print "Could not delete AI engine"
            flag = False

        try:
            del self._navStack
        except:
            print "Could not delete navigation stack"
            flag = False

        return flag

class astar_navEngine(nav_interface):

    EXT_Err = 1
    _navStack = []
    _AI_engine = None       #AI_engine.engine_v1()



    def navigate(self,board, goal, direction,getGrid = False):


        mapped_grid = ''
        tile_instance = board.get_tile()
        bor = board.get_board_size()
        height = bor["x"]
        width = bor["y"]
        diagram = GridWithWeights(height, width)
        for i in range(width):
            for j in range(height):
                if (board._instance[i][j] == tile_instance.get_WallVal()):
                    diagram.walls.append((i, j))
        car_loc = board.get_car_placement()
        came_from, cost_so_far = a_star_search(diagram, (car_loc["x"], car_loc["y"]),
                                               (goal.get_x(), goal.get_y()))
        path1 = calcPath(cost_so_far, (goal.get_x(), goal.get_y()), (car_loc["x"], car_loc["y"]), [])
        if path1 is False:
            return False,False
        else:
            self._navStack.append(goal)
        if getGrid:
            mapped_grid = get_grid(diagram, width=1, number=cost_so_far, start=(car_loc["x"], car_loc["y"]),
                      goal=(goal.get_x(), goal.get_y()), path=path1)
        final_path = get_path(path1, direction)
        return mapped_grid,final_path

    def __str__(self):
        """
        In case of printing the navigation stack,
        prints out the entire list
        :return: None
        """
        self.print_stack()
        return None

    def receiver(self, coordinateInp):
        try:
            xLoc = coordinateInp.get_x()
            yLoc = coordinateInp.get_y()
            self.push_navStack()
            print 'Got coordinate {0}\n'.format(coordinateInp)
        except:
            print sys.exc_info()
            return 0

        return 1

    def push_navStack(self,coo):
        """
        This method performs push to the navigation stack
        :param coo: Coordinate instance
        :return: True upon success, False otherwise
        """
        try:
            self._navStack += [coo]
        except:
            return False
        return True

    def get_stack(self):
        """
        This method returns the navigation stack
        :return: list
        """
        try:
            return self._navStack
        except:
            return []

    def terminate_process(self):
        """
        This method is used for terminating the scanning process.
        Will Define the nav stack to be the last coordinate and shuts down  the AI engine
        :return: True upon success, False otherwise
        """
        try:
            print "Navigating to the last coordinate"
            self._navStack = [self._AI_engine.get_initial_coordinate()]
        except:
            return False
        return True

    def print_stack(self):
        for coordInstance in self._navStack:
            print coordInstance
            print  '\n'

    def clean_atts(self):
        """
        This method is designed for a 'single-point' cleanup
        after the scanning process has been terminated
        Most of it's actions is deleting instance
        :return: True upon successful cleanup, False otherwise
        """
        flag = True
        try:
            del self._AI_engine
        except:
            print "Could not delete AI engine"
            flag = False

        try:
            del self._navStack
        except:
            print "Could not delete navigation stack"
            flag = False

        return flag

if __name__ == '__main__':
    nav_stack = stub_navEngine()
    nav_stack.print_stack()
    nav_stack.receiver(coordinate(6,7))
    nav_stack.print_stack()


