import sys
import boardReceiver
import random
import math

class coordinate():

    _x = None
    _y = None

    def __init__(self, x, y):
        self._x = x
        self._y = y

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def set_x(self, x):
        self._x = x
        print 'Coordinate x was updated successfully'

    def set_y(self, y):
        self._y = y
        print 'Coordinate y was updated successfully'

    def compare_two_coordinates(self, C1, C2):
        """
        receives two coordinates and returns True if  the are equal
        :param C1:
        :param C2:
        :return: True if identical, False otherwise
        """
        if C1.get_x == C2.get_x and C1.get_y == C2.get_y:
            return True
        return False

    def __str__(self):
        return "<{0},{1}>".format(self._x, self._y)

class engine_interface():

    def AIE_NotImplemented(self, message):
        if message is None:
            print "AI engine - method not implemented"
        else:
            print "AI engine - method % not implemented" % message

        sys.exit(1)

    def get_next_coordinate(self):
        """
        For a given board, calculates the next step
        :return: (X,Y) where X and Y are legal values for indexes within the board
        """
        raise self.AIE_NotImplemented('get_next_coordinate')

    def calculate_optimal_coordinate(self):
        """
        This method uses a calculation library which regardless of the previous coordinates calculates the
        by portability the optimal coordinate for scanning
        :return: coordinate instance
        """
        raise self.AIE_NotImplemented('calculate_optimal_coordinate')
    def is_mapping_done(self):
        """
        checks whether the map is surrounded with obstacles
        :return: 1 if true, 0 otherwise
        """
        raise self.AIE_NotImplemented('is_mapping_done')


class stub_engine(engine_interface):

    coordinate = coordinate(0,0)
    steps_stack = []
    _boardReceiver = boardReceiver.stub_boardReceiver()

    def push_to_stack(self,C):
        self.steps_stack.append(C)

    def is_exists(self, C):

        for x in self.steps_stack:
            if coordinate.compare_two_coordinates(x,C) == True:
                return True
        return False

    def get_next_coordinate(self):

        board = self._boardReceiver.get_board()
        sizeX = len(board[0])
        sizeY = len(board)

        targetX = (random.randrange(sizeX))
        targetY = (random.randrange(sizeY))
        target_co = coordinate(targetX, targetY)
        while self.is_exists(target_co) == True:
            targetX = (random.randrange(sizeX))
            targetY = (random.randrange(sizeY))
            target_co = coordinate(targetX, targetY)

        self.push_to_stack(target_co)
        return target_co



class engine_v1(engine_interface):

    DEFAULT_CAR_POSITION = coordinate(4,4)
    receiver = boardReceiver.stub_boardReceiver()
    _board = receiver.get_board()

    try:
        car_position = _board.getPosition()
    except:
        car_position = DEFAULT_CAR_POSITION

    def calculate_distance(self,x1,x2,y1,y2):
        distance = math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
        rnd_distance = round(distance,3)
        return rnd_distance

    def calc_prob_factor(self, distance, value ):
        pass
    def calc_tile_simple_imp(self,distance,value):
        tile_instance = self.receiver.get_board().get_tile()
        new_assignment= {tile_instance.get_FreeVal():0.5, tile_instance.get_UnmappedVal():1, tile_instance.get_WallVal():0}
        if value not in new_assignment:
            raise self.AIE_NotImplemented('No assignment for value {0}'.format(value))
        else:
            return round(distance*new_assignment[value],3)

    def calculate_optimal_coordinate(self):
        bestResult = -1
        bestCoordinate = None
        car_location = self._board.get_carLocation()
        carX = car_location['x']
        carY = car_location['y']
        for indexY in range(0,len(boardReceiver)):
            for indexX in range(0,len(boardReceiver)):
                currVal = self._board[indexY][indexX]
                calc_result = self.calc_tile_simple_imp(self.calculate_distance(carX, indexX, carY, indexY), currVal)
                self._board[indexY][indexX] = calc_result
                if calc_result > bestResult:
                    bestResult = calc_result
                    bestCoordinate = coordinate(indexX, indexY)
        return bestCoordinate


if __name__ == '__main__':

    engn = stub_engine()
    print engn.get_next_coordinate()
