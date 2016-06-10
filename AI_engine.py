import sys
import random
import math
from boardReceiver import stub_boardReceiver
from coordinate import coordinate

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

#    _boardReceiver = boardReceiver.stub_boardReceiver()

    def push_to_stack(self,C):
        self.steps_stack.append(C)

    def is_exists(self, C):

        for x in self.steps_stack:
            if coordinate.compare_two_coordinates(x,C) == True:
                return True
        return False

    def get_next_coordinate(self):

        board, direction = self._boardReceiver.get_board()
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
    receiver = stub_boardReceiver()
    _board, _direction = receiver.get_board()

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

    def calculate_direction_factor(self,coorX,coorY):
        """
        This method gives a some reward to each tile according to it's location
        and the car's direction
        :param coorX: tile coordination x value
        :param coorY: tile coordination y value
        :return: int
        """
        HIGH_PROB   = 1.3
        MED_PROB    = 1.1
        REG_PROB    = 1
        LOW_PROB    = 0
        car_location = self._board.get_car_placement()
        carX = car_location['x']
        carY = car_location['y']
        if carX == -1 or carY == -1:
            print 'AI_engine: Cannot find car location <{0},{1}>'.format(carX, carY)
            return 0
        """
        #   abbreviation map:
        {
            SU  - straight upper
            RU  - right upper
            RS  - right straight
            RB  - right bottom
            SB  - straight bottom
            LB  - left bottom
            LS  - left straight
            LU  - left upper
        }
        """
        available_directions = self._board._tile.get_car_directions()
        if available_directions['SU'] == self._direction:
            if coorX <= carX+(carY -coorY)and coorX >= carX-(carY -coorY) and carY >= coorY:
                return HIGH_PROB*(abs(carX-coorX))
        elif available_directions['RU'] == self._direction:
            if coorY <= carY and coorX >= carX :
                return HIGH_PROB*(abs(carX-coorX)+abs(carY-coorY))
        elif available_directions['RS'] == self._direction:
            if coorY <= carY+(carX -coorX )and coorY >= carY-(carX -coorX ) and carX >= coorX:
                return HIGH_PROB*(abs(carY-coorY))
        elif available_directions['RB'] == self._direction:
            if coorY >= carY and coorX >= carX:
                return HIGH_PROB*(abs(carX-coorX)+abs(carY-coorY))
        elif available_directions['SB'] == self._direction:
            if coorX <= carX+(coorY- carY)and coorX >= carX-(coorY- carY)and coorY >= carY :
                return HIGH_PROB*(abs(carX-coorX))
        elif available_directions['LB'] == self._direction:
            if coorY >= carY and coorX <= carX:
                return HIGH_PROB*(abs(carX-coorX)+abs(carY-coorY))
        elif available_directions['LS'] == self._direction:
            if coorY <= carY+(carX -coorX )and coorY >= carY-(carX -coorX ) and carX >= coorX:
                return HIGH_PROB*(abs(carY-coorY))
        elif available_directions['LU'] == self._direction:
            if coorY <= carY and coorX <= carX :
                return HIGH_PROB*(abs(carX-coorX)+abs(carY-coorY))
        pass

    def calc_tile_simple_imp(self,distance,value):
        tile_instance = self.receiver.get_board()[0].get_tile()
        new_assignment= {tile_instance.get_FreeVal():0.5, tile_instance.get_UnmappedVal():1, tile_instance.get_WallVal():0}
        if value not in new_assignment:
            new_assignment[value] = 0.01
#            raise self.AIE_NotImplemented('No assignment for value {0}'.format(value))
#        else:
        return round(distance*new_assignment[value],3)

    def get_initial_coordinate(self):
        """
        This method returns the initial coordinate
        :return:  coordinate instance, with the actual values, or <0,0> if the initial values does not exist
        """
        initial_location = self.receiver.car_initial_location
        if initial_location['x'] == -1 or initial_location['y'] == -1:
            return coordinate(0,0)
        else:
            return coordinate(initial_location['x'], initial_location['y'])

    def calculate_optimal_coordinate(self):
        """
        This method scans the board object and by factors of distance and direction calculates
        the most optimal coordinate to be scanned afterwards
        Note: each optimal coordinate selected will be stored in a coordinate list
                but can be selected again as long as the tile does not change its value
        :return: Coordinate instance upon success, False otherwise
        """
        self._board, self._direction = self.receiver.get_board()
        bestResult = -1
        bestCoordinate = None
        car_location = self._board.get_car_placement()
        carX = car_location['x']
        carY = car_location['y']
        for indexY in range(0, len(self._board._instance)):
            for indexX in range(0, len(self._board._instance[0])):
                currVal = self._board._instance[indexY][indexX]
                calc_result = self.calc_tile_simple_imp(self.calculate_distance(carX, indexX, carY, indexY), currVal)
                self._board._instance[indexY][indexX] = calc_result
                if calc_result > bestResult:
                    bestResult = calc_result
                    bestCoordinate = coordinate(indexX, indexY)
        return bestCoordinate


    def last_coordinate(self):
        #   Not sure if needed
        self.AIE_NotImplemented("last_coordinate")

    def terminate_process(self):
        #   Not sure if needed
        self.AIE_NotImplemented("terminate_process")

    def start_process(self):
        self.AIE_NotImplemented("start_process")

    def pause_process(self):
        self.AIE_NotImplemented("pause_process")

if __name__ == '__main__':

    engn = engine_v1()
    print engn.calculate_optimal_coordinate()
    engn.receiver.refresh_board()
    print engn.calculate_optimal_coordinate()
