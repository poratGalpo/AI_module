import sys
import random
import math
from boardReceiver import *
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

    def calculate_optimal_coordinate(self, refreshBoard = True):
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

        board, direction = self._boardReceiver.get_board(refreshBoard=False)
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
    receiver = None
    _board = None
    _direction = None

    try:
        car_position = _board.getPosition()
    except:
        car_position = DEFAULT_CAR_POSITION

    def __init__(self):

        self.receiver = boardReceiverImpl()
        self._board, self._direction = self.receiver.get_board(refreshBoard=False)


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

        tile_instance = self._board.get_tile()      #Bug fix   -  self.receiver.get_board()[0].get_tile()  was removed
        new_assignment= {tile_instance.get_FreeVal():0.001, tile_instance.get_UnmappedVal():1, tile_instance.get_WallVal():0}
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

    def calculate_optimal_coordinate(self, refreshBoard = True):
        """
        This method scans the board object and by factors of distance and direction calculates
        the most optimal coordinate to be scanned afterwards
        Note: each optimal coordinate selected will be stored in a coordinate list
                but can be selected again as long as the tile does not change its value
        :return: Coordinate instance upon success, False otherwise
        """
        self._board, self._direction = self.receiver.get_board(refreshBoard = refreshBoard)
        if self._board.is_mapping_done():
            return coordinate(-100,-100)
        bestResult = -1
        bestCoordinate = None
        car_location = self._board.get_car_placement()
        carX = car_location['x']
        carY = car_location['y']
        board_size = self._board.get_board_size()
        if board_size['x'] == -1 or board_size['y'] == -1:
            print 'Error in board size'
            sys.exit(-1)
        for indexY in range(0, board_size['y']):
            for indexX in range(0, board_size['x']):
                try:
                    currVal = self._board.get_cell_val(indexX,indexY)
                except:
                    print indexY,indexX


                calc_result = self.calc_tile_simple_imp(self.calculate_distance(carX, indexX, carY, indexY), currVal)
                #self._board._instance[indexY][indexX] = calc_result
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

class engine_v2(engine_interface):

    DEFAULT_CAR_POSITION = coordinate(4,4)
    receiver = None
    _board = None
    _direction = None

    try:
        car_position = _board.getPosition()
    except:
        car_position = DEFAULT_CAR_POSITION

    def __init__(self):

        self.receiver = boardReceiverImpl()
        self._board, self._direction = self.receiver.get_board(refreshBoard=False)


    def calculate_distance(self,x1,x2,y1,y2):
        distance = math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
        rnd_distance = round(distance,3)
        return rnd_distance

    def calc_neighbors_factor(self,x_loc ,y_loc, factor=(1, 1)):
        """
        This method is the main improvement of the new engine,
        for every tile found legit, the method enhance the probability of selecting this tile
        depending on the tile's neighbors.
        :param x_loc: index of the tile's x value
        :param y_loc: index of the tile's y value
        :param factor: a set declaring how much neighbors in the x,y indexes
        :return:
        """
        free_val = self._board._tile.get_FreeVal()
        unmapped_val = self._board._tile.get_UnmappedVal()
        neighbor_counter = float(0)
        neighbor_map = {free_val: 0.01, unmapped_val: 0.1, 'obstacle': 0}
        obstacle_rating = neighbor_map['obstacle']
        x_factor = factor[0]
        y_factor = factor[1]
        location_dictionary = {'x':x_loc, 'y':y_loc}
        cropped_board = self._board.get_xy_map(x_factor, y_factor, premade_location=location_dictionary)
        reduced_map = cropped_board._instance
        cropped_size = cropped_board.get_board_size()
        if cropped_size['x'] == -1 or cropped_size['y'] == -1:
            print 'Error calculating reduced map'
            return neighbor_map['obstacle']

        for y in range(0,cropped_size['y']):
            for x in range(0,cropped_size['x']):
                try:
                    neighbor_counter += neighbor_map[reduced_map[y][x]]
                except:
                    neighbor_counter += obstacle_rating

        return neighbor_counter

    def calculate_direction_factor(self, coorX, coorY):
        """
        This method gives a some reward to each tile according to it's location
        and the car's direction
        :param coorX: tile coordination x value
        :param coorY: tile coordination y value
        :return: int
        """
        HIGH_PROB   = 1.1
        MED_PROB    = 1.02
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
            if carX-(carY - coorY) <= coorX <= carX+(carY - coorY)and  carY >= coorY:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['RU'] == self._direction:
            if coorY <= carY and coorX >= carX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['RS'] == self._direction:
            if carY-(carX - coorX) <= coorY <= carY+(carX - coorX)and carX >= coorX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['RB'] == self._direction:
            if coorY >= carY and coorX >= carX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['SB'] == self._direction:
            if carX-(coorY- carY) <= coorX <= carX+(coorY - carY)and coorY >= carY :
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['LB'] == self._direction:
            if coorY >= carY and coorX <= carX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['LS'] == self._direction:
            if carY-(carX - coorX) <= coorY <= carY+(carX - coorX)and carX >= coorX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))
        elif available_directions['LU'] == self._direction:
            if coorY <= carY and coorX <= carX:
                return HIGH_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))

        return REG_PROB*(1 + 1 / self.calculate_distance(carX,coorX,carY,coorY))

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

    def calculate_optimal_coordinate(self, refreshBoard = True):
        """
        This method scans the board object and by factors of distance and direction calculates
        the most optimal coordinate to be scanned afterwards
        Note: each optimal coordinate selected will be stored in a coordinate list
                but can be selected again as long as the tile does not change its value
        :return: Coordinate instance upon success, if there are no unmapped tiles left
                the coordinate will be <-1,-1>, False otherwise
        """
        self._board, self._direction = self.receiver.get_board(refreshBoard = refreshBoard)
        if self._board.is_mapping_done():
            return coordinate(-100,-100)
        bestResult = -1
        bestCoordinate = coordinate(-1,-1)
        #car_location = self._board.get_car_placement()
        #carX = car_location['x']
        #carY = car_location['y']
        free_val = self._board._tile.get_FreeVal()
        unmapped_val = self._board._tile.get_UnmappedVal()
        board_size = self._board.get_board_size()
        if board_size['x'] == -1 or board_size['y'] == -1:
            print 'Error in board size'
            sys.exit(-1)
        for indexY in range(0, board_size['y']):
            for indexX in range(0, board_size['x']):
                currVal = self._board.get_cell_val(indexX,indexY)
                if currVal != unmapped_val:
                    continue
                direction_grade = self.calculate_direction_factor(indexX, indexY)
                neighbors_grade = self.calc_neighbors_factor(indexX, indexY, (1, 1))
                calc_result = direction_grade * neighbors_grade
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
