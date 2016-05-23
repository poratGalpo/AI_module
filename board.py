import sys
from tile import tile
import threading
import random
from AI_engine import coordinate

class board():

    _lock = threading.RLock()
    _defaultX = 10
    _defaultY = 10
    _tile = tile()
    _instance = None
    car_default_initial_placement = [4, 4]
    __carPlacement = None
    __CAR_INITIAL_DIRECTION = None      # dictionary with the keys: x,y,val where val is the car's direction value
    rotational_pivot = None     #   in case that car's mechanics is not flexible





    def __init__(self,boardSize = None,carPlacement = None,random_values = 0,rotational_pivot = 1):
        """
        This method initializes a singleton of a board,a two dimensional array of various tiles and a car tile
        :param boardSize: dictionary of the form {'x': Int, 'y':Int}
        :param carPlacement: dictionary of the form {'x': Int, 'y':Int}
        :param random_values: tells the generator whether to randomize tile values
        :return:
        """
        xVal = None         #   Setted x value
        yVal = None         #   Setted y value
        self.rotational_pivot = rotational_pivot
        if board._instance == None:
            if boardSize == None or type(boardSize) != type({}):
                if random_values == 0:
                    board._instance =self.init_board(board._defaultX, board._defaultY)
                elif random_values == 1:
                    board._instance =self.init_board(board._defaultX, board._defaultY,random_values = 1)
                xVal = board._defaultX
                yVal =  board._defaultY
            else:
                try:
                    xVal = boardSize['x']
                    yVal =  boardSize['y']
                    if (xVal<= 0) or (yVal <= 0):
                        raise self.impossible_action_exception("illegal board size")
                    board._instance =self.init_board(xVal, yVal)

                except:
                    board._instance =self.init_board(board._defaultX, board._defaultY)

            if carPlacement == None or type(carPlacement) != type({}):
                flag = self.initial_car_placement(int((xVal-1)/2), int((yVal-1)/2))
                print flag
            else:
                try:
                    flag = self.initial_car_placement(carPlacement['x'],carPlacement['y'])
                except:
                    #   A more - permissive form that allows to receive illegal car placement
                    #   flag = self.initial_car_placement(self.car_default_initial_placement[0], self.car_default_initial_placement[1])
                    #   less permisive form - raises an excpetion
                    raise self.impossible_action_exception("Cannot place car outside of the board borders")
            if flag == False:
                raise self.impossible_action_exception("Error car's initial placement")
    def set_initial_direction(self):
        directions = self._tile.get_car_directions()
        self.__CAR_INITIAL_DIRECTION = directions[directions.keys()[0]]
    def __str__(self):

        string_model = ''
        for row in self._instance:
            for cell in row:
                string_model += str(cell) + ' '
            string_model = string_model[:-1] + '\n'

        return string_model
    def impossible_action_exception(self,message):
        """
        prints error message and exits
        """
        print ('board exception',message)
        sys.exit(-1)
    def isObstacle(self,which):
        """
        This method gets a direction (1-4 clockwise) and returns if the cell in that direction is an obstacle, i.e
        can't go through this place
        :param which: Int ranged 1-4
        :return: True if wall, False otherwise. raises exception on error
        """
        pass
        """
        if which<1 or which >4 :
            raise self.impossible_action_exception("Adjacent cell represenation is from 1 to 4")
        free_val = self._tile.get_FreeVal()
        unmapped_val = self._tile.get_UnmappedVal()
        car_location = self.get_car_placement()
        locX = car_location['x']
        locY = car_location['y']
        if which == 1:
            if locY-1 < 0:
                return True
            return (self.get_cell_val(locX,locY-1) != free_val) and (self.get_cell_val(locX,locY-1) != unmapped_val)
        elif which == 2:
            if locX+1 >= self.get_board_size()['x']:
                return True
            return (self.get_cell_val(locX+1,locY) != free_val) and (self.get_cell_val(locX+1,locY) != unmapped_val)
        elif which == 3:
            if locY+1 >= self.get_board_size()['y']:
                return True
            return (self.get_cell_val(locX,locY+1) != free_val) and (self.get_cell_val(locX,locY+1) != unmapped_val)
        elif which == 4:
            if locX-1 < 0:
                return True
            return (self.get_cell_val(locX-1,locY) != free_val) and (self.get_cell_val(locX-1,locY) != unmapped_val)
        """
    def get_board_size(self):
        """
        This method returns a dictionary with the size of the board
        :return: dictionary of the form {'x':Int, 'y':Int}
        """
        try:
            return {'y':len(self._instance) ,'x':len(self._instance[0])}
        except:
            return {'x':-1,'y':-1}
    def get_cell_val(self,locationX,locationY):
        """
        Given an X and Y placement within the board, returns the value of the given cell
        :param locationX:
        :param locationY:
        :return:    cell's value upon success, -1 upon ERROR
        """
        try:
            xLen = len(self._instance[0])
            yLen = len(self._instance)
        except:
            return -1

        if ((locationX < 0) or (locationX >= xLen)) or ((locationY < 0) or (locationY >= yLen)) :
            raise self.impossible_action_exception("Error while trying to get cell")

        else:
            return self._instance[locationY][locationX]
    def tile_list_generator(self,length,val):
        """
        This method creates tile of specific val and length
        :param length: length of the returned list
        :val: value to fill each cell within the list
        :return: list
        """
        result = []
        if (length <=0) or (val<0):
            return []
        else:
            for i in range(length):
                result.append(val)

        return result
    def get_xy_map(self,sizeX,sizeY):
        """
        This method is responsible for for cropping a map sized X*Y around the car
        In case there is not enough space around some side, it will fill it with unreachable placement
        :param sizeX: number of cells on the right/left of the car
        :param sizeY: number of cells on the front/back of the car
        :return: a two dimensional array where the car is in the middle of it, raise exception in case of an error
        """
        if (sizeX < 0) or (sizeY <0):
            return []

        result = []
        unreachable_val = self._tile.get_UnreachableVal()
        xBoard_size = len(self._instance[0])
        yBoard_size = len(self._instance)
        car_place_dict = self.get_car_placement()
        carX_placement = car_place_dict['x']
        carY_placement = car_place_dict['y']
        if carX_placement == -1 or carY_placement == -1:
            raise self.impossible_action_exception("illegal size arguments")

        #   checking if unreachable spaces are needed
        missing_space_left = self.tile_list_generator(((min(0,(carX_placement-sizeX))) * -1),unreachable_val)
        missing_space_right = self.tile_list_generator((max(0,carX_placement+sizeX - xBoard_size+1)),unreachable_val)
        """
        missing_space_front = self.tile_list_generator((min(0,carY_placement-sizeY) * -1),unreachable_val)
        missing_space_rear = self.tile_list_generator((max(0,carY_placement+sizeY-yBoard_size+1)),unreachable_val)
        """
        #   the board will be filled with rows of unreachable tiles if needed
        missing_rows_front = (min(0,carY_placement-sizeY) * -1)
        missing_rows_rear = (max(0,carY_placement+sizeY-yBoard_size+1))


        xIteration_from = max(0,carX_placement-sizeX)
        xIteration_to =  min(xBoard_size-1,carX_placement+sizeX)
        yIteration_from = max(0,carY_placement-sizeY)
        yIteration_to =  min(yBoard_size-1,carY_placement+sizeY)

        for i in range(yIteration_from,yIteration_to+1):
            original_piece = missing_space_left + self._instance[i][xIteration_from:xIteration_to+1] + missing_space_right
            result.append(original_piece)

        if missing_rows_front !=0 or missing_rows_rear != 0:
            unreachable_row = self.tile_list_generator(sizeX*2+1,unreachable_val)
            rear_completion = []
            front_completion = []
            for i in range(missing_rows_front):
                front_completion.append(unreachable_row)
            for i in range(missing_rows_rear):
                rear_completion.append(unreachable_row)
            result = front_completion + result + rear_completion

        tempBoard = board()
        tempBoard._instance = result
        return tempBoard
    def init_board(self, xVal, yVal,random_values=0):
        if random_values == 1:
            return [[(random.choice(self._tile.get_tile_mapping().values())) for i in range(xVal)] for j in range(yVal)]
        return [[self._tile.data for i in range(xVal)] for j in range(yVal)]
    def set_cell(self,locX,locY,val):
        """
        setting a cel positioned [x.y] to value val (val needs to be legal tile value)
        :param locX: cell row
        :param locY: cell column
        :param val: new cell's value
        :return: True upon success, exception if illegal location, False if value is not found
        """

        legal_values = self._tile.get_tile_mapping()
        for key in legal_values:
            if legal_values[key] == val:
                try:
                    self._instance[locY][locX] = val
                except:
                    raise self.impossible_action_exception("Error while trying to set cell")
                return True


        return False
    def get_car_placement(self):
        """
        This method is responsible for locating the car within the board,
        !!  returns the first instance it sees, if two cells with the same value exist it will return the upper most left one   !!
        :return: dictionary containing the x and y points {'x':Int,'y':Int} the key's values would be -1
                 if they cannot be found
        """

        return self.__carPlacement

        #   previous implementation
        carVal = self._tile.get_CarVal()
        for row in range(len(self._instance)) :
            for cell in range(len(self._instance[row])):
                if self._instance[row][cell] == carVal:
                    return {'x':cell,'y':row}

        return {'x':-1,'y':-1}
    def initial_car_placement(self,locX,locY):
        """
        Assuming car initial position
        If cell is free, placing the car upon the data board
        :param locX: the car's place within the x axis
        :param locY: the car's place within the y axis
        :return: True upon success, False otherwise
        """
        current_cell_value = self.get_cell_val(locX,locY)
        if (current_cell_value == -1):
            return False
        elif (current_cell_value == self._tile.get_FreeVal()):
            return False

        car_val = self._tile.get_CarVal()
        self.__carPlacement = {'x':locX,'y':locY,'val':car_val}
        return self.set_cell(locX,locY,car_val)

    def set_movement_cell(self,val,locX,locY):
        """
        In case of changing the car's direction, this method change the tile value
        :param val: tile value representing the new direction
        :return: True upon success, False if the value is illegal
        """
        if self._tile.isDirection_available(val):
            try:
                self._instance[locY][locX] = val
            except:
                raise self.impossible_action_exception("Error while trying to set cell")
            return True
        return False
    def move_LU(self):
        """
        This method moves to to the left in a diagonal matter
        relatively to the car's direction !!!
        :return: True upon success, False otherwise
        """
        location_data = self.__carPlacement
        xLoc = location_data['x']
        yLoc = location_data['y']
        current_direction = location_data['val']
        direction_dictionary = self._tile.get_directions_dict()

        if current_direction == direction_dictionary['LU']:
            #   turn left
            xNext = xLoc - 1
            yNext = yLoc
            directionNext = direction_dictionary['LC']
            pass
        elif current_direction == direction_dictionary['CU']:
            #   left upper corner
            xNext = xLoc - 1
            yNext = yLoc - 1
            directionNext = direction_dictionary['LU']
            pass

        elif current_direction == direction_dictionary['RU']:
            #   center upper
            xNext = xLoc
            yNext = yLoc - 1
            directionNext = direction_dictionary['CU']
            pass
        elif current_direction == direction_dictionary['RC']:
            #   right upper corner
            xNext = xLoc + 1
            yNext = yLoc - 1
            directionNext = direction_dictionary['RU']
            pass
        elif current_direction == direction_dictionary['RB']:
            #   right
            xNext = xLoc + 1
            yNext = yLoc
            directionNext = direction_dictionary['RC']
            pass
        elif current_direction == direction_dictionary['CB']:
            #   right bottom corner
            xNext = xLoc + 1
            yNext = yLoc + 1
            directionNext = direction_dictionary['RB']
            pass
        elif current_direction == direction_dictionary['LB']:
            #   rear
            xNext = xLoc
            yNext = yLoc + 1
            directionNext = direction_dictionary['CB']
            pass
        elif current_direction == direction_dictionary['LC']:
            #   left bottom corner
            xNext = xLoc - 1
            yNext = yLoc + 1
            directionNext = direction_dictionary['LB']
            pass
    def move_front_tile(self):
        """
        This method moves the car one tile ahead and updates the tile it has been to to MAPPED
        :return: new location if possible,otherwise -1

        !   Note : lock is needed in the tile setting area
        """
        current_location = self.get_car_placement()
        locX = current_location['x']
        locY = current_location['y']
        if (locX == -1 or locY == -1) or (locY-1< 0)  :
            return False
        else:
            mapped_val = self._tile.get_FreeVal()
            car_val = self._tile.get_CarVal()
            unmapped_val = self._tile.get_UnmappedVal()

            #   checking if the next step is reachable
            if self.get_cell_val(locX,locY-1) != mapped_val and self.get_cell_val(locX,locY-1) != unmapped_val:
                return False
            if self.replace_two_tile(locX,locX,locY-1,locY,car_val,mapped_val) == False:
                error_val = self._tile.get_ErrorVal()
                self.set_cell(locX,locY,car_val)
                self.set_cell(locX,locY-1,error_val)
                print 'Error - Could not place the car in location X:{0} Y:{1}.\nCar moved back to location X:{2} Y:{3}\n'.format(locX,locY-1,locX,locY)
                return False
        return self.get_car_placement()
    def move_left_tile(self):
        """
        This method moves the car one tile to the left
        :return: new location if possible,otherwise -1
        """
        current_location = self.get_car_placement()
        locX = current_location['x']
        locY = current_location['y']
        if (locX == -1 or locY == -1) or (locX-1< 0)  :
            return False
        else:
            mapped_val = self._tile.get_FreeVal()
            car_val = self._tile.get_CarVal()
            unmapped_val = self._tile.get_UnmappedVal()

            #   checking if the next step is reachable
            if self.get_cell_val(locX-1,locY) != mapped_val and self.get_cell_val(locX-1,locY) != unmapped_val:
                return False
            if (self._tile.isVal_legal(mapped_val) == False) or (self._tile.isVal_legal(car_val) == False):
                return False
            # (self.set_cell(locX,locY-1,car_val)==False ) or (self.set_cell(locX,locY,mapped_val)==False ) - had been replaced by an atomic method
            if self.replace_two_tile(locX-1,locX,locY,locY,car_val,mapped_val) == False:
                error_val = self._tile.get_ErrorVal()
                self.set_cell(locX,locY,car_val)
                self.set_cell(locX,locY-1,error_val)
                print 'Error - Could not place the car in location X:{0} Y:{1}.\nCar moved back to location X:{2} Y:{3}\n'.format(locX,locY-1,locX,locY)
                return False
        return self.get_car_placement()
    def move_right_tile(self):
        """
        This method moves the car one tile to the right
        :return: new location if possible,otherwise False
        """
        board_size = self.get_board_size()
        row_length = board_size['x']
        if row_length == -1:
            return False
        current_location = self.get_car_placement()
        locX = current_location['x']
        locY = current_location['y']
        if (locX == -1 or locY == -1) or (locX+1 >= row_length)  :
            return False
        else:
            mapped_val = self._tile.get_FreeVal()
            car_val = self._tile.get_CarVal()
            unmapped_val = self._tile.get_UnmappedVal()

            #   checking if the next step is reachable
            if self.get_cell_val(locX+1,locY) != mapped_val and self.get_cell_val(locX+1,locY) != unmapped_val:
                return False
            if (self._tile.isVal_legal(mapped_val) == False) or (self._tile.isVal_legal(car_val) == False):
                return False
            # (self.set_cell(locX,locY-1,car_val)==False ) or (self.set_cell(locX,locY,mapped_val)==False ) - had been replaced by an atomic method
            if self.replace_two_tile(locX+1,locX,locY,locY,car_val,mapped_val) == False:
                error_val = self._tile.get_ErrorVal()
                self.set_cell(locX,locY,car_val)
                self.set_cell(locX,locY-1,error_val)
                print 'Error - Could not place the car in location X:{0} Y:{1}.\nCar moved back to location X:{2} Y:{3}\n'.format(locX,locY-1,locX,locY)
                return False
        return self.get_car_placement()
    def move_back_tile(self):
        """
        This method moves the car one tile to the back
        :return: new location if possible,otherwise -1
        """
        board_size = self.get_board_size()
        column_length = board_size['y']
        if column_length == -1:
            return False
        current_location = self.get_car_placement()
        locX = current_location['x']
        locY = current_location['y']
        if (locX == -1 or locY == -1) or (locY+1 >= column_length)  :
            return False
        else:
            mapped_val = self._tile.get_FreeVal()
            car_val = self._tile.get_CarVal()
            unmapped_val = self._tile.get_UnmappedVal()

            #   checking if the next step is reachable
            if self.get_cell_val(locX,locY+1) != mapped_val and self.get_cell_val(locX,locY+1) != unmapped_val:
                return False
            if (self._tile.isVal_legal(mapped_val) == False) or (self._tile.isVal_legal(car_val) == False):
                return False
            # (self.set_cell(locX,locY-1,car_val)==False ) or (self.set_cell(locX,locY,mapped_val)==False ) - had been replaced by an atomic method
            if self.replace_two_tile(locX,locX,locY+1,locY,car_val,mapped_val) == False:
                error_val = self._tile.get_ErrorVal()
                self.set_cell(locX,locY,car_val)
                self.set_cell(locX,locY-1,error_val)
                print 'Error - Could not place the car in location X:{0} Y:{1}.\nCar moved back to location X:{2} Y:{3}\n'.format(locX,locY-1,locX,locY)
                return False
        return self.get_car_placement()
    def expand_board(self,addX,addY,placementX,placementY):
        pass
    def process_new_data(self,data):
        pass
    def replace_two_tile(self,locX1,locX2,locY1,locY2,val1,val2):
        """
        This method is designed to replace two tiles with a value without having to take a risk,
        one or more tile would be changed as a result of multi-tasking
        note that this method does not check if the vales are legal
        :param locX1:   first location
        :param locX2:   second location
        :param locY1:
        :param locY2:
        :param val1:    replace tile1_val to
        :param val2:    replace tile2_val to
        :return: True if successful, False otherwise
        """


        with self._lock:
            return (self.set_cell(locX1,locY1,val1)) and  (self.set_cell(locX2,locY2,val2))
    def randomize_walls(self,number=None):
        """
        This method creates random walls
        :return: None
        """
        if number == None:
            number = self._defaultX
        board_size = self.get_board_size()
        wall_val = self._tile.get_WallVal()
        for x in range(number):
            flag= False
            while flag==False:
                randX = random.randint(0,board_size['x']-1)
                randY = random.randint(0,board_size['y']-1)
                if self.__CAR_INITIAL_DIRECTION == None:
                    self.set_initial_direction()
                if self.get_cell_val(randX,randY) != self.__CAR_INITIAL_DIRECTION:
                    flag = self.set_cell(randX,randY,wall_val)
        return None
    def insert_row_front(self,data = None):
        """
        This method adds a row of tiles at the upper bound of the board
        :param data: contains pre-made data, if None the we will randomize one
        :return: True if successful, False otherwise
        """
        if data == None:
            data =[]
            for i in range(len(self._instance[0])):
                data.append(random.choice(self._tile.get_tile_mapping().values()))
        else:
            if len(data) != len(self._instance[0]):
                return False
        try:
            self._instance.insert(0,data)
        except:
            return False

        return True
    def insert_row_right(self,data = None):
        """
        This method adds a column of tiles at the right bound of the board
        :param data: contains pre-made data, if None the we will randomize one
        :return: True if successful, False otherwise
        """
        if data == None:
            data =[]
            for i in range(len(self._instance)):
                data.append(random.choice(self._tile.get_tile_mapping().values()))
        else:
            if len(data) != len(self._instance):
                return False
        try:
            for index in range (len(self._instance)):
                self._instance[index].append(data[index])
        except:
            return False

        return True
    def insert_row_back(self,data = None):
        """
        This method adds a row of tiles at the rear bound of the board
        :param data: contains pre-made data, if None the we will randomize one
        :return: True if successful, False otherwise
        """
        if data == None:
            data =[]
            for i in range(len(self._instance[0])):
                data.append(random.choice(self._tile.get_tile_mapping().values()))
        else:
            if len(data) != len(self._instance[0]):
                return False
        try:
            self._instance.append(data)
        except:
            return False

        return True
    def insert_row_right(self,data = None):
        """
        This method adds a column of tiles at the left bound of the board
        :param data: contains pre-made data, if None the we will randomize one
        :return: True if successful, False otherwise
        """
        if data == None:
            data =[]
            for i in range(len(self._instance)):
                data.append(random.choice(self._tile.get_tile_mapping().values()))
        else:
            if len(data) != len(self._instance):
                return False
        try:
            for index in range (len(self._instance)):
                self._instance[index].insert(0,data[index])
        except:
            return False

        return True
    def insert_multiple_rows(self,data,car_facing):
        """
        This method allows adding several blocks of data, depending on the car facing
        :param data:        contains pre_made data, should be list of lists [[],[]]
        :param car_facing:  symbols where we need to add the data
        :return: True upon success, False otherwise
        """
        pass
    def get_obstacles_coordinates_list(self):
        """
        for each tile valued as wall, return its location
        :return: list of Coordinates
        """
        wall_val = self._tile.get_WallVal()
        indexX = 0
        indexY = 0
        obstacles_list = []
        for indexY in range(0,len(self._instance)):
            for indexX in range(0,len(self._instance[0])):
                if self.get_cell_val(indexX,indexY) == wall_val:
                    new_co = coordinate(indexX,indexY)
                    obstacles_list.append(new_co)
        return obstacles_list
    def get_obstacles_map(self):

        raise self.impossible_action_exception('need to be implemented')
    def get_nearest_obstacle(self):
        raise self.impossible_action_exception('need to be implemented')
    def get_obstacles_locations(self):
        """
        :return: an array depicting the coordinates where obstacles are
        """
        raise self.impossible_action_exception('need to be implemented')
    def is_all_wall_ahead(self, direction):
        raise self.impossible_action_exception('need to be implemented')

    def get_carLocation(self):
        pass
if __name__ == '__main__':


    newBoard = board()
    print newBoard
    print newBoard.get_obstacles_coordinates_list()
    sys.exit()
    print '*********'
    newBoard.randomize_walls()
    print newBoard
    print '*********'
    newBoard.insert_row_right()
    print newBoard