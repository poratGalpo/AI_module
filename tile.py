class tile():



    def get_tile_mapping(self):
        """
        A method for checking the static variables values
        :return: a dictionary containing the static variables and their values
        """
        return {
            'FREE':self.__FREE,
            'UNMAPPED':self.UNMAPPED,
            'WALL':self.WALL,
            'FELLOW':self.FELLOW,
            'UNREACHABLE':self.UNREACHABLE,
            'ERROR':self.ERROR,
            'CAR':self.CAR,
            #'DIRECTIONS': self.get_directions_dict()
            }

    def set_tile(self,previous_mapping,current_mapping ):
        """
        This method sets a new value to some tile, number should be larger than 0 and not set to any other variable
        :param previous_mapping: represents the tile we would like to set
        :param current_mapping: the new mapping value
        :return: False upon failure, True if successful
        """
        current_mapping_dict = self.get_tile_mapping()
        isExist = False
        isAvailable = True
        var_key = None
        for key in current_mapping_dict:
            if current_mapping_dict[key] == previous_mapping:
                isExist = True
                var_key = key
            if current_mapping_dict[key] == current_mapping:
                if previous_mapping != current_mapping:
                    isAvailable = False
        if (isAvailable == True) and (isExist == True) and (var_key != None) :

            if var_key == 'FREE':
                self.__FREE = current_mapping
                return True
            elif var_key == 'UNMAPPED':
                self.UNMAPPED = current_mapping
                return True
            elif var_key == 'CAR':
                self.CAR = current_mapping
                return True
            elif var_key == 'WALL':
                self.WALL = current_mapping
                return True
            elif var_key == 'FELLOW':
                self.FELLOW = current_mapping
                return True
            elif var_key == 'ERROR':
                self.ERROR = current_mapping
                return True
            elif var_key == 'UNREACHABLE':
                self.UNREACHABLE = current_mapping
                return True
            else:
                return False
        else:
            #   If the needed conditions are not true
            return False

    def isVal_available(self,value):
        """
        Checks if a value for mapping is taken
        :return: True if any variable uses this value, False otherwise
        """
        current_vals = self.get_tile_mapping()
        for key in current_vals:
            if current_vals[key] == value:
                return True

        return False

    #following methods designed to retrieve values of tile modes

    def isVal_legal(self,val):
        """
        For val argument, returns true if the val is being used by some mode
        :param val: Int
        :return: True if exist, False otherwise
        """
        all_modes = self.get_tile_mapping()
        for mode in  all_modes:
            if all_modes[mode] == val:
                return True

        return False

    def get_FreeVal(self):
        return self.__FREE

    def get_UnmappedVal(self):
        return self.UNMAPPED

    def get_WallVal(self):
        return self.WALL

    def get_FellowVal(self):
        return self.FELLOW

    def get_ErrorVal(self):
        return self.ERROR

    def get_UnreachableVal(self):
        return self.UNREACHABLE

    def get_car_directions(self):
        return self._directions

    def get_CarVal(self):
        return self.CAR

    def isDirection_available(self,direction):
        available_directions = self.get_car_directions()
        for key in available_directions:
            if available_directions[key] == direction:
                return True
        return False

    def __str__(self):
        return str(self._defaultVal)

    def __init__(self):
        """
        Initializing tile instance with the default value
        Assuming car direction will be represented as a string depicting a float
        :return: None
        """
        self.__FREE      =  '0'
        self.UNMAPPED    =  '1'
        self.WALL        =  '3'
        self.FELLOW      =  '4'
        self.UNREACHABLE =  '5'
        self.ERROR       =  '6'
        self.CAR         =  '9'
        self._defaultVal = self.UNMAPPED
        self.data = self._defaultVal
        self._directions = {'SU':'A','RU':'B', 'RS':'C', 'RB':'D', 'SB':'E','LB':'F', 'LS':'G', 'LU':'H'}
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



if __name__ == '__main__':
    tile1 = tile()
    print tile1.get_tile_mapping()
    print tile1.set_tile('3','20')
    tile1.__FREE = 20
    print tile1.get_tile_mapping()