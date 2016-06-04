import sys
import random
import tile
import board
import ast

class boardReceiverInterface():

    _tile = tile.tile()

    def BR_NotImplemented(self,message=None):
        """
        General exception for methods that are not implemented
        :param message: Relevant message
        :return: none
        """
        if message is None:
            print "Board Receiver - method not implemented"
        else:
            print "Board Receiver - method % not implemented" % message

        sys.exit(1)

    def receive_board(self):
        """
        This method receives a board from an outer sources and
        transforms it to 2D array
        :return: 2D array of strings
        """
        raise self.BR_NotImplemented('receive_board')

    def get_board(self):
        """
        :return: returns available map
        """
        raise self.BR_NotImplemented('get_board')

    def init_board(self, xVal, yVal):
        """
        Randomizing walls with random values according to a tile instance
        :param xVal: number of cells in an single array
        :param yVal: number of arrays
        :return:    2D array (yVal arrays) with random values
        """
        return [[(random.choice(self._tile.get_tile_mapping().values())) for i in range(xVal)] for j in range(yVal)]

    def convert_to_board_instance(self,data):
        raise self.BR_NotImplemented('convert to board instance')

    def refresh_board(self):
        raise self.BR_NotImplemented('refresh_board')

    def get_car_direction(self,data):
        raise self.BR_NotImplemented('convert to board instance')


class stub_boardReceiver(boardReceiverInterface):

    fileIndexes = ['0','1','2','3','4']
    X_default_size = 10
    Y_default_size = 10
    _board = None
    FILE_NAME = 'map'
    _self_index = 0
    def __init__(self):
        """

        :rtype: stub_boardReceiver
        """
        first_map = self.extract_map_from_file(self.fileIndexes[self._self_index])
        self._board = board.board(preMade_data=first_map)
        self._self_index += 1
        return


    def extract_map_from_file(self,index_num):
        """
        for a given index, concatenates FILENAME+index+.txt and returns the data as evaluated by ast
        :param fileName: string representation of the index
        :return: data upon success, False otherwise
        """
        current_fileName = self.FILE_NAME+index_num+'.txt'
        fileDes = open(current_fileName,'r')
        rawData = fileDes.read()
        fileDes.close()
        try:
            data = ast.literal_eval(rawData)
        except:
            print sys.exc_info()
            return False
        return data

    def receive_board(self):
        """
        Receives board for the first time ( second if init counts)
        and updates the board's var _instance var
        :return: True upon success, False otherwise
        """
        self._self_index = (self._self_index +1) % (len(self.fileIndexes))
        #   Done on order to make sure that after initialization the self_index is still in range


        map = self.extract_map_from_file(self.fileIndexes[self._self_index])
        if map == False:
            return False
        self._board.set_board(map)
        self._self_index = (self._self_index +1) % (len(self.fileIndexes))
        print '******************************************************************\n\n\n\n'
        return  True

    def get_board(self):
        return self._board
#        return self.init_board(xVal= self.X_default_size, yVal= self.Y_default_size)

    def refresh_board(self):
        """
        sets the board's var _instance var to be the next map
        :return: True upon success, False otherwise
        """
        map = self.extract_map_from_file(self.fileIndexes[self._self_index])
        if map == False:
            return False
        self._board.set_board(map)
        self._self_index = (self._self_index +1) % (len(self.fileIndexes))
        print '******************************************************************\n\n\n\n'
        return True

if __name__ == '__main__':

    receiver = stub_boardReceiver()
    receiver.receive_board()

    print receiver.get_board()
    receiver.refresh_board()
    print receiver.get_board()
    receiver.refresh_board()
    print receiver.get_board()
    receiver.refresh_board()