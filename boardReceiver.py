import sys
import random
import tile
import board

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

    def get_car_direction(self,data):
        raise self.BR_NotImplemented('convert to board instance')

class stub_boardReceiver(boardReceiverInterface):

    X_default_size = 10
    Y_default_size = 10
    _board = board.board(random_values=1)
    def receive_board(self):
        print "Receiving board of size {0}X{1}\n(default size)".format(self.X_default_size, self.Y_default_size)

    def get_board(self):
        return self._board
#        return self.init_board(xVal= self.X_default_size, yVal= self.Y_default_size)

if __name__ == '__main__':

    receiver = stub_boardReceiver()
    print receiver.get_board()
