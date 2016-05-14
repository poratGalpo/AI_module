import sys
import boardReceiver
import random

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


if __name__ == '__main__':

    engn = stub_engine()
    print engn.get_next_coordinate()
