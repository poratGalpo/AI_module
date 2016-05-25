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
