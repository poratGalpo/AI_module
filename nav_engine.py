import sys
from AI_engine import coordinate
class nav_interface():
    _navStack = None

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
    _navStack = []

    def receiver(self, coordinateInp):
        try:
            xLoc = coordinateInp.get_x()
            yLoc = coordinateInp.get_y()
            self._navStack.append(coordinateInp)
            print 'Got coordinate {0}\n'.format(coordinateInp)
        except:
            print sys.exc_info()
            return 0

        return 1

    def print_stack(self):
        for coordInstance in self._navStack:
            print coordInstance
            print  '\n'

if __name__ == '__main__':
    nav_stack = stub_navEngine()
    nav_stack.print_stack()
    nav_stack.receiver(coordinate(6,7))
    nav_stack.print_stack()


