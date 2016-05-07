import sys
class boardReceiverInterface():

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
        raise self.BR_NotImplemented()

    def get_board(self):
        """
        :return: returns available map
        """
        raise self.BR_NotImplemented()








