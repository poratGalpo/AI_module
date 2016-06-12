import sys
import AI_engine
import socket
import ast
import nav_engine as nav
import time


class driver():
    """
    This class represents the driver instance,
    it will
    """

    ################################
    #### Variables #################
    ################################
    EXT_ERR = 1
    EXT_OK = 0
    CONF_FILE = 'conf'
    _AI_engine = None
    s = None
    _nav_engine = None
    data = ''
    driver_conf = None


    def __init__(self):
        if not self.load_configurations(self.CONF_FILE):
            raise self.driver_exception('Could not create configuration file\n')
        if not self.open_socket():

            raise self.driver_exception("Could not open a socket")
        self.driver_control()

    def load_configurations(self,fileName):
        """
        This method loads parameters within a conf file
        and creates a variable containing those configurations called driver_conf
        :param fileName: file full path
        :return: True upon success, False otherwise
        """
        fileDes = open(fileName, 'r')
        raw_conf = fileDes.read()
        try:
            self.driver_conf = ast.literal_eval(raw_conf)
        except:
            print sys.exc_info()[1]
            return False
        return True

    def driver_exception(self, message = None):
        if message is None:
            print "Driver Error"
        else:
            print "Driver Error - {0}".format(message)

        print sys.exc_info()
        sys.exit(self.EXT_ERR)

    def no_implementation(self,message = None):
        if message is None:
            print "Implementation missing\n"
        else:
            print "No implementation for function - {0}".format(message)

        print sys.exc_info()
        sys.exit(self.EXT_ERR)

    def start_process(self):
        self._AI_engine = AI_engine.engine_v1()
        self._nav_engine = nav.stub_navEngine(self._AI_engine)
        print('Process has started successfully')

    def driver_control(self):
        hadStarted = False
        self.s.listen(5)
        c, addr = self.s.accept()
        data = ''
        while data != 'stop':
            if data == 'start':
                if hadStarted:
                    print 'Process had already started'
                else:
                    hadStarted = True
                    self.start_process()
            elif data == 'pause':
                print 'Scanning process had paused\nType "resume" to continue scanning\n '
                while data != 'resume':
                    print 'waiting for resuming scanning process'
                    time.sleep(2)
                    data = c.recv(1024)
                print '\n\nResuming for scanning process'
            data = c.recv(1024)

        if not self.terminate_process():
            print 'Could not terminate properly:\n{0}\n'.format(sys.exc_info()[1])
            sys.exit(self.EXT_ERR)

        print "Process terminated successfully! \n"
        sys.exit(self.EXT_OK)

    def terminate_process(self):
        """
        This method terminates the whole process of navigation
        :return: True upon success, False otherwise
        """
        if not self._nav_engine.terminate_process():
            print "Could not return to initial location, termination continues\n"
        self.s.close()
        if self.driver_conf['general']['print_steps']:
            print "The following coordinated were chosen:\n"
            self.print_steps()
        return True

    def print_steps(self):
        """
        This method prints the steps taken through the whole scanning process
        :return: None
        """
        data = self._nav_engine.get_stack()
        for instance in data:
            try:
                print instance
            except:
                continue
            raise self.no_implementation("print_steps")

    def open_socket(self):
        """
        This method is responsible for opening a socket with a port number specified in the conf doc
        :return: True upon success, False otherwise
        """
        ip = self.driver_conf['network']['ip']
        port = self.driver_conf['network']['port']
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.bind((ip,port))
        except:
            print sys.exc_info()[1]
            return False

        return True


if __name__ == '__main__':
    test_driver = driver()