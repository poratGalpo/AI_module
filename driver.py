import sys
import AI_engine
import socket
import ast
import nav_engine as nav
import time
import tile
from datetime import datetime


class timer():
    counter = None
    last_timing = None

    def __init__(self):
        self.timer = 0
        self.last_timing = 0

    def timer_handler(self,operation='start'):
        """
        timer method handles timing and sums it in the timer variable
        :param operation: str. indicating whether to start/stop/pause/resume
        :return: upon stop, returns the time in timer
        """
        if operation == 'start':
            self.last_timing = time.time()
            self.timer = 0
        elif operation == 'pause':
            current_time = time.time()
            self.timer += (current_time-self.last_timing)
            self.last_timing = current_time
        elif operation == 'resume':
            self.last_timing = time.time()
        elif operation == 'stop':
            current_time = time.time()
            self.timer += (current_time-self.last_timing)
            temp_result = self.timer
            self.last_timing = time.time()
            self.timer = 0
            return temp_result

    def get_timing(self):
        return self.timer_handler()

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
    driver_name = ''
    _AI_engine = None
    s = None
    _nav_engine = None
    data = ''
    driver_conf = None
    fileDes = None
    timer = timer()
    calculation_timer = timer()
    last_timing = None

    def __init__(self):

        if not self.load_configurations(self.CONF_FILE):
            raise self.driver_exception('Could not create configuration file\n')
        if not self.open_socket():

            raise self.driver_exception("Could not open a socket")
        self.driver_name = self.driver_conf['general']['name']
        self.handle_log_descriptor(operation='open',fileName=self.driver_conf['general']['log_file'],reWrite= True)
        self.driver_control()


    def send_to_client(self,message):
        print message

    def handle_log_descriptor(self,operation='close',fileName='default_log',reWrite=False):
        """
        This method opens a file descriptor for listing all the logs
        :param operation: str. depicting the operation of close/open the file
        :param fileName: log file name
        :param reWrite: states whether rewriting the log is needed
        :return: None
        """
        DEFAULT_LOG_FILE= 'default_log'
        if operation == 'close':
            try:
                self.fileDes.close()
            except:
                print 'Could not close log descriptor'
        elif operation == 'open':
            try:
                if reWrite:
                    self.fileDes = open(fileName,'w')
                else:
                    self.fileDes = open(fileName,'a')
            except:
                print 'Illegal log file name, taking the default location {0}'.format(DEFAULT_LOG_FILE)
                try:
                    self.fileDes = open(DEFAULT_LOG_FILE,'w')
                except:
                    print 'Could not open default log file, printing to the screen'
        return

    def write_to_log(self,message):
        timestamp = datetime.utcnow()
        text = '{0}\n\t{1}\n\n'.format(str(timestamp),message)
        if self.fileDes is None:
            print text
        else:
            try:
                self.fileDes.write(text)
            except:
                print text

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
        """
        This method initializes main components of the system
        :return: True upon success, False otherwise
        """
        try:
            self._AI_engine = AI_engine.engine_v1()
            self._nav_engine = nav.astar_navEngine()
            self.write_to_log('Process has started successfully')
        except:
            print sys.exc_info()[1]
            return False
        return True

    def driver_control(self):
        """
        This the main routine of the driver part,
        It's mainly waiting to the user's command whether to start,pause,stop
        Once process has started, its continuously picking the optimal coordinate, presents it to the user,
        waiting for the user to send 'ok' then picking another tile until the process has been completed/stopped
        Note: There is an option of printing the map at the user's console, it is defined in the conf file
        :return: None
        """

        start_time = time.time()
        self.s.listen(5)
        c, addr = self.s.accept()
        data = c.recv(1024)
        print data
        while data != 'start':
            self.send_to_client("Illegal command, please type 'start' to start mapping")
            data = c.recv(1024)
        self.timer.timer_handler('start')
        self.write_to_log('Mapping process had started')
        if not self.start_process():
            self.write_to_log('Initializing process has failed, closing program')
            sys.exit(self.EXT_ERR)

        self.calculation_timer.timer_handler('start')
        self.calculation_timer.timer_handler('pause')
        # Now starting to generate new optimal coordinates
        while data != 'stop' and data != 'home':
            if self.driver_conf['general']['print_board']:
                current_board = str(self._AI_engine._board)
                self.send_to_client(current_board)
            self.calculation_timer.timer_handler('resume')
            curr_optimal = self._AI_engine.calculate_optimal_coordinate(refreshBoard=True)
            self.calculation_timer.timer_handler('pause')
            # Sending the coordinate to the client
            self.send_to_client(str(curr_optimal))
            if curr_optimal.get_x() == -100 and curr_optimal.get_y() == -100:
                self.write_to_log("Mapping is done")
                break
            if self.driver_conf['general']['print_steps']:
                try:
                    print_navBoard = self.driver_conf['general']['print_navigation_board']
                except:
                    print_navBoard = False
                self.calculation_timer.timer_handler('resume')
                mapped_grid, final_path = self._nav_engine.navigate(self._AI_engine._board, curr_optimal,self._AI_engine._direction)
                self.calculation_timer.timer_handler('pause')
                if final_path is False:
                    self.send_to_client('Coordinate unreachable')
                elif print_navBoard:
                    self.send_to_client(mapped_grid)
                else:
                    self.send_to_client(final_path)

            data = c.recv(1024)
            while data != 'ok' and data != 'pali':
                if data == 'pause':
                    self.timer.timer_handler('pause')
                    self.write_to_log('Mapping process had paused')
                    print 'Mapping process had paused\nType "resume" to continue scanning\n '
                    while data != 'resume':
                        print 'waiting for resuming scanning process'
                        time.sleep(2)
                        data = c.recv(1024)
                    self.timer.timer_handler('resume')
                    print '\n\nResuming for scanning process'
                    self.write_to_log('Resuming for scanning process')
                elif data == 'stop':
                    # If the user wishes to stop in the middle of the mapping
                    break
                data = c.recv(1024)
            if data == 'pali' and self._AI_engine._board.is_mapping_done():
                new_msg = "Mapping is done !"
                self.send_to_client(new_msg)
                self.write_to_log(new_msg)
                data = 'stop'
                
        if not self.terminate_process():
            print 'Could not terminate properly:\n{0}\n'.format(sys.exc_info()[1])
            sys.exit(self.EXT_ERR)

        total_time = self.timer.timer_handler('stop')
        calculation_time = self.calculation_timer.timer_handler('stop')
        message = 'Whole process took {0} seconds, calculation lasted {1} seconds'.format(str(total_time),str(calculation_time))
        self.send_to_client(message)
        self.write_to_log("Process terminated successfully! \n")
        self.handle_log_descriptor(operation='close')
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
        return None

    def open_socket(self):
        """
        This method is responsible for opening a socket with a port number specified in the conf doc
        :return: True upon success, False otherwise
        """
        print ("open socket")
        ip = self.driver_conf['network']['ip']
        port = self.driver_conf['network']['port']
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.bind((ip,port))
        except:
            print sys.exc_info()[1]
            return False
        #self.s.send('kaka')
        #sys.exit()
        return True


if __name__ == '__main__':
    test_driver = driver()