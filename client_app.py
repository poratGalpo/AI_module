import socket
import sys
import ast
import time

CONF_FILE = 'conf'
SLEEP_ON_CONNECTION_REFUSED = 5

def load_configurations(fileName):
        """
        This method loads parameters within a conf file
        and creates a variable containing those configurations called driver_conf
        :param fileName: file full path
        :return: dictionary upon success, False otherwise
        """
        fileDes = open(fileName, 'r')
        raw_conf = fileDes.read()
        try:
            return ast.literal_eval(raw_conf)
        except:
            print sys.exc_info()[1]
            return False
        return True


def main_call():

    conf = load_configurations(CONF_FILE)
    if conf == False:
        return
    s = socket.socket()         # Create a socket object
    host = socket.gethostname() # Get local machine name
    port = conf['network']['port']                # Reserve a port for your service.
    flag = False

    while not flag:
        try:
            s.connect((host, port))
            flag = True
        except socket.error as e:
            print 'connection refused, waiting..'
            time.sleep(SLEEP_ON_CONNECTION_REFUSED)
            continue

        input = raw_input()

        while True:

            try:
                s.send(input)
                input = raw_input()
            except:
                break

        s.close


if __name__ == '__main__':
    main_call()

