#!/usr/bin/env python

import socket
import sys
import ast
import time


CONF_FILE = 'conf'
SLEEP_ON_CONNECTION_REFUSED = 5
WAITING_ON_SOCKET = 2
EXT_OK = 1
recieve_socket = None

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
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        # Create a socket object
    host = "localhost" # Get local machine name
    port = conf['network']['port']                # Reserve a port for your service.
    flag = False

    while not flag:
        try:
            time.sleep(WAITING_ON_SOCKET)
            s.connect((host, port))
            flag = True
        except socket.error as e:
            print 'connection refused, waiting..'
            time.sleep(SLEEP_ON_CONNECTION_REFUSED)
            continue

        print 'Connection has been established successfully\n\n'
        input = raw_input()

        while input!= 'stop':

            try:
                s.sendall(input)
                # Look for the response
                length = 0
                data = s.recv(1)
                while not data == '|' :
                    length = length*10 + int(data)
                    data = s.recv(1)
                data = ""
                amount_received = 0
                while amount_received < length:
                    data += s.recv(16)
                    amount_received = len(data)
                print >>sys.stderr, 'received "%s"' % data
                input = raw_input()
            except:
                print sys.exc_info()
                break
        s.sendall(input)
        print "Closing program, thank you"
        s.close()
        sys.exit(EXT_OK)

if __name__ == '__main__':
    main_call()

