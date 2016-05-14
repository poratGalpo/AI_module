import socket               # Import socket module
import AI_engine
import io
def main_call():
    s = socket.socket()         # Create a socket object
    host = socket.gethostname() # Get local machine name
    port = 6451                # Reserve a port for your service.

    s.connect((host, port))


    input = raw_input()

    while True:

        try:
            s.send(input)
            input = raw_input()
        except:
            break

    #print s.recv(1024)
    s.close


if __name__ == '__main__':
    main_call()

