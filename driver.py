import sys
import AI_engine
import socket

def main_call():
    engine = AI_engine.stub_engine()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    port = 6451
    s.bind(('',port))

    data = ''
    s.listen(5)
    c, addr = s.accept()
    while data != 'abort':
        data = c.recv(1024)
        print (c,addr,data)



if __name__ == '__main__':
    main_call()