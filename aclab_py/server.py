from socket import *

serverName = '127.0.0.1'
serverPort = 4420

while True:
    serverSocket = socket(AF_INET, SOCK_STREAM)
    serverSocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serverSocket.bind((serverName, serverPort))
    serverSocket.listen(1)
    print("waiting for client connecting...")
    connectionSocket, addr = serverSocket.accept()
    connectionSocket.setsockopt(SOL_SOCKET, SO_KEEPALIVE, 1)
    print(connectionSocket.getsockopt(SOL_SOCKET,SO_KEEPALIVE))
    print("...connected.")
    serverSocket.close()

    while True:
        try:
            sentence = connectionSocket.recv(4096).decode()
        except ConnectionResetError as e:
            print("Client connection closed")
            break
        if(len(sentence)==0):
            break 
        else:
            print("recv: "+str(sentence))

    connectionSocket.shutdown(SHUT_RDWR)
    connectionSocket.close()
    print("connection closed.")