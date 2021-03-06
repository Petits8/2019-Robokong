
import socket,os
from PIL import *
import pygame,sys
import pygame.camera
from pygame.locals import *

#Create server:
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("192.168.2.100",5000))
server.listen(5)

#Start Pygame
pygame.init()
pygame.camera.init()

screen = pygame.display.set_mode((320,240))

cam = pygame.camera.Camera("/dev/video0",(320,240),"RGB")
cam.start()

#Send data
while True:
    s,add = server.accept()
    print "Connected from",add
    image = cam.get_image()
    screen.blit(image,(0,0))
    data = cam.get_raw()
    s.sendall(data)
    pygame.display.update()

#Interupt
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
