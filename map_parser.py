#!/usr/bin/python

from PIL import Image as image
im = image.open("/home/igor/robot_movement/OlgaIgor_project/gmaps/test_corridor_90.pgm")

width, height = im.size
print width, height
hist = im.histogram()
print hist




im.close()
