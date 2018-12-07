# fast load of PPM images
# Copyright 2000, 2001, Iuri Wickert (iwickert@yahoo.com)
# License: GNU GPL

from array import *
from string import *
import os

#	# extracts an image from a Tkinter PhotoImage 
#	# to a 2-d list (slow for big images)
#	def load_image(self, file):
#
#		#  load the image via Tkinter
#		i = PhotoImage(file=file)
#		w = i.width()
#		h = i.height()
#
#		# scans the image and copy the pixels
#		output = [None] * w
#		for x in range(w):
#			output[x] = [None] * h
#			for y in range(h):
#				output[x][y] = int(split(i.get(x,y)) [0])
#
#		return output
	
# loads a PPM image into a list of arrays
def load_image(arq):	
	
	w = h = 0

	# open it
	image = open(arq, 'r')

	# reads the whole file and "parses" it
	temp = image.readlines()
	# prune comments inside the file (except the image data)
	clear_lines = filter(lambda x: x[0] != "#", temp[:-1])
	# lets keep the image data!
	clear_lines.append(temp[-1])
	fields = []
	for line in clear_lines:
		fields.extend(split(line))
	#print (clear_lines)
	# error catching for PPM (?)
	if upper(fields[0]) != "P6":
		image.close()
		return None
	
	# image size
	w = int(fields[1])
	h = int(fields[2])
	
	# copy the image to the list, line by line
	data = fields[4]
	output = [None] * h
	for i in range(h):
		start = w * i
		end = start + w 
		output[i] = array('B', data[start : end])
	#print (output)
	image.close()
	return output

