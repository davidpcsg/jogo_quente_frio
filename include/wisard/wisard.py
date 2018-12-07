# A WISARD neural net implementation
# Copyright 2000, 2001 Iuri Wickert (iwickert@yahoo.com)
# Date: 20010606
# English Version
# License: GNU GPL

# This file contains the implementation of a 
# standard WISARD weightless neural network,
# specially suited to use with images. 

# Changes:
# - now uses all pixels of the image, with one
# smaller neuron.
# a faster (O(n)) select method.

import random 

# the wisard class
class wisard:


	# constructor 
	def __init__(self, n_cat, n_bit, im_width, im_height):

		# create the standard input mapping
		# for all discriminators
		self.input_map = self.mapping(n_bit, im_width, im_height)		
		self.n_ram = len(self.input_map)

		# creates the neural network list.
		# each neuron is a hash-table, and a
		# discriminator is a list of neurons
		self.net = []
		for i in xrange(n_cat):
			temp = []
			for j in xrange(self.n_ram):
				temp.append({})
			self.net.append(temp)	


	# returns a random element of a sequence,
	# consuming it. Heavily optimized version
	def select(self, seq):

		if seq != []:
			pos = random.randint(0, len(seq) - 1)
			elem = seq[pos]
			seq[pos] = seq[-1]
			del seq[-1]
			return elem
			
		else:
			return None
		

	# creates input-neurons mappings lists
	def mapping(self, bits, x, y):

		pixels = []
		for i in xrange(x):
			for j in xrange(y):
				pixels.append((i,j))

		# create as many full neuron mappings as 
		# possible with the existing pixels
		input_map = []
		n_tuples = x * y / bits
		rest = x * y % bits
		for i in xrange(n_tuples):
			temp = []
			for i in xrange(bits):
				# appends one of the remaining pixels
				temp.append( self.select(pixels) )

			input_map.append(temp)
		
		# tries to create a final neuron with the
		# remaining pixels
		if rest:
			temp = []
			for i in xrange(rest):
				temp.append( self.select(pixels) )
			input_map.append(temp)
			
		return input_map


	# decompose the input image bits into a structure
	# suitable to use as neurons inputs
	# supports image windowing
	def decompose(self, image, off_x, off_y):

		dec_output = []
		for neuron in self.input_map:

			# the tuple accumulator
			acc = int(0)

			# converts a set of binary pixels
			# to an integer to address one neuron
			for bit in neuron:

				acc = acc << 1
				x = bit[0] + off_x
				y = bit[1] + off_y
				if image[x][y]:
					acc = acc + 1

			dec_output.append(acc)

		return dec_output


	# simple total training of a discriminator
	def train(self, cat, image, off_x, off_y):
	
		in_tuples = self.decompose(image, off_x, off_y)
		
		# shortcut
		discr = self.net[cat]
		for neuron in xrange(self.n_ram):
			discr[neuron][in_tuples[neuron]] = 1
			

	# consult the entire net. returns each discriminator sum
	def consult(self, image, off_x, off_y):
	
		in_tuples = self.decompose(image, off_x, off_y)
		
		outputs = []
		for discr in self.net:
		
			sum = 0
			for i in xrange(self.n_ram):
				sum = sum + discr[i].get(in_tuples[i], 0)
				
			outputs.append(sum)
			
		return outputs
		
		
	# return a list with tuples containing the sums and its percentual
	def stats(self, image, off_x, off_y):
	
		temp = self.consult(image, off_x, off_y)
		output = map(lambda x, y=self.n_ram: (x, float(x)/y*100.0), temp)
		
		return output


	# appends a new discriminator to the neural net
	def new_discr(self):
	
		discr = []
		for i in xrange(self.n_ram):
			discr.append({})
		
		self.net.append(discr)
