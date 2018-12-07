#!/usr/local/bin/python
# PyWISARD3
# copyright Iuri Wickert, 2000, 2001
# e-mail: iwickert@yahoo.com
# 20010413
# License: GNU GPL

from wisard import *
from string import *
from pyppm import *
import sys
import os.path


# the application class
class wiz:
    # init: receives the root widget
    def __init__(self):

        # general widget creation & variables
        #----------------------------------------------------------------
        # aqui coloca o numero de classes e  e o numero de bits do discrimandor
        self.n_classes = 2
        self.n_bits = 4
        self.train_class = 1
        # ----------------------------------------------------------------
        self.types = [("PPM", ".ppm")]
        self.net = self.x = self.y = None


    def train_sample(self):

        for i in range(1,3):
            self.train_class = i
            for j in range(1, 5):

                file = 'classe'+str(i)+'_treino/classe'+str(i)+'_treino_'+str(j)+'.ppm'
                image = load_image(file)
                x = len(image)
                y = len(image[0])

                #  check if the image has the same size of the last one
                if not self.x:
                    self.x = x
                    self.y = y
                elif self.x != x or self.y != y:
                    print("Image size inconsistent with the WISARD!\n")
                    return None

                # checks if the wisard was already instantiated
                if not self.net:
                    self.net = wisard(self.n_classes, self.n_bits, x, y)
                    print("Created WISARD net, input size=%dx%d.\n" % (x, y))
                    print("Each discriminator has %d RAM neurons.\n" % (self.net.n_ram))

                self.net.train(self.train_class - 1, image, 0, 0)

                # print feedback to user
                print("Class %d <- \"%s\" ." % (self.train_class, file))


    def rec_sample(self, file):
        image = load_image(file)
        x = len(image)
        y = len(image[0])

        #  check if the image has the same size of the last one
        if not self.x:
            self.x = x
            self.y = y
        elif self.x != x or self.y != y:
            print("Image size inconsistent with the WISARD!\n")
            return None

        # recognise the image and print a report
        output = self.net.stats(image, 0, 0)
        print("\nWISARD answer for \"%s\":" % file, "blue")
        print("Class   Sum  Recog.")
        for i in xrange(self.n_classes):
            print("%5d %5d %6.1f%%" % (i + 1, output[i][0], output[i][1]))


# wiz class end

# to run also as a script (pointless)
if __name__ == "__main__":
    a = wiz()
    file = 'classe1_treino/classe1_treino_1.ppm'
    wiz.train_sample(a)
    wiz.rec_sample(a,file)
