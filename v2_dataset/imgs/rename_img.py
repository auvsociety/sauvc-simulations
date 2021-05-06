#!/usr/bin/python

"""
This script renames the image files in the format framexxxx.jpg from the dataset into the format xxxx.jpg 
"""
import os

dirname = os.getcwd()+"/simpleRun_frnt"		# enter name of directory in which the images are present

if os.path.isdir(dirname):
	for _, filename in enumerate(os.listdir(dirname)):
		os.rename(dirname + "/" + filename, dirname + "/" + filename[5:])