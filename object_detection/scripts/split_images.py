"""
This script splits the images into test/train folders given the image percentage of each folder.

E.g. 
>>> python split_images.py -d ../images/complete_dataset -o ../images --train 80
"""

import os
import shutil

import argparse
import random

def split_images(directory, output, train_percentage):
    random_images = random.sample(range(0, len(os.listdir(directory))), int(len(os.listdir(directory))*(train_percentage/100)))

    if not os.path.exists(output + '/train'):
        os.makedirs(output + '/train')
    if not os.path.exists(output + '/test'):
        os.makedirs(output + '/test')

    for i in range(len(os.listdir(directory))):
        if i in random_images:
            shutil.copy(directory + '/' + os.listdir(directory)[i], output + '/train')
        else:
            shutil.copy(directory + '/' + os.listdir(directory)[i], output + '/test')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Split images")
    parser.add_argument('-d', '--directory', type=str, required=True, help='Directory containing the images')
    parser.add_argument('-o', '--output', type=str, required=True, help='Output directory containing the train/test folder')
    parser.add_argument('--train', type=int, required=True, help='Train percentage')
    args = parser.parse_args()
    split_images(args.directory, args.output, args.train)
