"""
This script saves each topic in a bagfile as a csv.
    author: Gal Brandwine.
    script_origin: Nick Speal www.speal.ca

    Usage:




Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided
Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon 

"""

import argparse
import csv
import os  # for file management make directory
import shutil  # for file management, copy file
import sys
import time

import rosbag

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", default="./",
                help="path to rosbag folder.")
ap.add_argument("-o", "--output", default="./",
                help="path to output folder, and folder name (e.g ./records/2018-11-29/) "
                     "output csv files wil be asved within designated folder.")
args = vars(ap.parse_args())

# todo: read a bagfile, create a folder by its name and save all topics by their name's as a csv.

# verify correct input arguments: 1 or 2
# if len(sys.argv) > 2:
#     print("invalid number of arguments:   " + str(len(sys.argv)))
#     print("should be 2: 'bag2csv.py' and 'bagName'")
#     print("or just 1  : 'bag2csv.py'")
#     sys.exit(1)
#
# elif (len(sys.argv) == 2):
#     listOfBagFiles = [sys.argv[1]]
#     numberOfFiles = "1"
#     print("reading only 1 bagfile: " + str(listOfBagFiles[0]))
#
# elif (len(sys.argv) == 1):
#     listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]  # get list of only bag files in current dir.
#     numberOfFiles = str(len(listOfBagFiles))
#     print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
#     for f in listOfBagFiles:
#         print(f)
#     print("\n press ctrl+c in the next 10 seconds to cancel \n")
#     time.sleep(1)
# else:
#     print("bad argument(s): " + str(sys.argv))  # shouldnt really come up
#     sys.exit(1)

listOfBagFiles = [f for f in os.listdir(args["input"]) if
                  f[-4:] == ".bag"]  # get list of only bag files in current dir.
numberOfFiles = str(len(listOfBagFiles))
print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
for f in listOfBagFiles:
    print(f)

print("\n press ctrl+c in the next 5 seconds to cancel \n")
time.sleep(5)

count = 0
for bagFile in listOfBagFiles:
    count += 1
    print("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
    # access bag
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    # create a new directory
    folder = bagName.rstrip(".bag")
    try:  # else already exists
        os.makedirs("bag_record_" + folder)
    except:
        pass
    shutil.copyfile(bagName, folder + '/' + bagName)

    # get list of topics from the bag
    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    for topicName in listOfTopics:
        # Create a new CSV file for each topic
        filename = folder + '/' + topicName.replace('/', '_slash_') + '.csv'
        with open(topicName.replace('/', ''), 'w+') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=',')
            firstIteration = True  # allows header row
            for subtopic, msg, t in bag.read_messages(
                    topicName):  # for each instant in time that has data for topicName
                # parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                #	- put it in the form of a list of 2-element lists
                msgString = str(msg)
                msgList = msgString.split('\n')
                instantaneousListOfData = []
                for nameValuePair in msgList:
                    splitPair = nameValuePair.split(':')
                    for i in range(len(splitPair)):  # should be 0 to 1
                        splitPair[i] = splitPair[i].strip()
                    instantaneousListOfData.append(splitPair)
                # write the first row from the first element of each pair
                if firstIteration:  # header
                    headers = ["rosbagTimestamp"]  # first column header
                    for pair in instantaneousListOfData:
                        headers.append(pair[0])
                    filewriter.writerow(headers)
                    firstIteration = False
                # write the value from each pair to the file
                values = [str(t)]  # first column will have rosbag timestamp
                for pair in instantaneousListOfData:
                    if len(pair) > 1:
                        values.append(pair[1])
                filewriter.writerow(values)
    bag.close()
print("Done reading all " + numberOfFiles + " bag files.")
