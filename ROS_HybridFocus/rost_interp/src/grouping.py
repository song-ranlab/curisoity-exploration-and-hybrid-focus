#!/usr/bin/env python

import numpy as np
import re
import os
import rospy
from std_msgs.msg import Float32

#ppx = perplexity

def Average(lst):
    avg = np.mean(lst) #Find mean of array (used for threshold)
    #var = np.std(lst) #Find std deviation (for threshold)
    
    return avg

def neighbours((x, y)):
        candidatesX = [(x - 1), (x + 1), (x), (x)]
        candidatesY = [(y), (y), (y - 1), (y + 1)]
        return [candidatesX, candidatesY]

def grouping(test_array, avg):
    
    var = np.std(test_array) #Find std deviation (for threshold)
    alpha = 2 #Deviating from mean by two std deviations to include data in (x + var to x + alpha*var)
    #print avg, "\n", var
    test_array = np.array(test_array)
    all_group = []
    group = [] 
    region = np.zeros((300))

    seen_x = []
    seen_y = []

    
    k = 0
    first = 0
    
    for i in range(len(test_array)):
        for j in range(len(test_array[0])):
            #Using first in case no variables were declared
            if first == 0:
                #Determine if the value of the current point > threshold
                if test_array[i,j] > (avg + (var)) and test_array[i,j] <= (avg + (alpha*var)):
                    first = 1 #counter to check if first iteration

                    group.append([j, i, (test_array[i,j])]) #variable to hold information of each member of a given group
                    region[k] = test_array[i,j] - (avg + var) #variable to track total volume of given group

                    #Determine neighbors to examine (cross pattern)
                    look_x = [(j - 1), (j + 1), (j), (j)]
                    look_y = [(i), (i), (i - 1), (i + 1)]

                    #Check variable to compare last group position
                    check_x = j
                    check_y = i
                    #add current position to list of seen 
                    seen_x.append(j)
                    seen_y.append(i)

                    #iterating through to look at neighbors
                    for c in range(len(look_x)):
                        #ensure look position is within bounds
                        if look_x[c] >= 0 and look_x[c] <= (len(test_array[0])-1) and look_y[c] >= 0 and look_y[c] <= (len(test_array)-1):
                            nx = 0 #counter reset for comparision of seen and look
                            #Compare loop of seen positions and current look position
                            for a in range(len(seen_x)):
                                if seen_x[a] == look_x[c] and seen_y[a] == look_y[c]:
                                    nx += 1
                            if nx >= 1:
                                continue
                            
                            #Add group member if criteria met
                            if test_array[look_y[c], look_x[c]] > (avg + (var)) and test_array[look_y[c],look_x[c]] <= (avg + (alpha*var)):
                                group.append([look_x[c], look_y[c], (test_array[look_y[c],look_x[c]])])
                                region[k] += test_array[look_y[c],look_x[c]] - (avg + var)
                                seen_x.append(look_x[c])
                                seen_y.append(look_y[c])   
                                check_x = look_x[c]
                                check_y = look_y[c]                             
            else:
                if test_array[i,j] > (avg + (var)) and test_array[i,j] <= (avg + (alpha*var)):
                    
                    nx = 0  #counter reset
                    #verify current position has not been seen              
                    for a in range(len(seen_x)):
                        if seen_x[a] == j and seen_y[a] == i:
                            nx += 1
                    if nx >= 1:
                        continue

                    #Check to see if current position is in same or new group
                    if (abs((j - check_x)) + abs(i - check_y)) > 1:
                        k = k + 1 #incrament region variable
                        all_group.append(group) #add group data to total list
                        group = [] #reset for new group   
                        

                    group.append([j, i, (test_array[i,j])])
                    region[k] += test_array[i,j] - (avg + var)
                    #Determine neighbors to examine (cross pattern)
                    look_x = [(j - 1), (j + 1), (j), (j)]
                    look_y = [(i), (i), (i - 1), (i + 1)]
                    #Check variable to compare last group position
                    check_x = j
                    check_y = i
                    #add current position to list of seen
                    seen_x.append(j)
                    seen_y.append(i)
                        
                    #iterating through to look at neighbors
                    for c in range(len(look_x)):
                        #ensure look position is within bounds
                        if look_x[c] >= 0 and look_x[c] <= (len(test_array[0])-1) and look_y[c] >= 0 and look_y[c] <= (len(test_array)-1):
                            nx = 0
                            #Compare loop of seen positions and current look position
                            for a in range(len(seen_x)):
                                if seen_x[a] == look_x[c] and seen_y[a] == look_y[c]:
                                    nx += 1
                            if nx >= 1:
                                continue

                            #Add group member if criteria met: within one variance of average
                            if test_array[look_y[c], look_x[c]] > (avg + (var)) and test_array[look_y[c],look_x[c]] <= (avg + (alpha*var)):
                                group.append([look_x[c], look_y[c], (test_array[look_y[c],look_x[c]])])
                                region[k] += test_array[look_y[c],look_x[c]] - (avg + var)
                                seen_x.append(look_x[c])
                                seen_y.append(look_y[c])   
                                check_x = look_x[c]
                                check_y = look_y[c]
            if i == (len(test_array)-1) and j == (len(test_array[0])-1):
                all_group.append(group) #if at end of array add current group to all group
                #print region

    #With a volume array and position information need to extract most perplexing position for navigation
    max_region = list(region).index(max(region)) #location of max region
    interm = np.asarray(all_group[max_region]) #variable to focus on points within max region
    #print interm
    a = np.argmax(interm[:,2]) #find the row of the maximum point
    max_point = interm[a,0] #retrive the x coordinate of that maximum point

    #Convert grid x to pixel x
    if max_point == 20:
        x_coord = (max_point*32) - (32/2)
    else: 
        x_coord = ((max_point + 1)*32 - (32/2))

    return x_coord



def PPX_Interp(): #Node for reading PPX file and outputing desired coordinate

    pub = rospy.Publisher('PPX_X_COORD', Float32, queue_size=10)
    rospy.init_node('PPX_Interp')
    #rate = rospy.Rate(500) #10 hz
    count = 0
    run_avg = 0

    while not rospy.is_shutdown():
        #Open up .json with PPX last_line, read only last line
        with open('/home/curran/perplexity.json', "r") as f:
            f.seek(-2, os.SEEK_END)
            while f.read(1) != b'\n':
                f.seek(-2, os.SEEK_CUR) 
            last_line = f.readline()

        #Seperate string into individual values
        last_line = re.split('[,]', last_line)
        #Convert string into 2D array of floats
        last_line = np.asarray(last_line)
        last_line = last_line.astype(float)
        last_line = np.reshape(last_line, (15,20))
        #Calculate a running average
        run_avg = (Average(last_line) + run_avg) / 2.0
        if count < 31:
            count += 1
            print count
                
        if count >= 30: #Wait for 30sec to compile ppx baseline
            #call funtion to get coordinate from data
            h = grouping(last_line, run_avg)
            #Publish value to ROS Msg
            pub.publish(h)

        rospy.sleep(1) 

#Begin MAIN for ROS Node

if __name__ == '__main__':
    try:
        PPX_Interp()
    except rospy.ROSInterruptException:
        pass