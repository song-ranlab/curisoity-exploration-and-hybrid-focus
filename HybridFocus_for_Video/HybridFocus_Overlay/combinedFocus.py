#!/usr/bin/env python

from darknet import load_image, load_meta, load_net, detect
import cv2
import numpy as np
import re
import os

def Average(lst):
    avg = np.mean(lst) #Find mean of array (used for threshold)
    #var = np.std(lst) #Find std deviation (for threshold)
    
    return avg

def readImg(count, select):
    
    if select == 'obj':
        imgRead = "/home/curran/Desktop/Stingray/Vid2Img/thumb%04d.jpg" % count
    elif select == 'ppx':
        imgRead = "/home/curran/Desktop/Stingray/obj_img/objImage%04d.jpeg" % count
    return imgRead

def doOverlap(Ax1, Ax2, Ay1, Ay2, Bx1, Bx2, By1, By2): 
      
    # X and Y coordinate check 
    if (Ax1 < Bx2 and Ax2 > Bx1 and Ay1 < By2 and Ay2 > By1):
        return True
    else: 
        return False

def obj_detect(imgRead,net, meta, count):

    #Load YOLO Model
    #net = load_net("/home/curran/Documents/yolo_ws/Darknet_DL/darknet/cfg/sting.cfg", "/home/curran/Documents/yolo_ws/Darknet_DL/darknet/backup/sting_final.weights", 0)
    #Import class info
    #meta = load_meta("/home/curran/Documents/yolo_ws/Darknet_DL/darknet/cfg/obj.data")
    #Find any detected objects
    results = detect(net, meta, imgRead)
    f = open("/home/curran/Desktop/Stingray/boundingBoxes/box%04d.txt" % count, "w+")
    check = 0
    #Use OpenCV to print box onto image
    img = cv2.imread(imgRead, cv2.IMREAD_COLOR)
    #Loop through to print all detected objects
    for cat, score, bounds in results:
        #check for file output
        check = 1
        x, y, w, h = bounds
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), thickness=3)
        cv2.putText(img,str(cat.decode("utf-8")),(int(x - w / 2),int((y - h / 2)-5)),cv2.FONT_HERSHEY_TRIPLEX,.6,(52,177,235), thickness=1)
        #write x,y of top left corner, and x,y of bottom right corner
        f.write(str(x) + "," + str(y) + "," + str(w) + "," + str(h) + "\n")
    #cv2.imshow("output", img)
    
    #write 0 to file to represent no object found
    if check == 0:
        f.write("0,0,0,0")
        #x, y, w, h = 0
    
    #Save Image to directory
    cv2.imwrite('/home/curran/Desktop/Stingray/obj_img/objImage%04d.jpeg' % count, img)
    
    f.close()
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #return(x,y,w,h)
     

def grouping(test_array, avg, count):
    
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
                        if i == (len(test_array)-1) and j == (len(test_array[0])-1):
                            all_group.append(group) #if at end of array add current group to all group
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
                                if i == (len(test_array)-1) and j == (len(test_array[0])-1):
                                    all_group.append(group) #if at end of array add current group to all group
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
    #max_region = list(region).index(max(region)) #location of max region
    
    #Object Detection Setup
    #Read in object box
    f = open('/home/curran/Desktop/Stingray/boundingBoxes/box%04d.txt' % count, 'r')
    bbox = f.readline()
    bbox = re.split('[,]', bbox)
    bbox = np.asarray(bbox)
    bbox = bbox.astype(float)

    #Define object box
    Lx2 = int(bbox[0] - bbox[2] / 2)
    Ly2= int(bbox[1] - bbox[3] / 2)
    Rx2 = int(bbox[0] + bbox[2] / 2)
    Ry2 = int(bbox[1] + bbox[3] / 2)

    #Loop to compare ppx point to object detector
    if len(all_group) == 1:
        max_point_x = all_group[0][0][0]
        max_point_y = all_group[0][0][1]

        #Convert grid x,y to pixel x,y
        if max_point_x == 20:
            x_coord = (max_point_x*32) - (32/2)
        else: 
            x_coord = ((max_point_x + 1)*32 - (32/2))

        if max_point_y == 15:
            y_coord = (max_point_y*32) - (32/2)
        else:
            y_coord = (max_point_y + 1)*32 - (32/2)
        
        #Compare focus point to detected objects
        #Define focus box
        Lx = int(x_coord - 96 / 2)
        Ly = int(y_coord - 96 / 2)
        Rx = int(x_coord + 96 / 2)
        Ry = int(y_coord + 96 / 2)
        
        #Check for object not recoginzed
        if all(bbox) == False:
            return Lx, Ly, Rx, Ry, all_group
    else:
        for i in range(1, len(all_group)):
            
            max_region = np.argsort(region)
                
            interm = np.asarray(all_group[max_region[len(max_region)-i]]) #variable to focus on points within max region
            
            a = np.argmax(interm[:,2]) #find the row of the maximum point
            
            max_point_x = interm[a,0] #retrive the x coordinate of that maximum point
            max_point_y = interm[a,1] #y coordinate

            #Convert grid x,y to pixel x,y
            if max_point_x == 20:
                x_coord = (max_point_x*32) - (32/2)
            else: 
                x_coord = ((max_point_x + 1)*32 - (32/2))

            if max_point_y == 15:
                y_coord = (max_point_y*32) - (32/2)
            else:
                y_coord = (max_point_y + 1)*32 - (32/2)
            
            #Compare focus point to detected objects
            #Define focus box
            Lx = int(x_coord - 96 / 2)
            Ly = int(y_coord - 96 / 2)
            Rx = int(x_coord + 96 / 2)
            Ry = int(y_coord + 96 / 2)
            
            #Check for object not recoginzed
            if all(bbox) == False:
                return Lx, Ly, Rx, Ry, all_group
            
            #Compare Boxes
            intersect = doOverlap(Lx, Rx, Ly, Ry, Lx2, Rx2, Ly2, Ry2)

            #In no intersection output focus
            if intersect == False:
                f.close()
                print "We made it"
                return Lx, Ly, Rx, Ry, all_group
                
            #If no focus box found output 0
            if i == (len(all_group) - 1):
                f.close()
                print "almost there"
                return 0, 0, 0, 0, all_group

def ppx_focus_overlay(data,imgRead, count, Lx, Ly, Rx, Ry):
    img = cv2.imread(imgRead, cv2.IMREAD_COLOR)
    overlay = img.copy()
    output = img.copy()

    for i in range(len(data)):
        for j in range(len(data[i])):
            x = data[i][j][0]
            y = data[i][j][1]

            if x != 0:
                xmin = (x*(650/20))
            else:
                xmin = 0

            if y != 0:
                ymin = (y*(480/15))
            else:
                ymin = 0

            if xmin == 0:
                xmax = xmin + ((640/20))
            else:
                xmax = xmin + ((640/20))
            
            if ymin == 0:
                ymax = ymin + ((480/15))
            else:
                ymax = ymin + ((480/15))

            cv2.rectangle(overlay,(xmin,ymin),(xmax,ymax),(0,0,255),-1)
    
    cv2.addWeighted(overlay, .4, output, .6, 0, output)
    cv2.rectangle(output, (Lx,Ly), (Rx,Ry), (58,235,52), thickness=4)        
    cv2.putText(output,"Focus",(int(Lx),int((Ly)-5)),cv2.FONT_HERSHEY_TRIPLEX,.6,(0,0,0), thickness=1)
    cv2.imwrite('/home/curran/Desktop/Stingray/ppx_img/ppxImage%04d.jpeg' % count, output)
    #cv2.imshow('image',output)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


if __name__ == "__main__":

    #Initial setup:
    #Load YOLO Model
    net = load_net("/home/curran/Documents/yolo_ws/Darknet_DL/darknet/cfg/sting.cfg", "/home/curran/Documents/yolo_ws/Darknet_DL/darknet/backup/sting_final.weights", 0)
    #Import class info
    meta = load_meta("/home/curran/Documents/yolo_ws/Darknet_DL/darknet/cfg/obj.data")
    #Load PPX data
    ppx_data = np.genfromtxt('/home/curran/Desktop/Stingray/BottomUpPPX/video_ppx.json', delimiter=",")
    #Initialize counter
    count = 0
    #avg found from taking data from ~30sec run on very first image in video (data in )
    run_avg = 359.529
    
    for i in range(466):
        count += 1
        imgRead = readImg(count,'obj')
        obj_detect(imgRead, net, meta, count)
        imgRead = readImg(count,'ppx')
        ppx_data_feed = np.reshape(ppx_data[count], (15,20))
        run_avg = (Average(ppx_data_feed) + run_avg) / 2.0
        Lx, Ly, Rx, Ry, group = grouping(ppx_data_feed, run_avg, count)

        if count == 277:
            breakTime = 1

        if count == 1:
            preLx, preLy, preRx, preRy = Lx, Ly, Rx, Ry
            ppx_focus_overlay(group, imgRead, count, Lx, Ly, Rx, Ry)
        elif count % 25 == 0:        
            preLx, preLy, preRx, preRy = Lx, Ly, Rx, Ry
            ppx_focus_overlay(group, imgRead, count, preLx, preLy, preRx, preRy)
        else: 
            ppx_focus_overlay(group, imgRead, count, preLx, preLy, preRx, preRy)

        

        
        
      