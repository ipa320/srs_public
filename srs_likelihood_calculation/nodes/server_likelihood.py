#!/usr/bin/env python
#################################################################
# \note likelihood service
# \input(from DEM to learning) command and candidates
# \output(from learning to DEM) candidate and correspomding likelihood
#   Project name: srs learning service for choosing priority 
# \author
#   Tao Cao, email:itaocao@gmail.com
#
# \date Date of creation: Dec 2011
#################################################################

import roslib; roslib.load_manifest('srs_likelihood_calculation')

from srs_likelihood_calculation.srv import *
from std_msgs.msg import String
import rospy

import fileinput

historical_data_LW=[]
        
#using learning window to read historical data 
#this function will be updated 
#the learning_window_width will be decided based on the learning experience
#this function should only need the historical_data file name, and will search this file itself

def data_from_learning_window(learning_window_width,historical_full_path):
    #learning_window_width=learning_window_width
    line_offset = []
    historical_data_LW=[]
    
    num_lines=0 #the number of lines of the historical_data(how big the experience is)
    offset = 0 #the offset for a line

    fo=open(historical_full_path,'r')#open the historical_data
    
    #calculate the num_lines and the offset for every line
    for line in fo:
	line_offset.append(offset)
	offset += len(line)
        num_lines+=1
    #file.seek(0) # reset offset to the begining of the file
    #n+learning_window_width=num_lines 
    #the n is the line number, so n >=0, this means, num_lines>=learning_window_width, the program should make sure this is true
    if num_lines>=learning_window_width:
      #learning according the learning_window_width
      n=num_lines-learning_window_width
    else:
      n=0 #using the whole historical_data
    
    # Now, to skip to line n (with the first line being line 0), just do
    fo.seek(line_offset[n])   
    

    for line in fo:
        
	#remove the end-line character from the action sequence
	if line[-1] == '\n':
           line1=line[:-1]
        else:
           line1=line        
	historical_data_LW.append(line1)
	
    # Close opend file
    fo.close()
    return historical_data_LW #return the historical_data based on the learning_window_width

def calculate_likelihood(data_for_likelihood,candidate_list):
    #task_success_flag=[]#flag to mark the task is sucessful or not
    candidates_list=candidate_list.split()#store all the candidates into a list
    #set up a dict structure to save candidates and correspomding frenquency
    candidate_frenquence={}
    candidate_weight={}
    candidate_likelihood={}
    initial_weight=0.1#in this stage, the weight is from 0.1,0.2,..., 1, totally 10 weights
    #the initial frequence and weight is 0
    for candi in candidates_list:
      candidate_frenquence[candi]=0
      candidate_weight[candi]=0
      candidate_likelihood[candi]=0
    
    for action in data_for_likelihood:#data_for_likelihood is a list, with action sequence as item
      if action.find('place_on_tray'):
	#find the action marker, the task is sucessful
	#task_success_flag.append(1) this task_success_flag is not needed
	#find the nearest move(base,position) action to match the candidates
	location_flat=action.rfind('move(base,')# mark where is the location for grasp
	sub_actions=action[location_flat:]#sub action sequence begining with move action
	sub_actions_list=sub_actions.split()
	print "the location is in %s"%sub_actions_list[0]
	#check which candidate is the location
	#comparing the candidates in candidates_list with the current loation
	for candidate in candidates_list:#choosing candidates from dict candidate_frenquence
	  
	  if sub_actions_list[0].find(candidate)>0:#the candidate is the location, correspomding frequence and weight will be updated
	    print "the location is %s"%candidate
	    candidate_frenquence[candidate]+=1/10.0#10.0 is the totally number of historical_data
	    print "the candidate_frenquence for %s is %s" %(candidate,candidate_frenquence[candidate])
	    candidate_weight[candidate]+=initial_weight/5.5#5.5=(0.1+0.2+...+1)
	    print "the candidate_weight for %s is %s" %(candidate,candidate_weight[candidate])
	    candidate_likelihood[candidate]=(candidate_frenquence[candidate]+candidate_weight[candidate])/2.0
	    #candidate_likelihood[candidate]+=candidate_frenquence[candidate]+candidate_weight[candidate]
	    print "the candidate_likelihood for %s is %s" %(candidate,candidate_likelihood[candidate])
	    initial_weight+=0.1
	    
	  
      else:
	#we assumed all action sequences for a task is successful in this stage, this else here is to handle exceptions of receiving failed tasks
	#without the action marker,the task is failed
	#there will be no frenquecy and weight for any candidate
	#task_success.append(0)
	initial_weight+=0.1
    #
    #after above for loop, all the historical_data_LW had been used for calculating frenquence and weight
    #now use the frenquence and weight to calculate the likelihood
    #
    #likelihood=(weight+frenquence)/2  this also can be done in the for loop as shown above   
    print "candidate_frenquence is: %s" %candidate_frenquence
    print "candidate_weight is: %s" %candidate_weight
    print "candidate_likelihood is %s" %candidate_likelihood
    candidate_likelihood_list=candidate_likelihood.items()
    return str(candidate_likelihood_list)
 	

# callback function for receive consulting command and candidates
def handle_consulting(req):
    #req is the consulting information command+candidates from the DEM to Learning_service
    #both the command and candidates will be used for likelihood calculating based on the historical data of learning 
    print "consulting command is: %s "%req.command
    print "consulting candidates are: %s" %req.candidate
    #here we just read the historical_data from the txt file
    #the final data should come from a search based on the command (task classification) and candidates(for activity cluster)
    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    data_for_likelihood=data_from_learning_window(10,'historical_data.txt')
    print"the historical_data based on learning_window_width=10 is: "
    print data_for_likelihood
    print "the number of historical_data is %d"%len(data_for_likelihood)
    #calculating the likelihood
    #likelihoodresponse="likelihood for %s from %s is ??? "%(req.command,req.candidate)
    likelihoodresponse=calculate_likelihood(data_for_likelihood,req.candidate)
    return LikelihoodResponse(likelihoodresponse)

def callback_record_data(data):
    #rospy.loginfo(rospy.get_name()+"Action sequence is: %s",data.data)
    #print rospy.get_name()+" Action sequence is: ",data.data
    print "Action sequence is: ",data.data
    # Open a file
    fo = open("historical_data.txt", "ab")
    fo.write( data.data+"\n");

    # Close opend file
    fo.close()

def consulting_server():
    
    rospy.init_node('likelyhood_server', anonymous=True)
    #rospy.init_node('recorder', anonymous=True)
    #rospy.loginfo("node is initialized")
    rospy.Subscriber("historical_data", String, callback_record_data) #record the historical data 
    #rospy.init_node('likelyhood_server')
    s = rospy.Service('likelihood', Likelihood, handle_consulting) #provide service
    print "Ready to receive consulting."
    rospy.spin()

if __name__ == "__main__":
    consulting_server()
