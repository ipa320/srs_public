#!/usr/bin/env python
#################################################################
# \note historical_data record subscriber
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

import actionlib

from srs_decision_making.msg import *

import fileinput

import json # or `import simplejson as json` if on Python < 2.6

#randomly generate the location

from random import choice

foo = ['table2', 'table1','fridge']

historical_data_LW=[]

#global action_sequences_global

action_sequences_global=[]# action sequence for taks instance

task_flag=""
        
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
    return historical_data_LW #return the historical_data list based on the learning_window_width

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
    
    for action in data_for_likelihood:#data_for_likelihood is a list, with action sequence as items
      # action in data_for_likelihood list is a string, which has the find method
      if action.find('place_on_tray'):
	#find the action marker, the task is sucessful
	#task_success_flag.append(1) this task_success_flag is not needed
	#find the nearest move(base,position) action to match the candidates
	location_flag=action.rfind('move(base,')# mark where is the location for grasp
	sub_actions=action[location_flag:]#sub action sequence begining with move action
	sub_actions_list=sub_actions.split(', ')#the delimiter is', ', this delimiter will split action sequence into action list
	print "the location is in %s"%sub_actions_list[0]
	#check which candidate is the location
	#comparing the candidates in candidates_list with the current loation
   
	for candidate in candidates_list:#choosing candidates from dict candidate_frenquence
	  
	  if sub_actions_list[0].find(','+candidate+')')>0:#the candidate is the location, correspomding frequence and weight will be updated
	    #sub_actions_list[0].find(','+candidate+')') make sure the candidate can be exactly matched
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
    #command used for task classification
    #cadidates used for activity cluster
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


#definition of the find_ralation function

def find_ralation(actions,results,objects):
    
    #define the on relationship
    on_something={}
    on_something['move']='3'
    on_something['detect']='3'
    on_something['grasp']='3'
    on_something['place_on_tray']='3'
    
    #define the in relationship
    in_something={}
    in_something['move']='3'
    in_something['open']='3'
    in_something['detect']='3'
    in_something['grasp']='3'    
    in_something['place_on_tray']='3'
    in_something['close']='3'
    
    action_list=actions.split()
    print action_list
    result_list=results.split()
    print result_list
    # build up the action_result dictionary to use the results of actions
    actions_result={}
    for action in action_list:
      actions_result[action]=result_list[int(action_list.index(action))]
    #print 'actions_result is: ',actions_result.items()
    #return str(actions_result)
    #print 'actions_result.keys()',actions_result.keys()
    
    for action in action_list:
      #for relationship detect action is the marker of the relationship
      print action
      
      if action.find('detect')>=0:
	  
	 print 'detect action is found!'
	  #find the action mark
	  #then check the detect action successful or not
	 if actions_result[action]=='3':
	    #detect action is successful
	    #then find the open action
	    print 'detect action is successful!'
	    if actions.rfind('open')>=0:
	      for action1 in action_list:
	       if action1.find('open')>=0:
		print 'is open'
		#find the open action
		#then check the open is successful
		if actions_result[action1]=='3':
		  return 'in_something'
		else:
		  print 'unknown relationship'
		  return 'unknow relationship'
		  
	    elif actions.rfind('move')>=0:
	       for action1 in action_list:
		 if action1.find('move')>=0:
		  print 'is move'
		  if actions_result[action1]=='3':
		      return 'on_something'
		  else:
		      print 'unknown relationship'
		      return 'unknow relationship'
	    else:
	      print 'unknown relationship'
	      return 'unknow relationship'
	 else:
	   print 'detect action is failed, can not detect object!'
	   return 'can not detect object!'	
   

# callback function for receive consulting for ontology expanding
def handle_consulting2(req):
    #req is the consulting information command+candidates from the DEM to Learning_service
    #both the command and candidates will be used for likelihood calculating based on the historical data of learning
    #command used for task classification
    #cadidates used for activity cluster
    print "consulting command is: %s "%req.command
    print "consulting actions are: %s" %req.actions
    print "consulting actions results are: %s" %req.results
    if req.objects!="":
      print "consulting objects are: %s" %req.objects
    #else:
      #find objects from the commands
    
    
    ontologyresponse=find_ralation(req.actions,req.results,req.objects)
    #ontologyresponse="ontology for %s  is ??? "%req.objects
    
    return OntologyResponse(ontologyresponse)

def callback_record_data(data):
  
    #rospy.loginfo(rospy.get_name()+"Action sequence is: %s",data.data)
    #print rospy.get_name()+" Action sequence is: ",data.data
    print "task instance result is:",data.result
    global action_sequences_global
    # Open a file
    #action_sequences=action_sequences_global
    if data.result==' return_value: 3':#task successfully executed return_value: 3
      print "Task successfully finished\n"
      task_action_sequence=','.join(action_sequences_global)
      fo = open("dynamic_historical_data.txt", "ab")
      fo.write( task_action_sequence +"\n");
      action_sequences_global=[]

      # Close opend file
      fo.close()
    

def historical_data_recorder(data): #publish the data
    #print "Json feedback is: ",data
    global action_sequences_global
    
    print "action_sequences_global is %s" %action_sequences_global
    
    feedback_temp=str(data)
    
    location_flag_jsonfeedback=feedback_temp.find('json_feedback:')# mark where is the location for grasp
    sub_feedback_temp=feedback_temp[location_flag_jsonfeedback+len('json_feedback:'):]#sub action sequence begining with move action
    if sub_feedback_temp.find('name')>0:
      print "sub_feedback_temp is: %s" %sub_feedback_temp      
      
      feedback_dict=json.loads(sub_feedback_temp) #type of feedback_dict: <type 'unicode'>
      
      
      
      #print "feedback_dict is: %s" %feedback_dict
      
      #print "type of feedback_dict: %s" %type(feedback_dict)
      
      feedback_dict1=json.loads(feedback_dict)#type of feedback_dict1: <type 'dict'>
      
      #print "type of feedback_dict1: %s" %type(feedback_dict1)
      
      actions_doing=feedback_dict1["current_action"]
      
      task_info=feedback_dict1["task"]
      
      task_flag_temp=task_info["task_id"]
            
	
      if actions_doing["state"]=='succeeded': # only record the successful actions
      
	  actionis=actions_doing["name"]#get the action
	  action_with_para_li=[actionis]
	  ## get the action target
	  
	  #action_with_para_li.append(actions_doing["target"])
	  action_target=actions_doing["target_object"]
	  
	  action_with_para_li.append(action_target)
	  
	  action_with_para_str=' '.join(action_with_para_li)
	
	  print "curret action is: %s" %action_with_para_str
	  
	  ### this action_sequences is used as a stack for storing action sequence of a task instance
	  ## the following two lines update action_sequences
	  
	  if  action_sequences_global==[]:#this action_sequences is null
	    action_sequences_global=[action_with_para_str]
	    print "action %s is recorded into historical data\n" %action_with_para_str
	    print "the first action_sequences_global is %s" %action_sequences_global
	    #action_sequences_global = action_sequences
	  
	  else:
	    print "the non_null action_sequences_global list is %s" %action_sequences_global
	    #####action_sequences_global=action_sequences_global.append(action_with_para_str)#update action sequence
	    action_sequences_global.append(action_with_para_str)#update action sequence
	    print "action %s is recorded into historical data\n" %action_with_para_str
	    #action_sequences_global = action_sequences
	  
	 # Open a file
	  fo = open("dict_dynamic_feedback_DM_historical_data.txt", "ab")	
	  fo.write(action_with_para_str+"\n")
     	  fo.close()
	  
    else:
	print 'This is init state'
  
    
    
def callback_record_data2(data):
    
    temp_feed= data.feedback
    
    
    
    task_actions=temp_feed
    
    historical_data_recorder(task_actions)

def consulting_server():
    
    rospy.init_node('likelyhood_server', anonymous=True)    
    
    rospy.Subscriber("srs_decision_making_actions/feedback", ExecutionActionFeedback, callback_record_data2) #record the historical data
    rospy.Subscriber("srs_decision_making_actions/result", ExecutionActionResult, callback_record_data) #record the historical data 
    
    rospy.Service('likelihood', Likelihood, handle_consulting) #provide service
    rospy.Service('ontology', Ontology, handle_consulting2) #provide service
    print "Ready to receive consulting."
    rospy.spin()

if __name__ == "__main__":
    
    consulting_server()
    
