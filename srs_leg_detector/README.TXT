
to make:
rosmake srs_legdetector 


to run:
1) start roscore
2) roslaunch srs_legdetector leg_detector.launch 

to see the results:
3) play bagfile with the laser scans,e.g.: rosbag play ~/scandata/person_scan2_2011-05-27-12-47-17.bag
  Note: still untested with a real laser range finder
4) eighter watch the topic /particle_filt_cloud or run legListener to save in a file (you need to create a file and modify the location accordingly in legListener.cpp) 


Future work:

I have plans are adding MHT for more reliable tracking of multiple people
