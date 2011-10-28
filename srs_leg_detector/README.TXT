
to make:
rosmake srs_legdetector 


to run:
1) start roscore
2) roslaunch srs_leg_detector srs_leg_detector.launch 

to see how it is working:
3) either start the laser range finder or play a bagfile with the laser scans,e.g.: rosbag play ~/scandata/person_scan2_2011-05-27-12-47-17.bag
4) watch the topic /particle_filt_cloud. Alternatevely run the legListener to save the output in a file (you need to create a file and modify the location accordingly in legListener.cpp and compile) 


