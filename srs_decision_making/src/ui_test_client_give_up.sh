#! /bin/bash

clear

echo "**** This is a test client that simulates the user intervention through Ipad   ****"
echo "It listens on /DM_UI/interface_cmd until it receives a suitable command and then sends a reply on /DM_UI/interface_cmd_response simulating the actions of the user on the Ipad"
echo






while [ true ]; do

echo

echo "Waiting for a command on /DM_UI/interface_cmd... "


echo
 if [ $(rostopic echo -n 1 /DM_UI/interface_cmd | grep command| cut -b 10-17) = "continue" ]; then
    `rostopic pub -1 /DM_UI/interface_cmd_response srs_msgs/UI_DMresp  "give_up" "give_up" 5 > /dev/null 2>&1` 
 else 
    `rostopic pub -1 /DM_UI/interface_cmd_response srs_msgs/UI_DMresp "move" "kitchen" 6 > /dev/null 2>&1`
 fi

 echo reply sent 
 echo
 echo

done
