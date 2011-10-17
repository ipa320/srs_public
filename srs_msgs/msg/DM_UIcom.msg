# this message contains information to define the comunication between the Decision making (DM) and UI_Pri in case of unexpected error in executing an previously issued command

string command   # currently supported comands for user intervention are: continue? (meaning do you want to continue:yes/no), position? (where to go:kitchen,home,order or coordinate from the map) 
string message   # some free text to be displayed in the popup window to the user
uint32 requestID # the uniqie ID used by DM to distinguish which responce corresponds to which command. Note: the requestID MUST be included in the responceID.
