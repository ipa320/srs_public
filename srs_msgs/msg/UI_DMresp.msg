# this message contains information to define the comunication between the Decision making (DM) and UI_Pri when an unexpected error has occured in the execution of an previously issued command

string solution   # currently supported comands from the user are: continue,give_up,move  
string parameter   # used when the command is move to carry the position. Possible vaues are:kitchen, home, order, [1.2, 3.4, 0.8] which are the coordinates from user click on the map
uint32 responseID  # the uniqie ID used by DM to distinguish which responce corresponds to which command. Note: the responceID MUST be the same as the requestID of the corresponding command
