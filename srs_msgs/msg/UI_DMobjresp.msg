# this message contains information to define the comunication between the Decision making (DM) and UI_Pri when an unexpected error has occured in the execution of an previously issued command

string solution   # currently supported comands from the user are: yes or no
uint32 responseID  # the uniqie ID used by DM to distinguish which responce corresponds to which command. Note: the responceID MUST be the same as the requestID of the corresponding command
