# this message contains information to define a SRS action 

uint32 objectID #targeted object

string objectName #object name for easy reading

uint32 actionID #action to be applied

string actionName #action name for easy reading

bool component_is_required #a component is required or not

srs_msgs/Component[] components   #components required

bool symbolic_is_required #a component is required or not

uint32 symbolicID # ID of the symbolic

string symbolicName #name of the symbolic

string description  #a short description on the current action set to null if no description