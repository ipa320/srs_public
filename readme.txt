the symbol grounder will translate the commands sent by the commander to best grasp pose in (x, y, theta).

to test the symbol grounder,
first, run the commander.py by using "rosrun symbolic_grounding commander.py".
it will publish control command and parameters once a second.

the command message type: 
int32 grasp		#grasp type: 1 for right, 2 for front, 3 for top
float32 rb_x		#current robot base location and orientation
float32 rb_y
float32 rb_theta
float32 obj_x		#target object location and orientation
float32 obj_y
float32 obj_theta


second, run the symbol_grounder.py by using "rosrun symbolic_grounding symbol_grounder.py"
it will subscribe to the commander node and generate a best grasp pose for the robot base and a reachability value which indicates how easily the object can be reached.
