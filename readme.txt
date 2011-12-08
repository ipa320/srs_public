This package contains three services:

1. symbol_grounding_grasp_base_pose

This service is used when the target object is detected and the grasp type has been chosen.

inputs: 

	grasp type (1: right grasp, 2: front grasp, 3: top grasp) 

	target object location and orientation (obj_x, obj_y, obj_th)
	
	Note: Now we assume that the object is heading towards positive x axis when its orientation is 0, and when obj_th increases, the object rotates around z axis anticlockwise.

	current robot base location and orientation (rb_x, rb_y, rb_th)

outputs:

	a list contains reachability and a robot base pose for grasping the object (reach, gbp_x, gbp_y, gbp_th)

	Note: The reachability is a value (from 0 to 1) which indicates how easily the target object can be reached when the robot is located at its current location and orientation.

2. symbol_grounding_primitive_base_pose

This service is used when the robot want to grasp and object but it only knows the primitive location and orientation of the object.

inputs:

	primitive object location and orientation (obj_x, obj_y, obj_th)

outputs:

	a list contains robot base pose for grasping the object (pbp_x, pbp_y, pbp_th)

3. symbol_grounding_scan_base_pose

This service is used when the robot want to scan the surface of an parent object (e.g. table) for dectecting a child object (e.g. milkbox).

inputs:

	parent object location, orientation and shape (table_x, table_y, table_th, table_length, table_width)

	Note: The parent object (table) is heading towards positive x axis when its orientation is 0, and when table_th increases, the table rotates around z axis anticlockwise. The layout of the kitchen/room is also need to be known. The parameters of this service are designed for the robot working in ipa kitchen now.

outputs:

	a list of robot base poses for scanning the parent object (sbp1_x, sbp1_y, sbp1_th, sbp2_x, sbp2_y, sbp2_th, ...)

	Notes: The surface of the parent object can be covered after it has been scanned by the robot from all the poses listed in the output. When the parent object is a square table, table_length = table_width. when the parent object is a round table, table_length = table_width = 2 * table_radius.

	





