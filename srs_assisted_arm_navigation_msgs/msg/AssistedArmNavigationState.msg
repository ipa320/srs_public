###
# This message is used to publish internal state of assisted arm navigation node
###

# Internal state (planned, executing etc.) - TBD

bool planning_started
bool position_reachable # IK can be found
bool position_in_collision

bool joints_out_of_limits

string error_description # position not reachable, collision etc.

# Spacenav variables
bool position_locked
bool orientation_locked

bool aco_state # true - enable, false - disabled

string[] attached_objects
