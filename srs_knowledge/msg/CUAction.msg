# still at alpha stage. not finalised yet. 

# indicate if task finished successfully (1); no feasible solution (-1); continue executing the given actions in this message (0)
int32 status

# [ifNavigate, ifDetect, ifGrasp]. 0: yes, 1: no, 2: others (undefined)
#int32[] actionFlags

#MoveAction ma
#PerceptionAction pa
#GraspAction ga
GenericAction generic

# move, perception, grasp, or generic
string actionType