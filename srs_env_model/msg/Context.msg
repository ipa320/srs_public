# context tags
uint8 status_tag
uint8 action_tag
uint8 connection_tag
uint8 collision_hazard_tag

# status tags
uint8 OK=0
uint8 EMERGENCY=1

# action tags
uint8 DEFAULT=0
uint8 GRASPING=1
uint8 MOVING=2

# connection tags
uint8 UNKNOWN=0
uint8 LAN=1
uint8 WIFI=2

# collision hazard tags
uint8 NONE=0 
uint8 LOW=1
uint8 MEDIUM=2
uint8 HIGH=3
uint8 COLLISION=4
