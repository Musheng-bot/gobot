# Command
uint8 START=1
uint8 CANCEL=2
uint8 CHASE_START=4
uint8 CHASE_UPDATE=5
uint8 CHASE_STOP=6

# Heading mode: decides sentry's heading at the end of navigation
uint8 AUTO_HEADING=0
uint8 FIXED_HEADING=1

uint8 command
uint32 task_id
uint32 plan_id
geometry_msgs/Pose goal
uint8 heading_mode 0   # 0=AUTO: 现状(终点面朝目标点); 1=FIXED: 终点对齐 goal.orientation
