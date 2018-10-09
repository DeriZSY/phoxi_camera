import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from std_msgs.msg import String
import rosbag
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

NUM_ATTEMPS = 1
PLANNER_NAME = "RRTConnect"

def get_target_coord():
  pass 

# initialize moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

# instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()
# instantiate a Planning SceneInterface object
scene = moveit_commander.PlanningSceneInterface()
# Instantiate a Move Group Commander object
group = moveit_commander.MoveGroupCommander("Arm")
# publish trajectory to RViz
display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                 moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)


# set two target position
pick_joint_positions = [-1.6571229807091412, -1.8987741030128207, 1.8297206708493179, -1.4844425337884575, -1.588071931215888, -0.10369734183303081]
place_joint_positions = [4.72094163298607e-05, -1.2083540076804813, 1.3808265007842797, -1.7260865345759318, -1.5880401480919681, -9.419479356147349e-05]

# specify planner

group.set_planner_id(PLANNER_NAME+'kConfig1')
group.set_num_planning_attempts(1)
group.set_planning_time(5)

plans_pick = []
plans_place = []
for i in range(2):
  group.set_start_state_to_current_state() # set start sate to current position
  group.clear_pose_targets() # clear previous targets 

  # if: ... set joint value target to place joint positions
  if abs(group.get_current_joint_values()[0]-pick_joint_positions[0])<0.001: 
    group.set_joint_value_target(place_joint_positions) # set target 
    for j in range(8): # try to get plans for 8 times 
      plan = group.plan()
      if len(plan.joint_trajectory.points)==0: 
        continue
      plans_place.append(plan)
      rospy.sleep(0.5)
    # quit if cannot get a plan 
    if len(plans_place)==0:
      print('Fail to get any place plan!')
      break
    # execute the first available plan
    group.execute(plans_place[0])
    rospy.sleep(3)
  
  # else: ... set joint value target to pick joint positions
  else:
    group.set_joint_value_target(pick_joint_positions)
    for j in range(8):# try to get a plan for 8 times
      plan = group.plan()
      if len(plan.joint_trajectory.points)==0:
        continue
      plans_pick.append(plan)
      rospy.sleep(0.5)
    # quit if cannot get a plan
    if len(plans_pick)==0:
      print('Fail to get any pick plan!')
      break
    # execute first plan available
    group.execute(plans_pick[0])
    rospy.sleep(3)

print('sucess rate for pick: %s and place: %s'%(len(plans_pick),len(plans_place)))
#bag = rosbag.Bag('test.bag', 'w')
#bag.write('plan',plan1)
#bag.close()

