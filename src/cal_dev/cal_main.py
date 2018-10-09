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
import tf 
import os 

NUM_ATTEMPS = 1
PLANNER_NAME = "RRTConnect"
MAX_PLAN_TIMES = 5

# FIXME: Unfinished part
class pose_bound:
  def __init__(self, translation, rotation):
    pass

    
# TODO
def get_path_all_around(current_pose):
  ''' 
  Get a path that mosty cover the configuration area of robot arm

  Target Representation:
    geometry_msgs -> Pose.msg:
      Point position (float64 x y z)
      Quaternion orientation (float64 w+xi+yj+zk)
  '''
  pass

# FIXME: Unfinished codes
def get_path_zigzag(current_pose):
  ''' 
  Get a path that moving slightly around current position in a zig zag manner

  Target Representation:
      geometry_msgs -> Pose.msg:
        Point position (float64 x y z)
        Quaternion orientation (float64 w+xi+yj+zk)
  '''
  # bound = pose_bound(x, y, z, yaw, pitch, roll)# create pose bound
  path = []
  new_pose = current_pose
  new_pose.position.z = current_pose.position.z + 0.03
  path.append(new_pose)
  return path



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


# specify planner
group.set_planner_id(PLANNER_NAME+'kConfig1')
group.set_num_planning_attempts(1)
group.set_planning_time(5)

plan_zigzag = []

path_zigzag = get_path_zigzag(group.get_current_joint_values())

# Rough Calibration
for path_target in path_zigzag:
  group.set_pose_target(path_target)
  for j in range(MAX_PLAN_TIMES):
    plan = group.plan()
    if len(plan.joint_trajectory.points) == 0:
      continue 
    plan_list.append(plan)
    rospy.sleep(0.5)

  if len(plans_place) == 0:
    print("Fail to get any plan")
    break

  group.execute(plans_place[0])
  status = os.system('sh /home/bionicdl/catkin_ws/src/phoxi_camera/image_cap.sh')
  rospy.sleep(10)
  print(status)
    

#print('sucess rate for pick: %s and place: %s'%(len(plans_pick),len(plans_place)))
#bag = rosbag.Bag('test.bag', 'w')
#bag.write('plan',plan1)
#bag.close()


