import rospy
import actionlib
import unittest
import numpy as np
from cartesian_control_msgs.msg import *


def make_rectangle(center, x, y, z):
    xmin = center[0] - x / 2
    xmax = center[0] + x / 2
    ymin = center[1] - y / 2
    ymax = center[1] + y / 2
    zmin = center[2] - z / 2
    zmax = center[2] + z / 2

    # x: (f)ront, (b)ack
    # y: (l)eft, (r)ight
    # z: (t)op, (d)own
    flu = np.array([xmin, ymax, zmax])
    fru = np.array([xmin, ymin, zmax])
    frd = np.array([xmin, ymin, zmin])
    fld = np.array([xmin, ymax, zmin])
    bld = np.array([xmax, ymax, zmin])
    blu = np.array([xmax, ymax, zmax])
    bru = np.array([xmax, ymin, zmax])
    brd = np.array([xmax, ymin, zmin])

    return (flu, fru, frd, fld, bld, blu, bru, brd)

def make_cube(center, width):
    return make_rectangle(center, width, width, width)

def all_edges(center, width):
    flu, fru, frd, fld, bld, blu, bru, brd = make_cube(center, width)

    path = [fru, flu, blu, bru, fru, frd, fru, flu,
            fld, bld, blu, flu, fld, frd, brd, bru,
            blu, bld, brd, bru]

    return path

def manipulability_example():
    x = 0.9
    y = 0.9
    z = 0.6

    floor_offset = 0.5
    center = np.array([0, 0, z / 2 + floor_offset])

    flu, fru, frd, fld, bld, blu, bru, brd = make_rectangle(center, x, y, z)
    path = [blu, flu, fld, bld, blu, bru, brd, frd, fru, bru, brd, bld]

    return path

def set_pose(position, orientation, point: CartesianTrajectoryPoint):
    point.pose.position.x = position[0]
    point.pose.position.y = position[1]
    point.pose.position.z = position[2]
    point.pose.orientation.w = orientation[0]
    point.pose.orientation.x = orientation[1]
    point.pose.orientation.y = orientation[2]
    point.pose.orientation.z = orientation[3]


class TaskPriorityTrajectoryControllerTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('task_priority_trajectory_controller_test')

        self.client = actionlib.SimpleActionClient('/task_priority_trajectory_controller/'
                                                   'follow_cartesian_trajectory',
                                                    TrajectoryExecutionAction)
        self.client.wait_for_server()

        self.start = np.array([0.6281204359253633, -1.150820145534631e-07, 0.507627954931471])
        self.orient = np.array([1, 0, 0, 0])
        
        self.width = 0.3 # width of cube
        self.v_max = 500.0 / 1000.0;  # m/s
        self.a_max = 1000.0 / 1000.0; # m/s^2
        self.j_max = 3000.0 / 1000.0; # m/s^3
    
    #@unittest.skip("skipping test_cartesian_trajectory")
    def test_cartesian_trajectory(self):
        traj = CartesianTrajectory()

        start_pose = CartesianTrajectoryPoint()
        set_pose(self.start, self.orient, start_pose)
        traj.points.append(start_pose)

        path = all_edges(self.start, self.width)
        for segment in path:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj.points.append(pose)
        traj.points.append(start_pose)

        goal = TrajectoryExecutionGoal()
        goal.trajectory = traj
        goal.limits.v_max = self.v_max
        goal.limits.a_max = self.a_max
        goal.limits.j_max = self.j_max

        self.client.send_goal(goal)
        self.client.wait_for_result()

    #@unittest.skip("skipping test_large_cube")
    def test_large_cube(self):
        traj = CartesianTrajectory()

        start_pose = CartesianTrajectoryPoint()
        set_pose(self.start, self.orient, start_pose)
        traj.points.append(start_pose)

        path = manipulability_example()
        for segment in path:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj.points.append(pose)
        traj.points.append(start_pose)

        goal = TrajectoryExecutionGoal()
        goal.trajectory = traj
        goal.limits.v_max = self.v_max
        goal.limits.a_max = self.a_max
        goal.limits.j_max = self.j_max

        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == "__main__":
    import rostest
    rostest.rosrun("za_controllers", "task_priority_trajectory_controller_test", TaskPriorityTrajectoryControllerTest)