import rospy
import numpy as np
import quaternion
from za_msgs.msg import PosVelSetpoint
from za_msgs.msg import ZaState

PERIOD = 5 # sec

class CircleTrajectory:
    def __init__(self, radius, start):
        self._radius = radius
        self._start  = start
        self._origin = np.array([start[0] + radius, start[1], start[2]])

    def __call__(self, t):
        position = np.array([-self._radius * np.cos(t * 2 * np.pi),
                            -self._radius * np.sin(t * 2 * np.pi),
                            0]) + self._origin
        
        velocity = 2 * np.pi * self._radius * np.array([np.sin(t * 2 * np.pi),
                                                       -np.cos(t * 2 * np.pi),
                                                        0])

        return position, velocity
    
def createCommand(position, orientation, velocity):
    command = PosVelSetpoint()
    command.pose.position.x = position[0]
    command.pose.position.y = position[1]
    command.pose.position.z = position[2]

    command.pose.orientation.w = 1
    command.pose.orientation.x = 0
    command.pose.orientation.y = 0
    command.pose.orientation.z = 0

    command.twist.linear.x = velocity[0]
    command.twist.linear.y = velocity[1]
    command.twist.linear.z = velocity[2]
    return command

def main():
    pub = rospy.Publisher('task_priority_controller/command', PosVelSetpoint, queue_size=1)
    rospy.init_node('task_priority_controlller_test', anonymous=False)
    state = rospy.wait_for_message('za_state_controller/za_states', ZaState, 10)
    initial_tranformation = np.array(state.O_T_EE).reshape(4, 4).T
    orient = quaternion.from_rotation_matrix(initial_tranformation[:3, :3])

    # time parameterize a circle
    traj = CircleTrajectory(0.05, initial_tranformation[:3, 3])

    ti = rospy.Time.now()
    rate = rospy.Rate(250) # 250 Hz
    scaling = 1 / PERIOD
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - ti).to_sec() * scaling

        pos, vel = traj(elapsed)
        command = createCommand(pos, orient, vel)

        pub.publish(command)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass