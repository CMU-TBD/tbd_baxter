
import rospy

from tbd_baxter_msgs.msg import(
    HeadCmdAction,
    HeadCmdGoal
)
import actionlib


class HeadController(object):

    def __init__(self, topic: str = 'head_command_pan'):
        self._client = actionlib.SimpleActionClient(topic, HeadCmdAction)
        self._client.wait_for_server()

    def move_head(self, target: float, duration: rospy.Duration, wait: bool = True, noise: bool = False) -> None:
        """ Move Baxter's head to the target

        parameters
        ----------
        target: float
            In Radian, the position of Baxter's head
        time: rospy.Duration
            Time plan for baxter to get to the position
        wait: boolean
            Whether to wait for the function to finish 
        noise: boolean
            Whether there will be idle noise movements
        topic: str
            Topic if different
        """
        goal = HeadCmdGoal()
        goal.enable_noise = noise
        goal.target = target
        goal.duration = duration

        self._client.send_goal(goal)
        if wait:
            return self._client.wait_for_result(rospy.Duration(10))
        else:
            return True

