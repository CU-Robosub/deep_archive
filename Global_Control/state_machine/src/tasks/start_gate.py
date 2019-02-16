#!/usr/bin/env python

"""
StartGate Task, attempts to go through the start gate
Objectives:
1) Search (based on prior)
2) Attack (goes behind gate based on arg distBehindGate)

"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty


class StartGate(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, gatePosePrior, searchAlg, distBehindGate):
        assert type(searchAlg) == str
        assert type(gatePosePrior) == Pose

        super(StartGate, self).__init__(self.outcomes) # become a state machine first

        self.gatePose = gatePosePrior
        self.initObjectives(gatePosePrior, searchAlg, distBehindGate)
        self.initMapperSubs()
        self.linkObjectives()

    def initObjectives(self, gatePriorPose, searchAlg, distBehind):
        self.search = Search(searchAlg, gatePriorPose)
        self.attack = Attack(gatePriorPose, distBehind)

    def initMapperSubs(self):
        rospy.Subscriber('/Global_State/start_gate', PoseStamped, self.start_gate_pose_cb)

    def linkObjectives(self):
        with self: # we are a StateMachine

            # NOTE we should be aborted before we reach the pose, aborting means we've found what we're looking for before we finished our path
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'Attack', 'success':'Search'})


            smach.StateMachine.add('Attack', self.attack, transitions={'success':'task_success', 'aborted':'Attack'})

    def start_gate_pose_cb(self, msg):
        rospy.loginfo("Received StartGate Pose")
        self.gatePose = msg.pose
        self.attack.start_gate_pose = msg.pose
        self.search.request_abort() # we've found our pose, so stop searching

class Attack(Objective):
    """
    Tell the sub to go through the gate
    """

    replanThreshold = 1.0 # if the change in startgate pose is greater than this value, we should replan our path there
    _start_gate_pose = None
    outcomes=['success','aborted']
    replanCounter = 0

    def __init__(self, priorPose, distBehindGate):
        self.distBehind = distBehindGate
        self._start_gate_pose = priorPose
        super(Attack, self).__init__(self.outcomes, "Attack")

    @property
    def start_gate_pose(self):
        """
        Properties allow us to determine when start_gate_pose variable has been changed (See start_gate_pose setter )
        """
        return self._start_gate_pose

    @start_gate_pose.setter
    def start_gate_pose(self, value):
        """"
        We just got a new pose for the startgate, decide what to do, this may be preempting our current task so that we loop again
        """
        rospy.loginfo("ATTACH RECEIVED A POSE!!!!!!!!!!!!")
        print value.position
        prevPose = self._start_gate_pose
        self._start_gate_pose = value

        changeInPose = self.getDistance(prevPose.position, value.position)
        if changeInPose > self.replanThreshold or self.replanCounter > 20:
            self.replanCounter = 0;
            self.request_abort() # this will loop us back to execute
        else:
            self.replanCounter += 1

    def execute(self, userdata):
        rospy.loginfo("Executing Attack")
        self.clear_abort()

        targetPose = self.start_gate_pose # let's just go to the gate...
#        targetPose = self._getPoseBehindStartGate(self.start_gate_pose, self.distBehind)

        if self.goToPose(targetPose):
            return 'aborted'
        return "success"

    def _getPoseBehindStartGate(self, gatePose, distBehind):
        """
        Get the point that is distBehind the startgate for the sub to go to
        """
        assert type(distBehind) == float
        assert type(gatePose) == Pose

        orientation_list = [ gatePose.orientation.x, \
                             gatePose.orientation.y, \
                             gatePose.orientation.z, \
                             gatePose.orientation.w ]
        (groll, gpitch, gyaw) = euler_from_quaternion(orientation_list)

        orientation_list = [ self.curPose.orientation.x, \
                             self.curPose.orientation.y, \
                             self.curPose.orientation.z, \
                             self.curPose.orientation.w ]
        (sroll, spitch, syaw) = euler_from_quaternion(orientation_list)



        # use the orientation of the gate to then place a point right behind it
        # TODO do the calculation for a gate facing any direction

        return gatePose


if __name__ == "__main__":
    rospy.init_node("start_gate_test")
    ps = Pose()
    ps.position.x = 1
    ps.position.y = 1
    ps.position.z = 1
    sg = StartGate(ps, 'simple', 1.0)
    sis = smach_ros.IntrospectionServer('fuckuluke', sg.sm, '/fuck')
    sis.start()
    sg.execute(0)
    sis.stop()
