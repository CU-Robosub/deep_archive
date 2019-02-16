#!/usr/bin/env python

"""
This is the top level state machine.
Inside of it will be all of the sub state machines (one for each task)

TODO:
- Get Ctrl-C to work
- Dynamically configure which tasks to do from rosparams defined in a yaml
- preempt on kill switch service and then restart on same service
"""

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Empty

from tasks.start_gate import StartGate

def point2Pose(self, point):
    """
    Converts a point msg to pose msg
    """
    assert type(point) == Point
    poseMsg = Pose()
    poseMsg.position = point
    return poseMsg

def genPoseMsg(list_xyz):
    pose = Pose()
    pose.position.x = list_xyz[0]
    pose.position.y = list_xyz[1]
    pose.position.z = list_xyz[2]
    return pose

def main():
    rospy.init_node('smach_top')

    # load in rosparams here
    gpp = genPoseMsg([7.0,3.0,2.0]) # gate pose prior (SHOULD BE ABOUT 4.5,0,1.5
    gateSearchAlg = 'simple'
    dbg = 1.0 # dist behind gate

    sm_top = smach.StateMachine(['success', 'aborted'])

    # initialize all of the state machines
    sg_sm = StartGate(gpp, gateSearchAlg, dbg) # keep the outcomes up here for readability

    with sm_top:
        smach.StateMachine.add('StartGate', sg_sm, transitions={'task_aborted' : 'StartGate', 'task_success':'success'})

    try:
        outcome = sm_top.execute() # does this allow us to receive messages in the states?
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
