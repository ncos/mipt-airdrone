#!/usr/bin/env python

import roslib; roslib.load_manifest('action_client')
import rospy, smach, smach_ros
import time

from action_client.msg import *
from actionlib import *
from actionlib.msg import *



# Ask user whether we want to launch the quadrotor or not
class PauseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'abort'])

    def execute(self, userdata):
        rospy.loginfo("PauseState: press 'y' to continue or 'n' to abort...")

        char = raw_input()
        while char.lower() not in ("y", "n"):
            rospy.loginfo("Please, choose 'y' or 'n'")
            char = raw_input()
        
        if char == 'y':
            return 'continue'
        return 'abort'


def move_along_result_cb(userdata, status, result):
    if status == GoalStatus.PREEMPTED:
        rospy.loginfo("move_along_result_cb -> status == GoalStatus.PREEMPTED")
        return 'aborted'
    
    if status == GoalStatus.ABORTED:
        rospy.loginfo("approach_door_result_cb -> status == GoalStatus.ABORTED")
        return 'aborted'
    
    if status == GoalStatus.SUCCEEDED:
        if result.error == True:
            rospy.loginfo("move_along_result_cb -> result.error == True")
            return 'aborted'
                
        if result.found == True:
            rospy.loginfo("move_along_result_cb -> result.found == True")
            return 'wall_found'
                
        return 'succeeded'
            
    rospy.loginfo("(move_along_result_cb): This line should never be reached")
    return 'aborted'


def approach_door_result_cb(userdata, status, result):
    if status == GoalStatus.PREEMPTED:
        rospy.loginfo("approach_door_result_cb -> status == GoalStatus.PREEMPTED")
        return 'aborted'
    
    if status == GoalStatus.ABORTED:
        rospy.loginfo("approach_door_result_cb -> status == GoalStatus.ABORTED")
        return 'aborted'
    
    if status == GoalStatus.SUCCEEDED:
        if result.succes == True:
            rospy.loginfo("approach_door_result_cb -> result.succes == True")
            return 'succeeded'
        return 'aborted'
            
    rospy.loginfo("(approach_door_result_cb): This line should never be reached")
    return 'aborted'




def main():
    rospy.init_node('action_server_state_mashine')


    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm0:
        smach.StateMachine.add('Pause', PauseState (),
                               transitions={'continue':'Move along',
                                            'abort'   :'aborted'})
        


        
        smach.StateMachine.add('Move along',
                               smach_ros.SimpleActionState('MoveAlongAS',
                                                           MoveAlongAction,
                                                           goal =  MoveAlongGoal(vel=0.8),
                                                           result_cb = move_along_result_cb,
                                                           outcomes=['aborted', 'succeeded', 'wall_found']),
                               transitions={'aborted'   :'aborted',
                                            'succeeded' :'Switch wall',
                                            'wall_found':'Approach door'} )


        smach.StateMachine.add('Switch wall',
                               smach_ros.SimpleActionState('SwitchWallAS', SwitchWallAction,
                                                           goal =  SwitchWallGoal(left=1),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'  :'aborted',
                                            'succeeded':'Move along'} )
        
        smach.StateMachine.add('Approach door',
                               smach_ros.SimpleActionState('ApproachDoorAS',
                                                           ApproachDoorAction,
                                                           goal =  ApproachDoorGoal(),
                                                           result_cb = approach_door_result_cb,
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Move along',
                                            'succeeded' :'Pause'} )
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('introspection_server', sm0, '/STATE_MASHINE')
    sis.start()


    # Execute SMACH plan
    outcome = sm0.execute()
    print("Stopped", outcome)

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
