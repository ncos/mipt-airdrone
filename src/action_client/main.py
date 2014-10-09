#!/usr/bin/env python

import roslib; roslib.load_manifest('action_client')
import rospy, smach, smach_ros
import time

from action_client.msg import *
from actionlib import *
from actionlib.msg import *

movement_speed = 0.0

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

class PauseStateDebug(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'abort'])

    def execute(self, userdata):
        rospy.loginfo("PauseStateDebug: press 'y' to continue or 'n' to abort...")

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
        
        if result.land_pad == True:
            rospy.loginfo("move_along_result_cb -> result.land_pad == True")
            return 'land_pad'
                
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
        if result.success == True:
            rospy.loginfo("approach_door_result_cb -> result.succes == True")
            return 'succeeded'
        if result.ortog_pass == True:
            rospy.loginfo("approach_door_result_cb -> result.ortog_pass == True")
            return 'ortog_pass'
        if result.middle_pass == True:
            rospy.loginfo("approach_door_result_cb -> result.middle_pass == True")
            return 'middle_pass'
        if result.part_failed == True:
            rospy.loginfo("approach_door_result_cb -> result.part_failed == True")
            return 'part_failed'

        return 'aborted'
            
    rospy.loginfo("(approach_door_result_cb): This line should never be reached")
    return 'aborted'




def main():
    rospy.init_node('action_server_state_mashine')


    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm0:
        """
        smach.StateMachine.add('Pause', PauseState (),
                               transitions={'continue':'Approach door',
                                            'abort'   :'aborted'})
        
        smach.StateMachine.add('Approach door',
                               smach_ros.SimpleActionState('ApproachDoorAS',
                                                           ApproachDoorAction,
                                                           goal =  ApproachDoorGoal(),
                                                           result_cb = approach_door_result_cb,
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Pause'} )
        """
        
        
        smach.StateMachine.add('Pause', PauseState (),
                               transitions={'continue':'Takeoff',
                                            'abort'   :'aborted'})
        
        smach.StateMachine.add('PauseStateDebug', PauseStateDebug (),
                               transitions={'continue':'Move along',
                                            'abort'   :'aborted'})

        smach.StateMachine.add('DebugState',
                               smach_ros.SimpleActionState('DebugStateAS', 
                                                           DebugStateAction,
                                                           goal =  DebugStateGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'  :'aborted',
                                            'succeeded':'Takeoff'} )
                
        smach.StateMachine.add('Move along',
                               smach_ros.SimpleActionState('MoveAlongAS',
                                                           MoveAlongAction,
                                                           goal =  MoveAlongGoal(vel=movement_speed),
                                                           result_cb = move_along_result_cb,
                                                           outcomes=['aborted', 'succeeded', 'wall_found', 'land_pad']),
                               transitions={'aborted'   :'aborted',
                                            'succeeded' :'Switch wall',
                                            'wall_found':'Approach door',
                                            'land_pad'  :'Landing'} )


        smach.StateMachine.add('Switch wall',
                               smach_ros.SimpleActionState('SwitchWallAS', SwitchWallAction,
                                                           goal =  SwitchWallGoal(rght=1),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'  :'aborted',
                                            'succeeded':'Move along'} )
        
        smach.StateMachine.add('Approach door',
                               smach_ros.SimpleActionState('ApproachDoorAS',
                                                           ApproachDoorAction,
                                                           goal =  ApproachDoorGoal(),
                                                           result_cb = approach_door_result_cb,
                                                           outcomes=['aborted', 'succeeded', 'middle_pass', 'ortog_pass', 'part_failed']),                               
                               transitions={'aborted'     :'Pause',
                                            'ortog_pass'  :'Switch side',
                                            'middle_pass' :'Middle pass',
                                            'part_failed' :'Move along',
                                            'succeeded'   :'Pass door'} )
        
        smach.StateMachine.add('Pass door',
                               smach_ros.SimpleActionState('PassDoorAS',
                                                           PassDoorAction,
                                                           goal =  PassDoorGoal(vel=-0.8),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Move along'} )
                                            
        smach.StateMachine.add('Switch side',
                               smach_ros.SimpleActionState('SwitchSideAS',
                                                           SwitchSideAction,
                                                           goal =  SwitchSideGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Approach wall'} )
        
        smach.StateMachine.add('Approach wall',
                               smach_ros.SimpleActionState('ApproachWallAS',
                                                           ApproachWallAction,
                                                           goal =  ApproachWallGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Move along'} )
        
        smach.StateMachine.add('Middle pass',
                               smach_ros.SimpleActionState('MiddlePassAS',
                                                           MiddlePassAction,
                                                           goal =  MiddlePassGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Move along'} )
        
        smach.StateMachine.add('Takeoff',
                               smach_ros.SimpleActionState('TakeoffAS',
                                                           TakeoffAction,
                                                           goal =  TakeoffGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'DebugState'} )
        
        smach.StateMachine.add('Landing',
                               smach_ros.SimpleActionState('LandingAS',
                                                           LandingAction,
                                                           goal =  LandingGoal(),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'Pause',
                                            'succeeded' :'Pause'} )
                
    # Create and start the introspection server
    # This is for debug purpose
    # sis = smach_ros.IntrospectionServer('introspection_server', sm0, '/STATE_MASHINE')
    # sis.start()


    # Execute SMACH plan
    outcome = sm0.execute()
    print("State mashine has finished with result ", outcome)

    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    if not rospy.has_param('movement_speed'):
        rospy.loginfo("movement_speed parameter was not specified. Terminating...")
        exit()
    
    
    movement_speed = rospy.get_param("movement_speed")
    print "movement_speed =", movement_speed

    
    main()
