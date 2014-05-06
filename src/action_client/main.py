#!/usr/bin/env python

import roslib; roslib.load_manifest('action_client')
import rospy, smach, smach_ros
import time

from action_client.msg import *
from actionlib import *
from actionlib.msg import *




class EntryState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'abort'])

    def execute(self, userdata):
        rospy.loginfo("EntryState: press 'y' to continue or 'n' to abort...")

        char = raw_input()
        while char.lower() not in ("y", "n"):
            rospy.loginfo("Please, choose 'y' or 'n'")
            char = raw_input()
        
        if char == 'y':
            return 'start'
        return 'abort'




def main():
    rospy.init_node('action_server_state_mashine')


    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm0:
        smach.StateMachine.add('ENTRY_STATE', EntryState (),
                               transitions={'start':'Move along',
                                            'abort':'preempted'})
        
        

        smach.StateMachine.add('Move along',
                               smach_ros.SimpleActionState('MoveAlongAS', MoveAlongAction,
                                                       goal =  MoveAlongGoal(vel=0.8)),
                               transitions={'aborted'  :'aborted',
                                            'succeeded':'Switch wall'} )
        smach.StateMachine.add('Switch wall',
                               smach_ros.SimpleActionState('SwitchWallAS', SwitchWallAction,
                                                       goal =  SwitchWallGoal(left=1)),
                               transitions={'aborted'  :'aborted',
                                            'succeeded':'Move along'} )

        def rotate_goal_callback(userdata, default_goal):
            goal = RotationGoal()
            goal.angle = 0
            return goal

        #smach.StateMachine.add('ROTATE_SECOND',
         #                      smach_ros.SimpleActionState('rotation', RotationAction,
          #                                             goal_cb = rotate_goal_callback),
           #                     transitions={'aborted'  :'aborted',
            #                                 'succeeded':'ROTATE_FIRST'} )




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
