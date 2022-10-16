(define (problem shelves_pb_2)
 (:domain shelves)
 (:objects
  l_gripper_tool_joint r_gripper_tool_joint -manipulator
  blue_stick_1 green_stick_1 -objs)
 (:init
  (on-lower-shelf blue_stick_1) (on-upper-shelf green_stick_1) (arm-empty l_gripper_tool_joint) (arm-empty r_gripper_tool_joint))
 (:goal (and (on-upper-shelf blue_stick_1) (on-lower-shelf green_stick_1) (arm-empty r_gripper_tool_joint) (arm-empty l_gripper_tool_joint))))
