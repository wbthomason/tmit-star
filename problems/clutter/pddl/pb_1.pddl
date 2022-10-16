(define (problem clutter_pb_1)
 (:domain clutter)
 (:objects
  table_1 table_2 table_3 table_4 -surface
  l_gripper_tool_joint r_gripper_tool_joint -manipulator
  blue_stick_1 -objs )
 (:init
  (on-surface blue_stick_1 table_1) (arm-empty l_gripper_tool_joint) (arm-empty r_gripper_tool_joint))
 (:goal (and (on-surface blue_stick_1 table_3) (arm-empty r_gripper_tool_joint) (arm-empty l_gripper_tool_joint))))
