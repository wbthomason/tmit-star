(define (problem clutter_pb_2)
 (:domain clutter)
 (:objects
  table_1 table_2 table_3 table_4 -surface
  l_gripper_tool_joint r_gripper_tool_joint -manipulator
  blue_stick_1 green_stick_1 -objs)
 (:init
  (on-surface blue_stick_1 table_1) (on-surface green_stick_1 table_1) (arm-empty l_gripper_tool_joint) (arm-empty r_gripper_tool_joint))
 (:goal (and (on-surface blue_stick_1 table_3) (on-surface green_stick_1 table_4) (arm-empty r_gripper_tool_joint) (arm-empty l_gripper_tool_joint))))
