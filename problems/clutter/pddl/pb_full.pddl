(define (problem clutter_pb_full)
 (:domain clutter)
 (:objects
  table_1 table_2 table_3 table_4 -surface
  l_gripper_tool_joint r_gripper_tool_joint -manipulator
  blue_stick_1 blue_stick_2 blue_stick_3 blue_stick_4 blue_stick_5 blue_stick_6 blue_stick_7 green_stick_1 green_stick_2 green_stick_3 green_stick_4 green_stick_5 green_stick_6 green_stick_7 red_stick_1 red_stick_2 red_stick_3 red_stick_4 red_stick_5 red_stick_6 red_stick_7 red_stick_8 red_stick_9 red_stick_10 red_stick_11 red_stick_12 red_stick_13 red_stick_14 -objs )
 (:init
  (on-surface blue_stick_1 table_1) (on-surface blue_stick_2 table_1) (on-surface blue_stick_3 table_1)
  (on-surface green_stick_1 table_1) (on-surface green_stick_2 table_1) (on-surface green_stick_3 table_1) (on-surface green_stick_4 table_1)
  (on-surface red_stick_1 table_1) (on-surface red_stick_2 table_1) (on-surface red_stick_3 table_1) (on-surface red_stick_4 table_1)
  (on-surface red_stick_5 table_1) (on-surface red_stick_6 table_1) (on-surface red_stick_7 table_1)
  (on-surface blue_stick_4 table_2) (on-surface blue_stick_5 table_2) (on-surface blue_stick_6 table_2) (on-surface blue_stick_7 table_2)
  (on-surface green_stick_5 table_2) (on-surface green_stick_6 table_2) (on-surface green_stick_7 table_2)
  (on-surface red_stick_8 table_2) (on-surface red_stick_9 table_2) (on-surface red_stick_10 table_2) (on-surface red_stick_11 table_2)
  (on-surface red_stick_12 table_2) (on-surface red_stick_13 table_2) (on-surface red_stick_14 table_2)
  (arm-empty l_gripper_tool_joint) (arm-empty r_gripper_tool_joint))
 (:goal (and
         (on-surface blue_stick_1 table_3) (on-surface blue_stick_2 table_3) (on-surface blue_stick_3 table_3) (on-surface blue_stick_4 table_3)
         (on-surface blue_stick_5 table_3) (on-surface blue_stick_6 table_3) (on-surface blue_stick_7 table_3)
         (on-surface green_stick_1 table_4) (on-surface green_stick_2 table_4) (on-surface green_stick_3 table_4) (on-surface green_stick_4 table_4)
         (on-surface green_stick_5 table_4) (on-surface green_stick_6 table_4) (on-surface green_stick_7 table_4))))
