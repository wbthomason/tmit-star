(define (problem shelves_pb_18)
 (:domain shelves)
 (:objects
  r_gripper_tool_joint -manipulator
  blue_stick_1 green_stick_1 red_stick_1 red_stick_2 red_stick_3 red_stick_4 red_stick_5 red_stick_6 red_stick_7 red_stick_8 red_stick_9 red_stick_10 red_stick_11 red_stick_12 red_stick_13 red_stick_14 red_stick_15 red_stick_16 -objs)
 (:init
  (on-lower-shelf blue_stick_1) (on-upper-shelf green_stick_1) (arm-empty r_gripper_tool_joint) (on-upper-shelf red_stick_1) (on-lower-shelf red_stick_2) (on-upper-shelf red_stick_3) (on-lower-shelf red_stick_4) (on-upper-shelf red_stick_5) (on-lower-shelf red_stick_6) (on-upper-shelf red_stick_7) (on-lower-shelf red_stick_8) (on-upper-shelf red_stick_9) (on-lower-shelf red_stick_10) (on-upper-shelf red_stick_11) (on-lower-shelf red_stick_12) (on-upper-shelf red_stick_13) (on-lower-shelf red_stick_14) (on-upper-shelf red_stick_15) (on-lower-shelf red_stick_16))
 (:goal (and (on-upper-shelf blue_stick_1) (on-lower-shelf green_stick_1) (arm-empty r_gripper_tool_joint))))
