(define (problem shelves_pb_7)
 (:domain shelves)
 (:objects
  r_gripper_tool_joint -manipulator
  blue_stick_1 green_stick_1 red_stick_1 red_stick_2 red_stick_3 red_stick_4 red_stick_5 -objs)
 (:init
  (on-lower-shelf blue_stick_1) (on-upper-shelf green_stick_1) (arm-empty r_gripper_tool_joint) (on-upper-shelf red_stick_1) (on-lower-shelf red_stick_2) (on-upper-shelf red_stick_3) (on-lower-shelf red_stick_4) (on-upper-shelf red_stick_5))
 (:goal (and (on-upper-shelf blue_stick_1) (on-lower-shelf green_stick_1) (arm-empty r_gripper_tool_joint))))
