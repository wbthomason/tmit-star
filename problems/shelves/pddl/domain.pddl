(define (domain shelves)
 (:requirements :strips :equality)
 (:predicates (arm-empty ?m -manipulator :discrete)
  (on-upper-shelf ?x -objs :discrete)
  (on-lower-shelf ?x -objs :discrete)
  (on-top-shelf ?x -objs :discrete)
  (holding ?m -manipulator ?x -objs (kinematic (link ?m ?x) (unlink ?m ?x)))
  (at ?m -manipulator ?x -objs)
  (in-lower-region ?m -manipulator)
  (in-upper-region ?m -manipulator)
  (in-top-region ?m -manipulator))

 (:action pick-lower
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (on-lower-shelf ?ob) (arm-empty ?m) (at ?m ?ob))
  :effect (and (holding ?m ?ob) (not (on-lower-shelf ?ob)) (not (arm-empty ?m))))

 (:action pick-upper
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (on-upper-shelf ?ob) (arm-empty ?m) (at ?m ?ob))
  :effect (and (holding ?m ?ob) (not (on-upper-shelf ?ob)) (not (arm-empty ?m))))

 (:action pick-top
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (on-top-shelf ?ob) (arm-empty ?m) (at ?m ?ob))
  :effect (and (holding ?m ?ob) (not (on-top-shelf ?ob)) (not (arm-empty ?m))))

 (:action putdown-lower
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (in-lower-region ?m) (holding ?m ?ob))
  :effect (and (arm-empty ?m) (on-lower-shelf ?ob) (not (holding ?m ?ob)))))

 (:action putdown-upper
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (in-upper-region ?m) (holding ?m ?ob))
  :effect (and (arm-empty ?m) (on-upper-shelf ?ob) (not (holding ?m ?ob)))))

 (:action putdown-top
  :parameters (?ob -objs ?m -manipulator)
  :precondition (and (in-top-region ?m) (holding ?m ?ob))
  :effect (and (arm-empty ?m) (on-top-shelf ?ob) (not (holding ?m ?ob)))))
