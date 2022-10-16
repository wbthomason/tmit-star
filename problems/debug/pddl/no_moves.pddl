(define (domain clutter)
 (:requirements :strips :equality)
 (:predicates (on-surface ?x -objs ?surf -surface :discrete)
  (arm-empty ?m -manipulator :discrete)
  (holding ?m -manipulator ?x -objs :discrete)
  (at ?m -manipulator ?x -objs)
  (above ?m -manipulator ?surf -surface))

 (:action pickup
  :parameters (?ob -objs ?surf -surface ?m -manipulator)
  :precondition (and (on-surface ?ob ?surf) (arm-empty ?m) (at ?m ?ob))
  :effect (and (holding ?m ?ob) (not (on-surface ?ob ?surf)) 
    (not (arm-empty ?m))))

 (:action putdown
  :parameters (?ob -objs ?surf -surface ?m -manipulator)
  :precondition (and (above ?m ?surf) (holding ?m ?ob))
  :effect (and (arm-empty ?m) (on-surface ?ob ?surf) (not (holding ?m ?ob)))))
