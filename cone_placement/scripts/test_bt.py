#!/usr/bin/env python

def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial One",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    priorities.add_child(idle)

    return root
