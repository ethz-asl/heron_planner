#!/usr/bin/env python

import rospy
import py_trees as pt
import ros_trees as rt


################################################################################
############################### leaf definitions ###############################
################################################################################


class Wait(rt.leaves.Leaf):
    def __init__(self, duration=1, task_name="", *args, **kwargs):
        super(Wait, self).__init__(
            name=task_name if task_name else "Wait",
            load_fn=self._load_fn,
            eval_fn=self._eval_fn,
            *args,
            **kwargs,
        )
        self.duration = duration

    def _load_fn(self):
        self.start = rospy.get_time()
        return None

    def _eval_fn(self, value):
        return True

    def _is_leaf_done(self):
        return rospy.get_time() - self.start > self.duration


class PopFromList(rt.leaves.Leaf):

    def __init__(self, task_name="", pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__(
            name=task_name if task_name else "Pop from list",
            result_fn=self._pop_item,
            *args,
            **kwargs,
        )
        self.pop_position = pop_position

    def _pop_item(self):
        if not self.loaded_data:
            return None
        item = self.loaded_data.pop(self.pop_position)
        rospy.logerr(f"Current list item: {item}")
        if self.load_key is not None:
            rt.data_management.set_value(self.load_key, self.loaded_data)
        else:
            rt.data_management.set_last_value(self, self.loaded_data)
        return item


class Print(rt.leaves.Leaf):

    def __init__(self, *args, **kwargs):
        super(Print, self).__init__(
            "Print", result_fn=self._print, *args, **kwargs
        )

    def _print(self):
        print(self.loaded_data)
        return True


class PrintObjects(rt.leaves.Leaf):

    def __init__(self, *args, **kwargs):
        super(PrintObjects, self).__init__(
            name="Print Objects", result_fn=self._print_objects, *args, **kwargs
        )

    def _print_objects(self):
        if self.loaded_data is None or not self.loaded_data:
            print("The detector found no objects!")
        else:
            print(
                "The detector found %d objects at the following coordinates:"
                % len(self.loaded_data)
            )
            for o in self.loaded_data:
                print(
                    "\t'%s' of pixel dimensions %dx%d @ top left coordinates:"
                    " (%d,%d)"
                    % (o.class_label, o.width, o.height, o.x_left, o.y_top)
                )

        return True


class WaitForEnterKey(rt.leaves.Leaf):

    def __init__(self, task_name="", *args, **kwargs):
        super(WaitForEnterKey, self).__init__(
            name=task_name if task_name else "Wait for enter key",
            result_fn=self._wait_for_enter,
            *args,
            **kwargs,
        )

    def _wait_for_enter(self):
        # NOTE: this is blocking within a leaf ... typically BAD
        input(
            self.loaded_data
            if self.loaded_data
            else "Press enter to continue: "
        )
        return True


class SaveData(rt.leaves.Leaf):
    def __init__(self, data, task_name="", *args, **kwargs):
        super(SaveData, self).__init__(
            name=task_name if task_name else "Data generator",
            load_value=data,
            save=True,
            *args,
            **kwargs,
        )


################################################################################
############################## custom decorators ###############################
################################################################################


class ConditionSuccessThreshold(pt.behaviour.Behaviour):
    def __init__(self, name: str, bb_key="", min_successes=1):
        super().__init__(name=name)
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard_key = bb_key
        self.min_successes = min_successes

    def update(self):
        count = self.blackboard.get(self.blackboard_key, 0)
        return (
            pt.common.Status.SUCCESS
            if count >= self.min_successes
            else pt.common.Status.FAILURE
        )


class CountSuccesses(pt.decorators.Condition):
    def __init__(self, name: str, child, bb_key="success_counter"):
        """
        A decorator that counts how many times a child returns SUCCESS.

        :param child: The child behavior to track
        :param blackboard_key: Blackboard variable to store the count
        """
        super().__init__(name=name, child=child)
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard_key = bb_key
        self.blackboard.set(self.blackboard_key, 0)
        self.count = self.blackboard.get(self.blackboard_key)

    def update(self):
        result = self.decorated.status

        if result == pt.common.Status.SUCCESS:
            self.count = self.blackboard.get(self.blackboard_key, 0)
            self.blackboard.set(self.blackboard_key, self.count + 1)
        rospy.logerr(f"success count: {self.count}")

        return result


class Repeat(pt.decorators.Decorator):
    """
    Runs until successful num of time. -1 runs continously.
    """

    def __init__(
        self, name: str, child: pt.behaviour.Behaviour, num_success: int
    ):
        super().__init__(name=name, child=child)
        self.success = 0
        self.num_success = num_success

    def initialise(self) -> None:
        """reset currently registered no. successes"""
        self.success = 0

    def update(self) -> pt.common.Status:
        """Repeat until nth consecutive success"""
        if self.decorated.status == pt.common.Status.FAILURE:
            self.feedback_message = f"failed, aborting [status: {self.success} success from {self.num_success}]"
            return pt.common.Status.FAILURE
        elif self.decorated.status == pt.common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = f"success [status: {self.success} success from {self.num_success}]"
            if self.success == self.num_success:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.RUNNING
        else:
            self.feedback_message = f"running [status: {self.success} success from {self.num_success}]"
            return pt.common.Status.RUNNING


class RetryUntilSuccessful(pt.decorators.Decorator):
    """
    custom decorator that retries its child until it suceeds
    or a maximum num of attempts has been reached
    """

    def __init__(self, child, max_attempts, name="RetryUntilSuccessful"):
        super().__init__(name=name, child=child)
        self.child = child
        self.max_attempts = max_attempts
        self.current_attempt = 1

    def initialise(self):
        """reset the attempt counter"""
        pass

    def update(self) -> pt.common.Status:
        if self.current_attempt >= self.max_attempts:
            rospy.logwarn(f"Max attempts ({self.max_attempts}) reached.")
            return pt.common.Status.FAILURE

        child_status = self.child.status

        if child_status == pt.common.Status.SUCCESS:
            return pt.common.Status.SUCCESS
        elif child_status == pt.common.Status.FAILURE:
            self.current_attempt += 1
            rospy.logwarn(
                f"Retrying... Attempt ({self.current_attempt} / {self.max_attempts})"
            )
            # self.child.stop(pt.common.Status.INVALID) # reset child
            return pt.common.Status.RUNNING
        else:
            rospy.logwarn(
                f"Retrying... waiting ({self.current_attempt} / {self.max_attempts})"
            )
            return pt.common.Status.RUNNING
