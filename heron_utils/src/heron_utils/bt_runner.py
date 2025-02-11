import threading
import time
import timeit  # for default_timer()
import rospy
import py_trees as pt
import ros_trees as rt


class BehaviourTreeRunner(rt.trees.BehaviourTree):
    def __init__(
        self, tree_name: str, root: pt.behaviour.Behaviour, hz: float = 10
    ) -> None:
        """
        wrapping the ros_trees BehaviourTree class
        
        adds pause/start/stop functionality with the use of threading
        """
        super(BehaviourTreeRunner, self).__init__(tree_name, root)
        self.tree_name = tree_name
        self.hz = hz
        self.running = False
        self.paused = False
        self._thread = None
        self._pause_condition = threading.Condition()

    def _run_tree(
        self,
        push_to_start: bool = True,
        log_level: str = None,
        setup_timeout: float = 5,
    ) -> None:
        """threaded execution for behaviour tree"""
        # logging level
        if log_level:
            log_level = log_level.upper()

            if not log_level in rt.trees.BehaviourTree._LOG_LEVELS:
                raise ValueError(
                    "Provided log level \'%s'\ is not supported."
                    "Supported values are: %s" %
                    (log_level, rt.BehaviourTree._LOG_LEVELS)
                )
            pt.logging.level = pt.logging.Level[log_level]

        # setup
        if not self.setup(timeout=setup_timeout):
            self.root.logger.error(
                f"Failed to setup the {self.tree_name} tree. Aborting run."
            )
            return False
        
        # run tree with or without enter
        if push_to_start:
            input(f"Press enter to start the {self.tree_name} tree.")
        else:
            rospy.loginfo(f"Running {self.tree_name}.")

        rate = 1.0 / self.hz
        rospy.loginfo(f"Starting Behaviour Tree: {self.tree_name}")

        while self.running:
            with self._pause_condition:
                while self.paused and self.running:
                    rospy.loginfo("BT Paused...")
                    self._pause_condition.wait()  # wait for resume

            # tick tree
            t = timeit.default_timer()
            self.tick(None, None)
            tree_status = self.root.status
            remaining = rate - (timeit.default_timer() - t)

            # if tree reacehs terminal state exit
            if tree_status in [pt.common.Status.SUCCESS, pt.common.Status.FAILURE]:
                rospy.loginfo(f"{self.tree_name} finished with status: {tree_status}")
                self.running = False # stop loop
                break

            if remaining > 0:
                try:
                    time.sleep(remaining)
                except KeyboardInterrupt:
                    break

        rospy.loginfo(f"Stopping Behaviour Tree: {self.tree_name}")

    def start(self):
        if self.running:
            rospy.loginfo("Tree is already running.")
            return

        self.running = True
        self.paused = False
        self._thread = threading.Thread(target=self._run_tree, daemon=True)
        self._thread.start()
        rospy.loginfo("Behaviour tree started.")

    def stop(self):
        self.running = False

        if self.paused:
            self.resume()  # ensure pause condition is released

        if self._thread:
            self._thread.join()
            self._thead = None

        rospy.loginfo("Behaviour tree stopped.")

    def pause(self):
        if not self.running:
            rospy.loginfo("Tree is not running, cannot pause.")
            return
        self.paused = True
        rospy.loginfo("Behaviour tree paused.")

    def resume(self):
        with self._pause_condition:
            self.paused = False
            self._pause_condition.notify()  # wake up paused thread

        if not self.running:
            rospy.loginfo("Tree is not running, cannot resume.")
            return
        rospy.loginfo("Behaviour tree resumed.")

    def is_running(self) -> bool:
        return self.running

    def is_paused(self) -> bool:
        return self.paused
