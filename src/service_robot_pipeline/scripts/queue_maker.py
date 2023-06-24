#!/usr/bin/env python

import py_trees


class GetObjectFromQueue(py_trees.behaviour.Behaviour):
    """Gets a location name from the queue"""

    def __init__(self, name, blackboard_key):
        super(GetObjectFromQueue, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.bb_key = blackboard_key

    def update(self):
        """Checks for the status of the navigation action"""
        objects_list = self.bb.get(self.bb_key)
        if len(objects_list) == 0:
            self.logger.info("No more objects available available")
            return py_trees.common.Status.INVALID
        else:
            chosen_object = objects_list.pop()
            self.logger.info("Object chosen is : %s" % chosen_object)
            self.bb.set(self.bb_key, chosen_object)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info("Terminated with status %s" % new_status)
