"""Condition behaviors for checking system state."""

import py_trees
from py_trees import common


class IsPathComplete(py_trees.behaviour.Behaviour):
    """Check if path queue is empty."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=common.Access.READ)
        
    def update(self):
        path_queue = self.blackboard.get("path_queue")
        if not path_queue:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
