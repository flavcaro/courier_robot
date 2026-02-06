import py_trees
from .actions import move_forward, move_back, move_left, move_right, arm_up, arm_down, open_hand, close_hand

# Nodi BT
class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveForward"):
        super().__init__(name)
    def update(self):
        move_forward()
        return py_trees.common.Status.SUCCESS

class MoveBack(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveBack"):
        super().__init__(name)
    def update(self):
        move_back()
        return py_trees.common.Status.SUCCESS

class GrabObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="GrabObject"):
        super().__init__(name)
    def update(self):
        arm_down()
        close_hand()
        arm_up()
        return py_trees.common.Status.SUCCESS

class DropObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="DropObject"):
        super().__init__(name)
    def update(self):
        arm_down()
        open_hand()
        arm_up()
        return py_trees.common.Status.SUCCESS

# Sequenza principale
def root_bt():
    root = py_trees.composites.Sequence(name="Delivery Sequence", memory=False)
    root.add_child(MoveForward())
    root.add_child(GrabObject())
    root.add_child(MoveBack())
    root.add_child(DropObject())
    return root
