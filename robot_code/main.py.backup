# main.py
# Controller semplice per Behavior Tree base (senza navigazione)

from bt.behaviours import root_bt
import py_trees
import time

def run_bt():
    """Esegue il Behavior Tree base."""
    # Creo la radice del BT
    root = root_bt()

    # Creo l'albero con py_trees
    tree = py_trees.trees.BehaviourTree(root)

    print("Partenza del Behavior Tree!")

    try:
        while True:
            tree.tick()  # tick del BT
            time.sleep(0.5)  # mezzo secondo tra un tick e l'altro

    except KeyboardInterrupt:
        print("\nInterruzione manuale. Stop del BT.")

if __name__ == "__main__":
    run_bt()
