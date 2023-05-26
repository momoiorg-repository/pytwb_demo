import py_trees
from pytwb.common import behavior

@behavior
class DebugSuccess(py_trees.behaviour.Behaviour):
    def __init__(self, name, message=None, loop=1):
        super(DebugSuccess, self).__init__(name)
        self.message = message
        self.loop = 1

    def update(self):
        if self.loop > 0:
            self.loop -= 1
            return py_trees.common.Status.RUNNING
        if self.message:
            print(f"debug success [{self.message}]")
        else:
            print("debug success")
        return py_trees.common.Status.SUCCESS

# quoted from py_trees
@behavior
class PassThrough(py_trees.decorators.Decorator):
    """
    This decorator simply reflects the child's current status.

    This behaviour is useful for debugging or visualisation purposes.
    """

    def __init__(self, name: str, child: py_trees.behaviour.Behaviour):
        """
        Initialise with the standard decorator arguments.

        Args:
            name: the decorator name
            child: the child to be decorated
        """
        super(PassThrough, self).__init__(name=name, child=child)
    
    def initialise(self) -> None:
        super().initialise()
        print(f'PassTrough initialize')

    def update(self) -> py_trees.common.Status:
        """
        Just reflect the child status.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """
        print(f'PassThrough update{self.decorated.status}')
        return self.decorated.status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        print(f'PassTrough terminate {new_status}')
        return super().terminate(new_status)