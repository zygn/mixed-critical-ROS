import rclpy
from rclpy.executors import Executor, TimeoutException, ShutdownException, ConditionReachedException
from rclpy.node import Node 

from mixed_critic.hi_node import HiNode
from mixed_critic.lo_node import LoNode

from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Callable, Union

class TimeoutObject:
    """Use timeout object to save timeout."""

    def __init__(self, timeout: float):
        self._timeout = timeout

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, timeout):
        self._timeout = timeout

class MyExecutor(Executor):

    def __init__(self, nodes):
        super().__init__()
        self.my_executor = ThreadPoolExecutor()
        
        self.mode = True
        self.nodes = nodes 
        for node in self.nodes:
            self.add_node(node)
    
    def set_mode(self, mode):
        self.mode = mode 

    def ensure_spin(self, node, mode):
        node.pub()
        if mode:
            return rclpy.spin_once(node, timeout_sec=node.hi_mode_hz)
        else:
            return rclpy.spin_once(node, timeout_sec=node.lo_mode_hz)
    
        
    def mode_spin(self):
        
        while rclpy.ok:
            futures = []
            with self.my_executor as exec:
                for node in self.nodes:
                    futures.append(exec.submit(self.ensure_spin, node, self.mode))
                    # futures.append(exec.submit(self.release_by_mode))
                
                for future in futures:
                    future.result()

    def release_by_mode(
            self,
            node,
            timeout_sec: Optional[Union[float, TimeoutObject]] = None,
            wait_condition: Callable[[], bool] = lambda: False
            ):
        
        # while rclpy.ok:
        futures = []
        # for node in self.nodes:
        if self.mode:
            timeout_sec = node.hi_mode_hz
        else:
            timeout_sec = node.lo_mode_hz
        
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec, None, wait_condition)
            futures.append(handler)
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        except ConditionReachedException:
            pass
        else:
            handler()
        
        # for handler in futures:
        #     handler()
        #     if handler.exception() is not None:
        #         raise handler.exception()

        #     handler.result()
            
                    




class Scheduler(Node):

    def __init__(self) -> None:
        super().__init__('scheduler_node')
        
        self.mode = True # false: LO, true: HI

    @property
    def get_mode(self):
        return self.mode 
    

# def ensure_spin(node, mode, executor):
#     node.pub()
#     if mode:
#         return rclpy.spin_once(node, executor=executor, timeout_sec=node.hi_mode_hz)
#     else:
#         return rclpy.spin_once(node, executor=executor, timeout_sec=node.lo_mode_hz)


def main():
    rclpy.init()
    scheduler_node = Scheduler()
    nodes = [HiNode(), LoNode()]
    # futures = []

    executor = MyExecutor(nodes)
    try:
        executor.mode_spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()

    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(HiNode())
    # executor.add_node(LoNode())
    # executor_thread = threading.Thread(target=executor.spin)

    # while rclpy.ok():
    #     with ThreadPoolExecutor() as exec:

    #         for node in nodes:
    #             futures.append(ensure_spin(node, scheduler_node.get_mode, exec))
    #             # futures.append(exec.submit(ensure_spin, node, scheduler_node.get_mode))

    #     for future in futures:
    #         future.result()
    
    # scheduler_node.destroy_node()
    # for node in nodes:
    #     node.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()