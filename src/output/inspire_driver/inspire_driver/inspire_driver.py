import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import multiprocessing
import time
from .inspire_sdkpy import inspire_sdk, inspire_hand_default

def worker(ip,LR,name,network=None):
    rclpy.init()
    node = Node("inspire_hand_"+LR)
    handler = inspire_sdk.ModbusDataHandler(network=network,ip=ip, LR=LR, device_id=1, node=node)

    call_count = 0 
    start_time = time.perf_counter()
    time.sleep(0.5)  # Allow some time for the node to initialize
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        while True:
            data_dict = handler.read()
            call_count += 1
            time.sleep(0.001)

            if call_count % 10 == 0:
                elapsed_time = time.perf_counter() - start_time
                frequency = call_count / elapsed_time
                print(f"{name} frequency: {frequency:.2f} Hz, call count: {call_count}, elapsed time: {elapsed_time:.6f} seconds")
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - start_time
        frequency = call_count / elapsed_time if elapsed_time > 0 else 0
        print(f"{name} terminated. call count: {call_count}, elapsed time: {elapsed_time:.6f} seconds, frequency: {frequency:.2f} Hz")

    node.destroy_node()
    rclpy.shutdown()

def main(args=None):

    process_r = multiprocessing.Process(target=worker, args=('192.168.123.210', 'r', "right_hand"))
    process_l = multiprocessing.Process(target=worker, args=('192.168.123.211', 'l', "left_hand"))

    process_r.start()
    time.sleep(0.6)  # Ensure the first process starts before the second
    process_l.start()

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        process_r.terminate()
        process_l.terminate()

if __name__ == "__main__":
    main()