import concurrent.futures
import multiprocessing
import time

def cont_loop(stop_event):
    print("in cont loop")
    counter = 0
    while not stop_event.is_set():
        time.sleep(1)
        print("slept")
        counter += 1
    return counter

def single_loop():
    print("in single Loop")
    time.sleep(5)
    return "done"

if __name__ == "__main__":
    with multiprocessing.Manager() as manager:
        stop_event = manager.Event()

        with concurrent.futures.ProcessPoolExecutor() as executor:
            while True:
                cont_future = executor.submit(cont_loop, stop_event)
                single_future = executor.submit(single_loop)
                single_result = single_future.result()
                stop_event.set()
                cont_result = cont_future.result()
                print("Times slept while waiting: ", cont_result)
                stop_event.clear()
