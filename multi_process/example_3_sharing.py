# sharing data between processes using queue
from multiprocessing import Process, Queue

def worker(queue):
    queue.put('Hello from worker')

if __name__ == '__main__':
    queue = Queue()
    p = Process(target=worker, args=(queue,))
    p.start()
    print(queue.get())  # Output: Hello from worker
    p.join()
