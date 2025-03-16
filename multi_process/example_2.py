import multiprocessing as mp
import time

# Function to simulate a time-consuming task
def worker(num):
    print(f'Worker {num} starting')
    time.sleep(2)
    print(f'Worker {num} finished')

if __name__ == '__main__':
    processes = []
    for i in range(5):
        p = mp.Process(target=worker, args=(i,))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()
