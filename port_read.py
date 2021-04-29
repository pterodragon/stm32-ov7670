import serial
import re
import cv2
import numpy as np
import time
import queue
from concurrent.futures import ThreadPoolExecutor

img_rows = 144
img_cols = 174

img_bytes = img_rows * img_cols
BUF_THRESHOLD = img_bytes * 10  # 10 frames
pat = re.compile(b"\xff" * 100)
bytes_block_queue = queue.Queue()

def input_consumer():
    print('start consumer')
    buf = b''
    while True:
        if bytes_block_queue.empty():
            print('queue empty')
            time.sleep(1)
        else:
            print(f'{bytes_block_queue.qsize()=}')
            start = time.time()
            item = bytes_block_queue.get()
            buf += item
            if not buf.startswith(b'\xff' * 100):
                it = pat.finditer(buf)
                try:
                    st = next(it).start()
                except StopIteration:
                    buf = b''
                buf = buf[st:]
            while len(buf) >= img_bytes:
                img_raw = buf[:img_bytes]
                buf = buf[img_bytes:]
                img_arr = np.resize(np.array(list(map(int,img_raw)), dtype=np.uint8), (img_rows, img_cols))
                cv2.imshow('cam2', img_arr)
                print('--- frame displayed')
                time.sleep(1)
            end = time.time()

            time_taken = end - start
            print(f'time used: {time_taken:.6f}, buf size = {len(buf)}')
            if len(buf) > BUF_THRESHOLD:
                print('buf too full, dropping frames')
                buf = b'buf'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.waitKey(0)
    cv2.destroyAllWindows()





def read_input_worker():
    # mind the timeout! (Serial.read can read less than stated)
    with serial.Serial("/dev/ttyACM0", 230400, timeout=10) as ser:
        while True:
            print(f'{ser.in_waiting=}')
            # print(f'{bytes_block_queue.qsize()=}')
            # s = ser.read(img_bytes * 2)  # read up to x bytes (timeout)
            start = time.time()
            s = ser.read(2 ** 16)  # read up to x bytes (timeout)
            bytes_block_queue.put(s)
            end = time.time()
            time_taken = end - start
            print('put block time used=', f'{time_taken:.6f}')

            # it = pat.finditer(s)
            # try:
            #     st = next(it).start()
            # except StopIteration:
            #     print("failed to find 100 reference white pixels")
            #     exit()
            # img_raw = s[st : st + img_bytes]


        # img_arr = np.resize(np.array(list(map(int,img_raw)), dtype=np.uint8), (img_rows, img_cols))

        # # img = cv2.imdecode(img_arr, cv2.IMREAD_GRAYSCALE)
        # cv2.imshow('cam2', img_arr)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        # print('frame')
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

def main():
    with ThreadPoolExecutor(max_workers=2) as executor:
        fut1 = executor.submit(read_input_worker)
        fut2 = executor.submit(input_consumer)
        print('fut2 res', fut2.result())
        print('fut1 res', fut1.result())

if __name__ == '__main__':
    main()
