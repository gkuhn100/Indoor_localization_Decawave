from sense_hat import SenseHat
import multiprocessing as mp
import time

tag1 = serial.Serial("/dev/ttyACM0", 115200, timeout = 1)
tag2 = serial.Serial("/dev/ttyACM1", 115200, timeout = 1)
time.sleep(1)

def get_accel():
    

if __name__ == '__main__':
    while True:
