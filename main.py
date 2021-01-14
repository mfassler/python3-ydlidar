#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import signal
import numpy as np
import cv2

from misc_utils import get_last_packet
from misc_map_tools import make_map

from YDLidar import YDLidar

PIXELS_PER_METER = 100

class LidarWithVisual(YDLidar):
    def __init__(self, *args, **kwargs):
        super(LidarWithVisual, self).__init__(*args, **kwargs)
        self._a_map = make_map(800, 800, PIXELS_PER_METER)
        self.amap = np.copy(self._a_map)
        self.prev_angle = 0.0


    def parse_pc_packet(self, angle_start, angle_end, payload):
        rad_start = np.radians(angle_start / 128.0)
        rad_end = np.radians(angle_end / 128.0)
        if rad_end < rad_start:
            rad_end += 2*np.pi

        ranges = np.frombuffer(payload, np.uint16) / 1000.0  # in meters
        angleSpace = np.linspace(rad_start, rad_end, len(ranges))

        for i,theta in enumerate(angleSpace):
            r = ranges[i]
            x = r * np.sin(theta)
            y = r * np.cos(theta)

            x_pixel = int(round(-x*PIXELS_PER_METER + 400))
            y_pixel = int(round(y*PIXELS_PER_METER + 400))

            if x_pixel > 1 and x_pixel < 798:
                if y_pixel > 1 and y_pixel < 798:
                    self.amap[y_pixel, x_pixel] = 0,0,0

            if angle_start < self.prev_angle:
                cv2.imshow('asdf', self.amap)
                cv2.waitKey(1)
                self.amap = np.copy(self._a_map)

            self.prev_angle = angle_start


    def _thread_quit(self):
        cv2.destroyWindow('asdf')


    def quit(self):
        self.force_stop()
        self._running = False
        self._t1.join()
        self._ser.close()



lidar = LidarWithVisual('/dev/ttyUSB0', 500000)
def handle_quit(sig, frame):
    print('Shutting down...')
    lidar.quit()

signal.signal(signal.SIGHUP, handle_quit)
signal.signal(signal.SIGINT, handle_quit)

lidar.force_stop()
lidar.stop()
lidar.get_device_health()
lidar.get_device_info()

#lidar.get_sample_rate()  # not really working yet...

lidar.scan()



