#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import signal
import struct
import socket
import numpy as np
import cv2

from misc_utils import get_last_packet
from misc_map_tools import make_map
from misc_map_tools import COLOR_YELLOW, COLOR_BLACK
from misc_map_tools import make_yellow_avoidance_area, make_black_avoidance_area

from YDLidar import YDLidar

PIXELS_PER_METER = 100


MOAB_ADDR = ('192.168.13.201', 12346)

DO_GUI = False
DO_NETWORK = True
IMG_RECV_ADDRESS = ('192.168.13.101', 53521)

def check_color(a, b):
    return a[0] == b[0] and a[1] == b[1] and a[2] == b[2]


class LidarWithVisual(YDLidar):
    def __init__(self, *args, **kwargs):
        super(LidarWithVisual, self).__init__(*args, **kwargs)
        self._a_map = make_map(800, 800, PIXELS_PER_METER)
        make_yellow_avoidance_area(self._a_map, PIXELS_PER_METER)
        make_black_avoidance_area(self._a_map, PIXELS_PER_METER)
        self.amap = np.copy(self._a_map)
        self.prev_angle = 0.0

        self.closest_yellow_object = (500.0, None, None)  # will have the form of r,x,y
        #self.closest_red_object = (500.0, None, None)  # will have the form of r,x,y
        self.closest_black_object = (500.0, None, None)  # will have the form of r,x,y

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(("0.0.0.0", 0))


    def parse_pc_packet(self, angle_start, angle_end, payload):
        rad_start = np.radians(angle_start / 128.0)
        rad_end = np.radians(angle_end / 128.0)
        if rad_end < rad_start:
            rad_end += 2*np.pi

        ranges = np.frombuffer(payload, np.uint16) / 1000.0  # in meters
        angleSpace = np.linspace(rad_start, rad_end, len(ranges))

        for i,theta in enumerate(angleSpace):
            r = ranges[i]  # real distance, meters
            if r > 0.1:
                x = r * np.sin(theta)  # real X, meters
                y = r * np.cos(theta)  # real Y, meters

                x_pixel = int(round(-x*PIXELS_PER_METER + 400))
                y_pixel = int(round(y*PIXELS_PER_METER + 400))

                if x_pixel > 1 and x_pixel < 798:
                    if y_pixel > 1 and y_pixel < 798:
                        pixel_color = self.amap[y_pixel, x_pixel]
                        if check_color(pixel_color, COLOR_YELLOW):
                            if r < self.closest_yellow_object[0]:
                                self.closest_yellow_object = (r, x, y)
                            self.amap[y_pixel, x_pixel] = 0,0,255
                        elif check_color(pixel_color, COLOR_BLACK):
                            if r < self.closest_black_object[0]:
                                self.closest_black_object = (r, x, y)
                            self.amap[y_pixel, x_pixel] = 0,255,255
                        else:
                            self.amap[y_pixel, x_pixel] = 0,0,0

            if angle_start < self.prev_angle:
                if DO_NETWORK:
                    _nothing, pngBuffer = cv2.imencode('*.png', self.amap)
                    bufLen = len(pngBuffer)
                    filepos = 0
                    numbytes = 0
                    START_MAGIC = b"__HylPnaJY_START_PNG %09d\n" % (bufLen)
                    self._sock.sendto(START_MAGIC, IMG_RECV_ADDRESS)
                    while filepos < bufLen:
                        if (bufLen - filepos) < 1400:
                            numbytes = bufLen - filepos
                        else:
                            numbytes = 1400  # ethernet MTU is 1500
                        self._sock.sendto(pngBuffer[filepos:(filepos+numbytes)], IMG_RECV_ADDRESS)
                        filepos += numbytes
                    STOP_MAGIC = b"_g1nC_EOF"
                    self._sock.sendto(STOP_MAGIC, IMG_RECV_ADDRESS)

                if DO_GUI:
                    cv2.imshow('asdf', self.amap)
                    cv2.waitKey(1)

                # Auto-pilot:
                if self.closest_black_object[0] < 499.0:
                    print('STOP')
                    sbus_steering = 1024
                    sbus_throttle = 1024

                elif self.closest_yellow_object[0] < 499.0:
                    r, x, y = self.closest_yellow_object
                    if x < 0:  # object on right
                        print('  <---- steer left')
                        sbus_steering = 1024 - 100
                        sbus_throttle = 1200
                    else:  # object on left
                        print('  steer right ---->')
                        sbus_steering = 1024 + 100
                        sbus_throttle = 1200

                else:
                    sbus_steering = 1024
                    sbus_throttle = 1324

                pkt = struct.pack('HH', sbus_steering, sbus_throttle)
                self._sock.sendto(pkt, MOAB_ADDR)


                # reset the map:
                self.amap = np.copy(self._a_map)
                self.closest_yellow_object = (500.0, None, None)
                self.closest_black_object = (500.0, None, None)

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



