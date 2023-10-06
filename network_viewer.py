#!/usr/bin/env python3

import struct
import socket
import numpy as np
import cv2

from misc_map_tools import make_map

PIXELS_PER_METER = 100


class LidarViewer:
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', 25811))
        self._a_map = make_map(800, 800, PIXELS_PER_METER)
        self.amap = np.copy(self._a_map)
        self.prev_angle = 0.0

    def run(self):
        while True:
            pkt, addr = self._sock.recvfrom(65535)
            #print('rx', len(pkt), 'bytes')

            fsa, lsa = struct.unpack('<HH', pkt[:4])
            pc_data = pkt[4:]

            self.parse_pc_packet(fsa, lsa, pc_data)

    def parse_pc_packet(self, angle_start, angle_end, payload):
        #print('parse_pc_packet')
        rad_start = np.radians(angle_start / 128.0)
        rad_end = np.radians(angle_end / 128.0)
        if rad_end < rad_start:
            rad_end += 2 * np.pi

        ranges = np.frombuffer(payload, np.uint16) / 1000.0  # in meters
        angleSpace = np.linspace(rad_start, rad_end, len(ranges))

        for i, theta in enumerate(angleSpace):
            r = ranges[i]

            # For the Lidar:
            #  Theta 0.0 is pointing backwards (towards the cable), laser rotates clockwise.
            #
            # For the robot, we have an FRD coordinate system (like ArduPilot):
            #  +X is Forward
            #  +Y is Right
            #  (+Z is Down, but we're just 2-dimensional, here)

            frd_x = -r * np.cos(theta)  # Forward is +X, meters
            frd_y = -r * np.sin(theta)  # Rightward is +Y, meters

            # In computer graphics, +x is rightward, +y is downard, and the origin is top-left
            x_pixel = int(round(frd_y * PIXELS_PER_METER + 400))  # rightward pixels, opencv bitmap
            y_pixel = int(round(-frd_x * PIXELS_PER_METER + 400))  # downward pixels, opencv bitmap

            if x_pixel > 1 and x_pixel < 798:
                if y_pixel > 1 and y_pixel < 798:
                    self.amap[y_pixel, x_pixel] = 0, 0, 0

            if angle_start < self.prev_angle:
                cv2.imshow('asdf', self.amap)
                cv2.waitKey(1)
                self.amap = np.copy(self._a_map)

            self.prev_angle = angle_start


viewer = LidarViewer()
viewer.run()


