#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


import os
import select
import socket
import struct
import time
import numpy as np
import cv2 as cv



WRITE_VIDEO = False
vid_out = None

if len(sys.argv) == 2:
    WRITE_VIDEO = True
    vid_filename = sys.argv[1]

if WRITE_VIDEO:
    vid_out = cv.VideoWriter(vid_filename, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (2544, 480))


START_MAGIC = b"__HylPnaJY_START_PNG "
STOP_MAGIC = b"_g1nC_EOF"

IMAGE_PORT = 53521


img_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
img_sock.bind(("0.0.0.0", IMAGE_PORT))

cv.namedWindow('amap')
cv.moveWindow('amap', 0, 0)


rx_img = {'size': 0, 'packets': [], 'inBand': False}

image = np.empty((800, 800, 3), np.uint8)

rx_data = {
    'remote_id': 0,
    'nboxes': 0,
    'closestIdx': -1,
    'distances': np.zeros(16, np.float),
    'bboxes': np.zeros((16, 4), np.int32),
    'ts': time.time()
}


_last_render_time = time.time()
def maybe_render():
    global _last_render_time
    t1 = time.time()
    tDelta = t1 - _last_render_time
    if tDelta > 0.05:
        _last_render_time = t1
        cv.imshow('amap', image)
        cv.waitKey(1)
        if WRITE_VIDEO:
            vid_out.write(image)


def rx_png_packet(data, addr):
    global rx_img
    global image
    if rx_img['inBand']:
        if data == STOP_MAGIC:
            rx_img['inBand'] = False
            if len(rx_img['packets']) > 1:
                jpgData = b''.join(rx_img['packets'])
                if rx_img['size'] == len(jpgData):
                    rx_img['jpgData'] = np.frombuffer(jpgData, np.uint8)
                    try:
                        im = cv.imdecode(rx_img['jpgData'], cv.IMREAD_UNCHANGED)
                    except Exception as ee:
                        print("Failed to decode jpeg:", ee)
                    else:
                        if im is not None:
                            image = im
                            maybe_render()
                else:
                    print('image size doesn\'t match')
        else:
            rx_img['packets'].append(data)
        
    if data.startswith(START_MAGIC):
        rx_img['size'] = int(data[-10:], 10)
        rx_img['packets'] = []
        rx_img['inBand'] = True



while True:
    inputs, outputs, errors = select.select([img_sock], [], [])
    for oneInput in inputs:
        if oneInput == img_sock:
            imgdata, addr = img_sock.recvfrom(2048)
            rx_png_packet(imgdata, addr)


