#!/usr/bin/env python3

import struct
import threading
import serial


def parse_single_response(mode, _type, payload):

    if _type == 0x04:  # device information
        model_number, firm_minor, firm_major, hard_version = struct.unpack('BBBB', payload[:4])
        print('Firmware version: %d.%d' % (firm_major, firm_minor))
        print('Hardware version: %d' % (hard_version))
        print('Model:', model_number)
        serNum = ''
        for val in payload[4:]:
            serNum += val.__str__()
        print('Serial:', serNum)

    elif _type == 0x06:  # device health
        status, health = struct.unpack('<BH', payload)
        if status == 0 and health == 0:
            print('Health status is good.')
        else:
            print("Problem.  status_code: %x,  error_code: %x" % (status, health))


class YDLidar:
    def __init__(self, device, baud):
        self._ser = serial.Serial(device, baud)
        self._t1 = threading.Thread(target=self.reading_task)
        self._t1.start()

        # just for debugging, etc:
        self._single_packets = []
        self._continuous_packets = []


    def reading_task(self):
        #count = 0
        #while count < 100:
        while True:
            c = self._ser.read(1)

            # Single-reponse packets begin with 0xA5, 0x5A
            if c == b'\xa5':
                c = self._ser.read(1)
                if c == b'\x5a':
                    len_mode_type = self._ser.read(5)
                    plen, = struct.unpack('I', len_mode_type[:4])
                    plen &= 0x3fffffff
                    mode = (len_mode_type[3] & 0xc0) >> 6

                    # mode 1 means continuous response, so the single-response payload is ignored
                    if mode == 1:
                        plen = 0

                    _type = len_mode_type[4]
                    print("plen:", plen, "mode:", mode, "type:", _type)

                    payload = self._ser.read(plen)
                    self._single_packets.append(  (len_mode_type, payload) )
                    parse_single_response(mode, _type, payload)

                else:  # invalid 2nd char
                    print('parser miss (expecting 0x5a):', c)

            # Continuous-reponse packets begin with 0xAA, 0x55
            elif c == b'\xaa':
                c = self._ser.read(1)
                if c == b'\x55':
                    ct, lsn, fsa, lsa, cs = struct.unpack('<BBHHH', self._ser.read(8))
                    pc_data = self._ser.read(lsn*2)
                    #self._continuous_packets.append( (ct, lsn, fsa, lsa, cs, pc_data) )
                    #count += 1
                    if ct == 0:
                        self.parse_pc_packet(fsa, lsa, pc_data)

                else:  # invalid 2nd char
                    print('parser miss (expecting 0x55):', c)

            else:  # invalid 1st char
                print('parser miss (expecting either 0xa5 or 0xaa):', c)

    def parse_pc_packet(self, angle_start, angle_end, payload):
        pass

    def stop(self):
        pkt = bytes([0xa5, 0x65])
        self._ser.write(pkt)

    def force_stop(self):
        pkt = bytes([0xa5, 0x00])
        self._ser.write(pkt)

    def get_device_info(self):
        pkt = bytes([0xa5, 0x90])
        self._ser.write(pkt)

    def get_device_health(self):
        pkt = bytes([0xa5, 0x92])
        self._ser.write(pkt)

    def get_sample_rate(self):
        pkt = bytes([0xa5, 0xd1])
        self._ser.write(pkt)

    def scan(self):
        pkt = bytes([0xa5, 0x60])
        self._ser.write(pkt)


#lidar.get_sample_rate()

# 0xd1 -- get_sample_rate:
# 0xd0 -- set_sample_rate:
# 0x0d -- get_aimspeed:
# 0x93 -- ??
# 0x60 -- scan:

