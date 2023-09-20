#!/usr/bin/env python3

import time
import struct
import socket
import threading
import serial


class YDLidar:
    def __init__(self, device, baud, destination_addr, offset_degrees=0):
        '''
        Parameters
        ----------
        device : path to serial port device (eg: "/dev/ttyUSB0")
        baud : baud rate of serial port
        destination_addr : network address of viewer (eg:  ("192.168.1.5", 25811)
        offset_degrees : rotational offset of lidar (to allow for mis-mounting)
        '''

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(("0.0.0.0", 0))
        self._DEST_ADDR = destination_addr

        self._ser = serial.Serial(device, baud)

        # Allow for the lidar to be slightly mis-mounted (rotation):
        self._offset = int(round(offset_degrees * 128))

        # Send stop command:
        pkt = bytes([0xa5, 0x65])  # Stop scanning (and switch to idle mode)
        self._ser.write(pkt)

        # Wait for things to settle down:
        time.sleep(0.01)
        self._ser.read(self._ser.inWaiting())

        self._running = False
        self._t1 = None

    def start(self):
        self._running = True
        self._t1 = threading.Thread(target=self.reading_task)
        self._t1.start()
        pkt = bytes([0xa5, 0x60])  # Start scanning (and switch to scan mode)
        self._ser.write(pkt)

    def stop(self):
        pkt = bytes([0xa5, 0x65])  # Stop scanning (and switch to idle mode)
        self._ser.write(pkt)
        self._running = False
        self._t1.join()

    def set_desired_scan_frequency(self, desired_frequency: float):
        # device can actually be set to higher frequencies...  prolly not good for it
        assert desired_frequency <= 12.0, 'Requested scan frequency too high'
        assert desired_frequency >= 5.0, 'Requested scan frequency too low'

        freq = self.get_scan_frequency()
        if freq is None:
            time.sleep(0.1)
            freq = self.get_scan_frequency()

        for i in range(20):
            err = freq - desired_frequency
            if err < -1:
                freq = self.increase_scan_freq_1hz()
            elif err <= -0.09:
                freq = self.increase_scan_freq_01hz()
            elif err > 1:
                freq = self.decrease_scan_freq_1hz()
            elif err >= 0.09:
                freq = self.decrease_scan_freq_01hz()
            else:
                break

        return freq

    def reading_task(self):
        lsn = None
        fsa = None
        lsa = None
        while self._running:
            c = self._ser.read(1)

            # Single-reponse packets begin with 0xA5, 0x5A
            if c == b'\xa5':
                c = self._ser.read(1)
                if c == b'\x5a':
                    len_mode_type = self._ser.read(5)
                    plen, = struct.unpack('<I', len_mode_type[:4])
                    plen &= 0x3fffffff
                    mode = (len_mode_type[3] & 0xc0) >> 6
                    _type = len_mode_type[4]

                    if mode == 0:
                        print('WARNING:  received a single-response packet during continuous mode')
                        print('plen:', plen, 'mode:', mode, 'type:', _type)
                        payload = self._ser.read(plen)
                    elif _type == 0x81 and plen == 5:
                        # Ignore the payload (plen==5) ... I guess it just jumps
                        # into continuous mode...
                        print('Scan mode started')
                    else:
                        print('WARNING:  unknown packet')
                        print('plen:', plen, 'mode:', mode, 'type:', _type)
                        payload = self._ser.read(plen)

                else:  # invalid 2nd char
                    print('parser miss (expecting 0x5a):', c)

            # Continuous-reponse packets begin with 0xAA, 0x55
            elif c == b'\xaa':
                c = self._ser.read(1)
                if c == b'\x55':
                    ct, lsn, fsa, lsa, cs = struct.unpack('<BBHHH', self._ser.read(8))
                    pc_data = self._ser.read(lsn * 2)
                    if ct & 0x01 == 0:

                        # Rotational offset
                        # (to allow for slight mis-mounting)
                        fsa = fsa + self._offset
                        lsa = lsa + self._offset
                        if fsa < 0:
                            fsa += 360 * 128
                        if lsa < 0:
                            lsa += 360 * 128

                        self.parse_pc_packet(fsa, lsa, pc_data)
                        #if fsa <= 13800 and lsa >= 13800:
                        #    nothing = self._ser.read(1)
                    else:
                        self.actual_scan_frequency = (((ct & 0xfe) >> 1) + 30) / 10.0
                        #print('freq:', self.actual_scan_frequency)

                else:  # invalid 2nd char
                    print('parser miss (expecting 0x55):', c)

            else:  # invalid 1st char
                pass
                # We always seem to get an extra byte around lsa ~= 13800... wtf...
                #print('parser miss (expecting either 0xa5 or 0xaa):', c, 'lsn:', lsn, fsa, lsa)

    def send_then_get(self, pkt):
        # empty out the RX buffer
        nothing = self._ser.read(self._ser.inWaiting())

        self._ser.write(pkt)
        c = self._ser.read(1)
        if c == b'\xa5':
            c = self._ser.read(1)
            if c == b'\x5a':
                len_mode_type = self._ser.read(5)
                plen, = struct.unpack('<I', len_mode_type[:4])
                plen &= 0x3fffffff
                mode = (len_mode_type[3] & 0xc0) >> 6
                if mode == 1:
                    print('WARNING:  received a continuous-response packet during idle mode')
                _type = len_mode_type[4]
                payload = self._ser.read(plen)
                return mode, _type, payload
            else:
                print('unknown 2nd byte:', c)
        else:
            print('unknown start byte:', c)

        print('WARNING:  didn\'t parse anything...  bytes:', self._ser.inWaiting())
        return None, None, None

    def parse_pc_packet(self, fsa: int, lsa: int, pc_data: bytes):
        '''
        Parameters
        ----------
        fsa : int, start angle *128
        lsa : int, end angle *128
        pc_data : bytes, pointcloud data, array of 2-bytes, little-endian
            usually, about 80 bytes (40 samples), I think
        '''
        pkt = struct.pack('<HH', fsa, lsa) + pc_data
        self._sock.sendto(pkt, self._DEST_ADDR)

    #def stop(self):
    #    pkt = bytes([0xa5, 0x65])
    #    self._ser.write(pkt)

    def force_stop(self):
        pkt = bytes([0xa5, 0x00])
        self._ser.write(pkt)

    def get_device_info(self):
        pkt = bytes([0xa5, 0x90])
        mode, _type, payload = self.send_then_get(pkt)
        assert mode == 0
        assert _type == 0x04
        assert len(payload) == 0x14
        model_number, firm_minor, firm_major, hard_version = struct.unpack('BBBB', payload[:4])
        print('Firmware version: %d.%d' % (firm_major, firm_minor))
        print('Hardware version: %d' % (hard_version))
        known_models = {
            100: 'TG15',
            101: 'TG30',
            102: 'TG50',
        }
        if model_number in known_models:
            print(f'Model: {model_number} ({known_models[model_number]})')
        else:
            print(f'Model: {model_number}')
        serNum = ''
        for val in payload[4:]:
            serNum += val.__str__()
        print('Serial:', serNum)

    def get_device_health(self):
        pkt = bytes([0xa5, 0x92])
        mode, _type, payload = self.send_then_get(pkt)
        assert mode == 0
        assert _type == 0x06
        assert len(payload) == 3
        status, health = struct.unpack('<BH', payload)
        if status == 0 and health == 0:
            print('Health status is good.')
        else:
            print("Problem.  status_code: %x,  error_code: %x" % (status, health))

    def scan(self):
        pkt = bytes([0xa5, 0x60])
        self._ser.write(pkt)

    def _parse_scan_frequency(self, mode, _type, payload):
        if mode == 0 and _type == 4 and len(payload) == 4:
            val, = struct.unpack('<I', payload)
            freq = val / 100.0
            return freq
        else:
            print('FAILED to parse scan frequency')

    def increase_scan_freq_01hz(self):
        '''
        Increase scan frequency by 0.1 Hz
        '''
        pkt = bytes([0xa5, 0x09])
        mode, _type, payload = self.send_then_get(pkt)
        return self._parse_scan_frequency(mode, _type, payload)

    def decrease_scan_freq_01hz(self):
        '''
        Decrease scan frequency by 0.1 Hz
        '''
        pkt = bytes([0xa5, 0x0a])
        mode, _type, payload = self.send_then_get(pkt)
        return self._parse_scan_frequency(mode, _type, payload)

    def increase_scan_freq_1hz(self):
        '''
        Increase scan frequency by 1 Hz
        '''
        pkt = bytes([0xa5, 0x0b])
        mode, _type, payload = self.send_then_get(pkt)
        return self._parse_scan_frequency(mode, _type, payload)

    def decrease_scan_freq_1hz(self):
        '''
        Increase scan frequency by 1 Hz
        '''
        pkt = bytes([0xa5, 0x0c])
        mode, _type, payload = self.send_then_get(pkt)
        return self._parse_scan_frequency(mode, _type, payload)

    def get_scan_frequency(self):
        '''
        Get the Scan Frequency on the device.  (This is the set-point, not the actual)
        '''
        pkt = bytes([0xa5, 0x0d])
        mode, _type, payload = self.send_then_get(pkt)
        return self._parse_scan_frequency(mode, _type, payload)

    def get_scan_frequency_async(self):
        '''
        Get the Scan Frequency on the device.  (This is the set-point, not the actual)
        '''
        pkt = bytes([0xa5, 0x0d])
        self._ser.write(pkt)

    def get_zero_angle_deviation(self):
        self._commandQueue.append('Zero Angle Deviation')
        pkt = bytes([0xa5, 0x93])
        self._ser.write(pkt)


if __name__ == '__main__':

    #lidar = YDLidar('/dev/ttyUSB0', 512000, ('127.0.0.1', 25811))

    SERIAL_DEVICE = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    #lidar = YDLidar(SERIAL_DEVICE, 512000, ('192.168.0.104', 25811))
    #lidar = YDLidar(SERIAL_DEVICE, 512000, ('192.168.0.116', 25811))
    lidar = YDLidar(SERIAL_DEVICE, 512000, ('192.168.1.5', 25811), -3.125)

    lidar.get_device_info()
    lidar.get_device_health()
    freq = lidar.set_desired_scan_frequency(11.5)
    print(f'Scan frequency: {freq} Hz')

    lidar.start()


