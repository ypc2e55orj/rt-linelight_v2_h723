import serial
import datetime
from pathlib import Path

COM = '/dev/ttyACM0'
RECVSIZE = 1024

ser = serial.Serial(COM, 912600, timeout=1)

def dump(savedir):
    file = None
    filename = ''
    buffer = b''
    total = 0
    while True:
        buffer = ser.read(RECVSIZE)
        stx = buffer.find(b'\x02')
        if stx == -1:
            print(buffer.decode('utf-8'), end='')
        else:
            print(buffer[:stx].decode('utf-8'))
            filename = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.csv'
            print(f'\x1b[32mSTX is received, create "{filename}"\n')
            file = open(savedir / filename, 'w')
            file.write(buffer[stx + 1:].decode('utf-8'))
            total += len(buffer[stx + 1:])
            break
    while True:
        buffer = ser.read(RECVSIZE)
        etx = buffer.find(b'\x03')
        if etx != -1:
            total += len(buffer[:etx])
            file.write(buffer[:etx].decode('utf-8'))
            file.close()
            print(f'ETX is received (Total: {total} bytes), close "{filename}"\x1b[m\n')
            break
        else:
            file.write(buffer.decode('utf-8'))
            total += len(buffer)
            print(f'\x1b[1A\x1b[2K{total:>10} bytes received')


def main():
    try:
        savedir = Path.home() / 'rt-linelight'
        if savedir.exists() is False:
            savedir.mkdir()
        while (True):
            dump(savedir)
    except KeyboardInterrupt:
        pass
    finally:
        print('\x1b[m')
        ser.close()


if __name__ == '__main__':
    main()
