import serial
import datetime
import logging
from pathlib import Path

COM = '/dev/ttyACM0'
RECVSIZE = 1024

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('rt-linelight')
logger.setLevel(logging.INFO)
ser = serial.Serial(COM, 912600, timeout=1)


def dump(savedir):
    f = None
    data = b''
    recieved = 0
    logger.info('Waiting for STX...')
    while True:
        data = ser.read(RECVSIZE)
        stx = data.find(b'\x02')
        if stx != -1:
            now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            logger.info(f'STX is received, create "{now}.csv"')
            f = open(savedir / f'{now}.csv', 'w')
            f.write(data[stx + 1:].decode('utf-8'))
            recieved += len(data[stx + 1:])
            break
    while True:
        data = ser.read(RECVSIZE)
        etx = data.find(b'\x03')
        if etx != -1:
            recieved += len(data[:etx])
            f.write(data[:etx].decode('utf-8'))
            f.close()
            logger.info(f'ETX is received (Total: {recieved} bytes), close "{now}.csv"')
            break
        else:
            f.write(data.decode('utf-8'))
            recieved += len(data)
            print(f'\x1b[1A\x1b[2K{recieved:>10} bytes received')


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
        ser.close()


if __name__ == '__main__':
    main()
