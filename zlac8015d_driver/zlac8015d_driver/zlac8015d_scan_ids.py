import argparse

from zlac8015d_driver.zlac8015d_cmd_vel import Zlac8015dBus


def main(argv=None):
    parser = argparse.ArgumentParser(description='Scan ZLAC8015D RS485 Modbus IDs.')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--start', type=int, default=1)
    parser.add_argument('--end', type=int, default=10)
    parser.add_argument('--timeout', type=float, default=0.6)
    parser.add_argument('--retries', type=int, default=2)
    args = parser.parse_args(argv)

    bus = Zlac8015dBus(args.port, args.baudrate, timeout=args.timeout, retries=args.retries)
    try:
        found = []
        for slave_id in range(args.start, args.end + 1):
            try:
                value = bus.read_registers(slave_id, 0x2001, 1)[0]
                print('id %d: responds, 2001=%d' % (slave_id, value))
                found.append(slave_id)
            except Exception:
                print('id %d: no response' % slave_id)
        if not found:
            print('No drivers responded. Check power, A/B wiring, port, baudrate, and RS485 adapter.')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
