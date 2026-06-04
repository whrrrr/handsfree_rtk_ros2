import argparse
import time

from zlac8015d_driver.zlac8015d_cmd_vel import Zlac8015dBus, int16_to_u16


REG_WATCHDOG_MS = 0x2000
REG_PARKING = 0x200C
REG_RUN_MODE = 0x200D
REG_CONTROL = 0x200E
REG_ASYNC_SYNC = 0x200F
REG_LEFT_RATED_CURRENT = 0x2033
REG_LEFT_MAX_CURRENT = 0x2034
REG_RIGHT_RATED_CURRENT = 0x2063
REG_RIGHT_MAX_CURRENT = 0x2064
REG_ACCEL_LEFT = 0x2080
REG_ACCEL_RIGHT = 0x2081
REG_DECEL_LEFT = 0x2082
REG_DECEL_RIGHT = 0x2083
REG_TARGET_SPEED_L = 0x2088
REG_TARGET_POS_L_HIGH = 0x208A
REG_MAX_RPM_LEFT = 0x208E
REG_MAX_RPM_RIGHT = 0x208F

MODE_RELATIVE_POSITION = 1
CONTROL_STOP = 0x06
CONTROL_ENABLE = 0x08
CONTROL_START_LEFT = 0x11
CONTROL_START_RIGHT = 0x12
CONTROL_START_BOTH = 0x13


def clamp_current_0p1a(value):
    return max(1, min(300, int(round(float(value) * 10.0))))


def i32_words(value):
    value = int(value)
    if value < 0:
        value = (1 << 32) + value
    value &= 0xFFFFFFFF
    return (value >> 16) & 0xFFFF, value & 0xFFFF


def configure_position(bus, driver_id, args):
    rated = clamp_current_0p1a(args.rated_a)
    max_current = clamp_current_0p1a(args.max_a)

    print('id=%d init relative-position mode' % driver_id)
    bus.write_register(driver_id, REG_WATCHDOG_MS, 0)
    bus.write_registers(driver_id, REG_TARGET_SPEED_L, [0, 0])
    bus.write_register(driver_id, REG_CONTROL, CONTROL_STOP)
    time.sleep(0.05)

    bus.write_register(driver_id, REG_PARKING, 0)
    bus.write_register(driver_id, REG_LEFT_RATED_CURRENT, rated)
    bus.write_register(driver_id, REG_LEFT_MAX_CURRENT, max_current)
    bus.write_register(driver_id, REG_RIGHT_RATED_CURRENT, rated)
    bus.write_register(driver_id, REG_RIGHT_MAX_CURRENT, max_current)

    bus.write_register(driver_id, REG_ASYNC_SYNC, 0)
    bus.write_register(driver_id, REG_RUN_MODE, MODE_RELATIVE_POSITION)
    bus.write_register(driver_id, REG_ACCEL_LEFT, args.accel_ms)
    bus.write_register(driver_id, REG_ACCEL_RIGHT, args.accel_ms)
    bus.write_register(driver_id, REG_DECEL_LEFT, args.decel_ms)
    bus.write_register(driver_id, REG_DECEL_RIGHT, args.decel_ms)
    bus.write_register(driver_id, REG_MAX_RPM_LEFT, args.max_rpm)
    bus.write_register(driver_id, REG_MAX_RPM_RIGHT, args.max_rpm)
    bus.write_register(driver_id, REG_CONTROL, CONTROL_ENABLE)
    time.sleep(0.1)


def jog_one(bus, driver_id, side, counts, settle_s):
    zero_hi, zero_lo = i32_words(0)
    count_hi, count_lo = i32_words(counts)
    if side == 'left':
        values = [count_hi, count_lo, zero_hi, zero_lo]
        start = CONTROL_START_LEFT
    elif side == 'right':
        values = [zero_hi, zero_lo, count_hi, count_lo]
        start = CONTROL_START_RIGHT
    else:
        raise ValueError('side must be left or right')

    print('id=%d %s jog %+d counts' % (driver_id, side, counts))
    bus.write_register(driver_id, REG_CONTROL, CONTROL_ENABLE)
    time.sleep(0.03)
    bus.write_registers(driver_id, REG_TARGET_POS_L_HIGH, values)
    bus.write_register(driver_id, REG_CONTROL, start)
    time.sleep(settle_s)
    bus.write_register(driver_id, REG_CONTROL, CONTROL_STOP)
    time.sleep(0.1)


def jog_both_on_driver(bus, driver_id, left_counts, right_counts, settle_s):
    left_hi, left_lo = i32_words(left_counts)
    right_hi, right_lo = i32_words(right_counts)
    print('id=%d both jog left=%+d right=%+d counts' % (driver_id, left_counts, right_counts))
    bus.write_register(driver_id, REG_CONTROL, CONTROL_ENABLE)
    time.sleep(0.03)
    bus.write_registers(driver_id, REG_TARGET_POS_L_HIGH, [left_hi, left_lo, right_hi, right_lo])
    bus.write_register(driver_id, REG_CONTROL, CONTROL_START_BOTH)
    time.sleep(settle_s)
    bus.write_register(driver_id, REG_CONTROL, CONTROL_STOP)
    time.sleep(0.1)


def stop_all(bus, ids):
    for driver_id in ids:
        try:
            bus.write_registers(driver_id, REG_TARGET_SPEED_L, [0, 0])
            bus.write_register(driver_id, REG_CONTROL, CONTROL_STOP)
        except Exception as exc:
            print('id=%d stop failed: %s' % (driver_id, exc))


def parse_ids(text):
    return [int(part.strip(), 0) for part in text.split(',') if part.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Run a small RS485 relative-position jog on four ZLAC8015D motors.'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--driver-ids', default='1,2')
    parser.add_argument('--counts', type=int, default=100)
    parser.add_argument('--id1-left-counts', type=int)
    parser.add_argument('--id1-right-counts', type=int)
    parser.add_argument('--id2-left-counts', type=int)
    parser.add_argument('--id2-right-counts', type=int)
    parser.add_argument('--max-rpm', type=int, default=5)
    parser.add_argument('--accel-ms', type=int, default=500)
    parser.add_argument('--decel-ms', type=int, default=500)
    parser.add_argument('--rated-a', type=float, default=5.0)
    parser.add_argument('--max-a', type=float, default=8.0)
    parser.add_argument('--settle-s', type=float, default=0.8)
    parser.add_argument('--timeout', type=float, default=0.6)
    parser.add_argument('--retries', type=int, default=2)
    parser.add_argument(
        '--same-time',
        action='store_true',
        help='Jog the two motors on each driver together. Default is one motor at a time.',
    )
    args = parser.parse_args(argv)

    ids = parse_ids(args.driver_ids)
    if not ids:
        raise SystemExit('--driver-ids must contain at least one id, for example 1 or 1,2')

    counts = {}
    for index, driver_id in enumerate(ids):
        if index == 0:
            left_counts = args.id1_left_counts
            right_counts = args.id1_right_counts
        elif index == 1:
            left_counts = args.id2_left_counts
            right_counts = args.id2_right_counts
        else:
            left_counts = None
            right_counts = None
        counts[(driver_id, 'left')] = left_counts if left_counts is not None else args.counts
        counts[(driver_id, 'right')] = right_counts if right_counts is not None else args.counts

    print(
        'port=%s baud=%d ids=%s counts=%d max_rpm=%d current=%.1f/%.1fA'
        % (args.port, args.baudrate, ids, args.counts, args.max_rpm, args.rated_a, args.max_a)
    )
    print('Make sure the wheels are lifted or free to move. Ctrl-C stops both drivers.')

    bus = Zlac8015dBus(args.port, args.baudrate, timeout=args.timeout, retries=args.retries)
    try:
        for driver_id in ids:
            configure_position(bus, driver_id, args)

        if args.same_time:
            for driver_id in ids:
                jog_both_on_driver(
                    bus,
                    driver_id,
                    counts[(driver_id, 'left')],
                    counts[(driver_id, 'right')],
                    args.settle_s,
                )
        else:
            for driver_id in ids:
                jog_one(bus, driver_id, 'left', counts[(driver_id, 'left')], args.settle_s)
                jog_one(bus, driver_id, 'right', counts[(driver_id, 'right')], args.settle_s)
        print('done')
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        stop_all(bus, ids)
        bus.close()


if __name__ == '__main__':
    main()
