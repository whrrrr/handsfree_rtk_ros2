import re
import time

from sensor_msgs.msg import NavSatStatus


FIX_TYPE_MAPPING = {
    0: ("Invalid Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    1: ("GPS Fix (SPS)", NavSatStatus.STATUS_FIX, 1.0),
    2: ("DGPS Fix", NavSatStatus.STATUS_SBAS_FIX, 0.5),
    3: ("PPS Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    4: ("RTK Fixed", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    5: ("RTK Float", NavSatStatus.STATUS_GBAS_FIX, 0.1),
    6: ("Estimated", NavSatStatus.STATUS_FIX, 5.0),
    7: ("Manual", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    8: ("Simulation", NavSatStatus.STATUS_FIX, 10000.0),
}

GNRMC_MODE_MAPPING = {
    'A': 'Autonomous Mode',
    'D': 'Differential Mode',
    'E': 'INS Mode',
    'F': 'RTK Float',
    'M': 'Manual Input Mode',
    'N': 'No Fix',
    'P': 'Precision Mode',
    'R': 'RTK Fixed',
    'S': 'Simulator Mode',
    'V': 'Invalid Mode',
}


def dms_to_decimal(dms, direction):
    try:
        if not dms:
            return None
        match = re.match(r'^(\d+)(\d\d\.\d+)$', dms)
        if not match:
            return None
        degrees = float(match.group(1))
        minutes = float(match.group(2))
        decimal = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal
    except Exception:
        return None


def parse_nmea_sentence(sentence):
    try:
        if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
            parts = sentence.split(',')
            if len(parts) < 10:
                return None
            return {
                'type': 'GNGGA',
                'latitude': dms_to_decimal(parts[2], parts[3]),
                'longitude': dms_to_decimal(parts[4], parts[5]),
                'altitude': to_float(parts[9], 0.0),
                'fix_quality': int(parts[6] or 0),
                'num_sats': int(parts[7] or 0),
                'hdop': to_float(parts[8], 99.9),
            }

        if sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
            parts = sentence.split(',')
            if len(parts) < 12:
                return None
            speed_knots = to_float(parts[7], 0.0)
            return {
                'type': 'GNRMC',
                'status': parts[2],
                'latitude': dms_to_decimal(parts[3], parts[4]),
                'longitude': dms_to_decimal(parts[5], parts[6]),
                'speed_mps': speed_knots * 0.514444,
                'cog_deg': to_float(parts[8], 0.0),
                'raw_speed_knots': speed_knots,
                'date': parts[9],
            }

        if sentence.startswith('$GNTHS') or sentence.startswith('$GPTHS'):
            parts = sentence.split(',')
            if len(parts) < 3:
                return None
            return {
                'type': 'GNTHS',
                'heading': to_float(parts[1], None),
                'valid': parts[2].startswith('A') if parts[2] else False,
            }

        if sentence.startswith('#UNIHEADINGA'):
            if ';' not in sentence:
                return None
            payload = sentence.split(';', 1)[1].split('*', 1)[0]
            parts = payload.split(',')
            if len(parts) < 4:
                return None
            return {
                'type': 'UNIHEADINGA',
                'solution_status': parts[0],
                'position_type': parts[1],
                'baseline_m': to_float(parts[2], None),
                'heading': to_float(parts[3], None),
                'pitch': to_float(parts[4], None) if len(parts) > 4 else None,
                'valid': parts[0] == 'SOL_COMPUTED' and parts[1] not in ('NONE', 'INVALID', ''),
            }

        return None
    except Exception:
        return None


def verify_nmea_checksum(sentence):
    try:
        if '*' not in sentence:
            return True
        body, checksum = sentence[1:].split('*', 1)
        value = 0
        for char in body:
            value ^= ord(char)
        return ('%02X' % value) == checksum.strip().upper()[0:2]
    except Exception:
        return True


def to_float(value, default):
    try:
        return float(value) if value not in (None, '') else default
    except Exception:
        return default


def sleep_until_stopped(stop_event, seconds):
    end = time.time() + seconds
    while not stop_event.is_set() and time.time() < end:
        time.sleep(min(0.05, end - time.time()))
