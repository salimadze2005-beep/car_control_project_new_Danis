"""TCP reachability only, NOT an FSDS RPC/GUI end-to-end test."""
import argparse
from pathlib import Path
import socket
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'shared'))
from car_control_core.fsds import validate_host


def check(host='localhost', port=41451):
    host = validate_host(host)
    try:
        with socket.create_connection((host, port), timeout=3):
            print('TCP reachable: %s:%d. Next check ROS /clock, track, odom, GO and commands.' % (host, port))
        return 0
    except OSError as error:
        print('FSDS connection failed at %s:%d: %s' % (host, port, error), file=sys.stderr)
        print('Start FSDS, check host/bind address and inbound TCP 41451 on the FSDS machine. '
              'WSL2 NAT may require the Windows gateway address; mirrored networking supports localhost.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost')
    args = parser.parse_args()
    raise SystemExit(check(args.host))
