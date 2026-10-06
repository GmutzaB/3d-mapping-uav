"""Receive-only UDP telemetry recorder for a MAVLink router or SITL output."""
import argparse
import json
import math
from pathlib import Path
import socket
import time
from .telemetry import Receiver


def monitor(sock, receiver, duration, log, capture):
    """Record arrival times plus raw datagrams; never send a heartbeat or reply."""
    start = time.monotonic()
    next_report = start
    while True:
        now = time.monotonic()
        remaining = duration-(now-start)
        if remaining <= 0:
            break
        sock.settimeout(min(0.2,remaining))
        try:
            data, peer = sock.recvfrom(65535)
        except socket.timeout:
            data = None
        now = time.monotonic()
        if data is not None:
            capture.write(json.dumps({'received_at':now,'peer':list(peer),'hex':data.hex()})+'\n')
            capture.flush()
            receiver.feed(data,now)
        if now >= next_report:
            row = receiver.snapshot(now)
            log.write(json.dumps(row)+'\n')
            log.flush()
            print(json.dumps(row),flush=True)
            next_report = now+1
    return receiver.snapshot(time.monotonic())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=14560)
    parser.add_argument('--system-id',type=int,default=1)
    parser.add_argument('--component-id',type=int,default=1)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--output',type=Path,default=Path('outputs/telemetry'))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not 1 <= args.port <= 65535:
        parser.error('Use a positive finite duration and a port from 1 to 65535')
    receiver = Receiver(args.system_id,args.component_id)
    args.output.mkdir(parents=True,exist_ok=True)
    # Exclusive files avoid overwriting an earlier experiment.
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind,args.port))
        with (args.output/'telemetry.jsonl').open('x') as log, (args.output/'capture.jsonl').open('x') as capture:
            summary = monitor(sock,receiver,args.seconds,log,capture)
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__ == '__main__':
    main()
