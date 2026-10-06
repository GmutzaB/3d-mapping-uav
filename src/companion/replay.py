"""Deterministic synthetic health replay; creates real wire bytes, never opens a link."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from .supervisor import Supervisor, Health, VelocityPlan
from .mavlink import Encoder


def run(output):
    output.mkdir(parents=True,exist_ok=True)
    supervisor, encoder = Supervisor(), Encoder()
    events, packets = [], bytearray()
    # Explicit operator enable at start and after recovery; automatic recovery must latch.
    for step in range(61):
        now = step/10
        health = Health(now, now if not 3 <= now < 4 else 2.9, True, True, True)
        plan = VelocityPlan(now,now+.25,(.5,0,0),collision_checked=True)
        if 1 <= now < 2:
            plan = None
        decision = supervisor.tick(now,health,plan,enable=step in (0,25,45),disable=step == 60)
        packet = b''
        if decision.velocity is not None:
            packet = encoder.velocity(decision.velocity,round(now*1000),target_system=1)
            packets.extend(packet)
        events.append(dict(time=now,**asdict(decision),packet_hex=packet.hex()))
    (output/'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in events))
    (output/'commands.mavlink').write_bytes(packets)
    transitions = [row for i,row in enumerate(events) if i == 0 or row['state'] != events[i-1]['state']]
    print(json.dumps(dict(output=str(output),transitions=transitions),indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/companion_replay'))
    run(parser.parse_args().output)
