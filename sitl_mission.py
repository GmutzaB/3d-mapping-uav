"""Launch our own local ArduCopter SITL, fly a short mission, and save evidence."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
ARDUPILOT_COMMIT = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'


def launch(sitl, output, server):
    binary = sitl/'build/sitl/bin/arducopter'
    defaults = sitl/'Tools/autotest/default_params/copter.parm'
    if not binary.is_file() or not defaults.is_file():
        raise RuntimeError('SITL is not built. Follow setup/SITL.md first.')
    revision = subprocess.check_output(['git','-C',str(sitl),'rev-parse','HEAD'],text=True).strip()
    if revision != ARDUPILOT_COMMIT:
        raise RuntimeError(f'Expected pinned ArduCopter commit {ARDUPILOT_COMMIT}; found {revision}')
    # Reserve our own listener first; the simulator connects to this exact socket.
    # There is no attach-to-vehicle or serial-port mode.
    server.bind(('127.0.0.1',0))
    server.listen(1)
    server.settimeout(.2)
    params = output/'defaults.parm'
    params.write_text(defaults.read_text()+'\n# Local SITL mission experiment\nSYSID_MYGCS 255\nFS_GCS_ENABLE 1\n')
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as rc:
        rc.bind(('127.0.0.1',0))
        rc_port = rc.getsockname()[1]
    args = [str(binary), '--model','quad','--speedup','1','--synthetic-clock',
            '--home','-35.363261,149.165230,584,353', '--defaults',str(params), '--wipe',
            '--serial0',f'tcpclient:127.0.0.1:{server.getsockname()[1]}',
            '--serial1','none','--serial2','none','--rc-in-port',str(rc_port)]
    metadata = dict(ardupilot_commit=revision, release='Copter-4.6.3',
                    binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    command=args, platform=platform.platform(), python=sys.version,
                    packages={name:version(name) for name in ['pymavlink','numpy']},
                    scope='Owned local ArduCopter SITL; default quad, not calibrated project airframe.')
    (output/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
    log = (output/'sitl.log').open('x')
    try:
        process = subprocess.Popen(args,cwd=output,stdout=log,stderr=subprocess.STDOUT)
    finally:
        log.close()
    return process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sitl-root',type=Path,default=ROOT/'external/ardupilot')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--altitude',type=float,default=3.)
    parser.add_argument('--north',type=float,default=3.)
    parser.add_argument('--east',type=float,default=2.)
    parser.add_argument('--no-visuals',action='store_true')
    args = parser.parse_args()
    from src.companion.mission import Mission, MissionError, MissionPlan, Session
    try:
        plan = MissionPlan(altitude=args.altitude,north=args.north,east=args.east)
    except ValueError as error:
        parser.error(str(error))
    output = (args.output or ROOT/'outputs/sitl'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')).resolve()
    try:
        output.mkdir(parents=True,exist_ok=False)
    except OSError as error:
        parser.error(f'Cannot create a fresh output directory: {error}')
    print(f'Mission logs: {output}',flush=True)
    process = session = mission = None
    result = dict(passed=False,plan=asdict(plan),scope='ArduCopter SITL only')
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as server:
            process = launch(args.sitl_root.resolve(),output,server)
            deadline = time.monotonic()+30
            while True:
                if process.poll() is not None:
                    raise MissionError('Simulator exited during startup; see sitl.log')
                if time.monotonic() >= deadline:
                    raise MissionError('Simulator connection timed out; see sitl.log')
                try:
                    sock,_ = server.accept()
                    break
                except socket.timeout:
                    continue
        session = Session(sock,output)
        mission = Mission(session,plan)
        result.update(mission.run())
    except (MissionError,RuntimeError,OSError,subprocess.SubprocessError,KeyboardInterrupt) as error:
        result['error'] = str(error) or 'Interrupted by user'
        print(f'Mission failed: {result["error"]}',file=sys.stderr,flush=True)
        if mission is not None:
            try:
                mission.abort(result['error'])
            except (MissionError,OSError):
                pass
    finally:
        if mission is not None:
            result.update(origin_ned=mission.origin,targets_ned=mission.targets,phases=mission.completed)
        if session is not None:
            session.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if not args.no_visuals and (output/'telemetry.csv').exists():
        from mission_visuals import export
        export(output)
    print(f'Result: {"PASS" if result["passed"] else "FAIL"}. Evidence: {output}',flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
