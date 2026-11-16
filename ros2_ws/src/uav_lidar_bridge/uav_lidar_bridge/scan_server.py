"""Read-only HTTP server that lets the phone app download scans after landing.

Plain Python standard library; ROS is not needed to run it.

    GET /health                    {"ok": true}
    GET /scans                     list of scan.json summaries, newest first
    GET /scans/<name>              one scan.json
    GET /scans/<name>/map.ply      the point cloud (binary PLY, ENU meters)

`complete: false` in a summary means an autosave from a flight still in progress
(or one that ended without a clean disarm). Anyone on the same network can read the
scans; it serves nothing outside the scan folder and accepts no uploads.
"""
import argparse
import json
import os
import re
import shutil
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
FILES = {'map.ply': 'application/octet-stream', 'scan.json': 'application/json'}


def list_scans(scan_dir):
    scans = []
    try:
        names = os.listdir(scan_dir)
    except FileNotFoundError:
        return scans
    for name in names:
        meta_path = os.path.join(scan_dir, name, 'scan.json')
        if not NAME.match(name) or not os.path.isfile(meta_path):
            continue
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except (OSError, ValueError):
            continue
        meta['name'] = name
        ply = os.path.join(scan_dir, name, 'map.ply')
        meta['size_bytes'] = os.path.getsize(ply) if os.path.isfile(ply) else 0
        meta['url'] = f'/scans/{name}/map.ply'
        scans.append(meta)
    scans.sort(key=lambda m: (m.get('started', ''), m['name']), reverse=True)
    return scans


def make_handler(scan_dir):
    root = os.path.realpath(scan_dir)

    class Handler(BaseHTTPRequestHandler):
        server_version = 'uav-scan-server/1'

        def do_GET(self):
            parts = [unquote(p) for p in urlparse(self.path).path.split('/') if p]
            if parts == ['health']:
                return self.send_json({'ok': True})
            if parts == ['scans']:
                return self.send_json(list_scans(root))
            if len(parts) in (2, 3) and parts[0] == 'scans' and NAME.match(parts[1]):
                file = parts[2] if len(parts) == 3 else 'scan.json'
                if file in FILES:
                    return self.send_file(os.path.join(root, parts[1], file), FILES[file])
            self.send_error(HTTPStatus.NOT_FOUND)

        def send_json(self, obj):
            body = json.dumps(obj, indent=2).encode()
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

        def send_file(self, path, content_type):
            path = os.path.realpath(path)
            if os.path.dirname(os.path.dirname(path)) != root or not os.path.isfile(path):
                return self.send_error(HTTPStatus.NOT_FOUND)
            with open(path, 'rb') as f:  # opened before replace() can swap it: stays whole
                size = os.fstat(f.fileno()).st_size
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(size))
                self.send_header('Access-Control-Allow-Origin', '*')
                if path.endswith('.ply'):
                    name = os.path.basename(os.path.dirname(path))
                    self.send_header('Content-Disposition', f'attachment; filename="{name}.ply"')
                self.end_headers()
                shutil.copyfileobj(f, self.wfile)

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--dir', default='~/scans', help='scan folder (default ~/scans)')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    args, _ros_args = parser.parse_known_args(argv)  # ignore --ros-args from launch
    scan_dir = os.path.expanduser(args.dir)
    os.makedirs(scan_dir, exist_ok=True)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(scan_dir))
    print(f'Serving {scan_dir} on http://{args.host}:{args.port}/scans', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
