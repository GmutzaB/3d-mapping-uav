"""Starts the scan server on localhost. Skipped where opening a socket isn't allowed."""
import json
import os
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import urlopen

from uav_lidar_bridge.scan_map import VoxelMap, save_scan
from uav_lidar_bridge.scan_server import make_handler


class ScanServerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = self.tmp.name
        for name, started, done in (('scan_a', '2026-11-01T10:00:00', True),
                                    ('scan_b', '2026-11-02T10:00:00', False)):
            m = VoxelMap(0.1)
            m.add([(1, 2, 3)])
            save_scan(os.path.join(root, name), m, dict(name=name, started=started, complete=done))
        os.makedirs(os.path.join(root, 'empty_folder'))
        with open(os.path.join(root, 'secret.txt'), 'w') as f:
            f.write('nope')
        try:
            self.server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(root))
        except PermissionError:
            self.skipTest('sandbox does not allow opening a local socket')
        self.server.RequestHandlerClass.log_message = lambda *a: None
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def tearDown(self):
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        self.tmp.cleanup()

    def get(self, path):
        with urlopen(self.base + path, timeout=5) as r:
            return r.status, r.headers, r.read()

    def test_list_newest_first(self):
        _, _, body = self.get('/scans')
        scans = json.loads(body)
        self.assertEqual([s['name'] for s in scans], ['scan_b', 'scan_a'])
        self.assertEqual([s['complete'] for s in scans], [False, True])
        self.assertEqual(scans[1]['url'], '/scans/scan_a/map.ply')
        self.assertGreater(scans[1]['size_bytes'], 0)

    def test_download(self):
        status, headers, body = self.get('/scans/scan_a/map.ply')
        self.assertEqual(status, 200)
        self.assertTrue(body.startswith(b'ply\n'))
        self.assertEqual(int(headers['Content-Length']), len(body))
        self.assertEqual(json.loads(self.get('/scans/scan_a')[2])['points'], 1)

    def test_nothing_outside_scan_folder(self):
        for path in ('/scans/..%2Fsecret.txt', '/scans/scan_a/../../secret.txt',
                     '/scans/%2E%2E/secret.txt', '/secret.txt', '/scans/scan_a/other.txt',
                     '/scans/empty_folder/map.ply'):
            with self.subTest(path=path), self.assertRaises(HTTPError) as e:
                self.get(path)
            self.assertEqual(e.exception.code, 404)


if __name__ == '__main__':
    unittest.main()
