from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import threading
import unittest
from unittest.mock import patch
from hand_preview.bridge_client import BridgeClient
from hand_preview.hardware_monitor import HardwareMonitor
from hand_preview.mapping import PreviewMapper


class BridgeClientTests(unittest.TestCase):
    def setUp(self):
        self.connections=0;self.posts=0;owner=self
        class Handler(BaseHTTPRequestHandler):
            protocol_version='HTTP/1.1'
            def setup(self):
                super().setup();owner.connections+=1
            def do_GET(self):
                self.send_response(200);self.send_header('Content-Length','11');self.end_headers();self.wfile.write(b'{"ok":true}')
            def do_POST(self):
                self.rfile.read(int(self.headers.get('Content-Length',0)));owner.posts+=1
                payload=b'{"error":"rejected"}'
                self.send_response(400);self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
            def log_message(self,*args):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever);self.thread.start()
        self.client=BridgeClient(port=self.server.server_port)

    def tearDown(self):
        self.client.close();self.server.shutdown();self.server.server_close();self.thread.join()

    def test_requests_reuse_socket_and_expire_idle_socket(self):
        for _ in range(4):self.assertTrue(self.client.request('GET','/status')['ok'])
        self.assertEqual(self.connections,1)
        self.client.used_at-=1
        self.client.request('GET','/status')
        self.assertEqual(self.connections,2)

    def test_failed_control_is_not_replayed(self):
        with self.assertRaises(ValueError):self.client.request('POST','/control',b'{}')
        self.assertEqual(self.posts,1)
        self.assertIsNone(self.client.connection)
        self.assertTrue(self.client.request('GET','/status')['ok'])

    def test_monitor_error_preserves_mode_and_marks_local_failure(self):
        stop=threading.Event()
        monitor=HardwareMonitor(PreviewMapper(),lambda:None,stop)
        monitor.data={'mode':'FULL_HAND_FOLLOW','control_capable':True,'motion_calls':12,'follow':{'active':True},'_at':0}
        def fail(*args):stop.set();raise TimeoutError('test timeout')
        with patch('hand_preview.hardware_monitor.BridgeClient') as client:
            client.return_value.request.side_effect=fail
            monitor.run()
        self.assertEqual(monitor.data['mode'],'FULL_HAND_FOLLOW')
        self.assertEqual(monitor.data['motion_calls'],12)
        self.assertEqual(monitor.data['connection_error_layer'],'local_bridge')
        self.assertFalse(monitor.data['follow']['active'])
