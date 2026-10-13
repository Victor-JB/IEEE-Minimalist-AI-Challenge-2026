"""Main app launch and real HTTP proxy behavior, using a local fake observer only."""
import json
import os
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from luxoiii import __main__, preview, web


class MainLaunch(unittest.TestCase):
    def test_main_launches_the_live_web_app_without_legacy_arm_startup(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(web.sys, 'platform', 'darwin'):
            args = web.parse_args([])
        self.assertIs(__main__.main, web.main)
        self.assertEqual((args.port, args.observer_url), (8082, 'http://127.0.0.1:18083'))
        self.assertNotIn('luxoiii.app', sys.modules)

    def test_board_default_override_and_offline(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(web.sys, 'platform', 'linux'):
            self.assertEqual(web.parse_args([]).observer_url, 'http://127.0.0.1:8083')
        with patch.dict(os.environ, {'LUXOIII_OBSERVER_URL': 'http://localhost:19083/'}):
            self.assertEqual(web.parse_args([]).observer_url, 'http://localhost:19083')
            self.assertEqual(web.parse_args(['--offline']).observer_url, '')
            self.assertEqual(web.parse_args(['--observer-url', 'http://localhost:18083/']).observer_url,
                             'http://localhost:18083')

    def test_older_preview_command_stays_offline_by_default(self):
        with patch.object(preview, 'run_app') as launch:
            preview.main()
        launch.assert_called_once_with(observer_default='')


class Observer(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(dict(age_s=.05, size=[640,480], detections=[], hands=[],
                               outputs=dict(enabled=True, brightness=1.,
                                            lamp_gate=dict(reason='No faces · whole beam below horizon')))).encode()
        self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        self.server.actions.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
        body=b'{"ok": true}'
        self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers()
        self.wfile.write(body)

    def log_message(self,*args): pass


class MainHTTP(unittest.TestCase):
    def setUp(self):
        self.observer=ThreadingHTTPServer(('127.0.0.1',0),Observer)
        self.observer.actions=[]
        self.app=ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
        self.app.observer_url=f'http://127.0.0.1:{self.observer.server_port}'
        self.url=f'http://127.0.0.1:{self.app.server_port}'
        self.threads=[]
        for server in (self.observer,self.app):
            t=threading.Thread(target=server.serve_forever,daemon=True);t.start();self.threads.append(t)

    def tearDown(self):
        for server in (self.app,self.observer):server.shutdown();server.server_close()
        for t in self.threads:t.join(timeout=2)

    def get(self,path):
        with urlopen(self.url+path,timeout=2) as r:return r.read()

    def post(self,data,origin=None):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']=origin
        return urlopen(Request(self.url+'/control',data=json.dumps(data).encode(),headers=headers),timeout=2)

    def test_home_serves_current_app_and_live_binary_lamp_status(self):
        page=self.get('/').decode()
        for marker in ('Big Caslon','lampGate','stopDemo','gestureProgress','cad'):
            self.assertIn(marker,page)
        snapshot=json.loads(self.get('/spatial'))
        self.assertTrue(snapshot['fresh'])
        self.assertEqual(snapshot['outputs']['brightness'],1.)
        self.assertEqual(self.get('/preview'),self.get('/'))
        self.assertIn(b'createLuxoiiiCAD',self.get('/cad-view.js'))

    def test_controls_forward_only_approved_actions(self):
        with self.post({'action':'stop','level':1000},origin=self.url) as r:
            self.assertTrue(json.load(r)['ok'])
        self.assertEqual(self.observer.actions,[{'action':'stop'}])
        with self.assertRaises(HTTPError) as caught:self.post({'action':'move_stepper'})
        caught.exception.close()
        with self.assertRaises(HTTPError) as caught:self.post({'action':'enable'},origin='http://unrelated.invalid')
        caught.exception.close()
        self.assertEqual(len(self.observer.actions),1)

    def test_offline_cannot_forward_physical_controls(self):
        self.app.observer_url=''
        self.assertFalse(json.loads(self.get('/spatial'))['fresh'])
        with self.assertRaises(HTTPError) as caught:self.post({'action':'enable'})
        caught.exception.close()
        self.assertEqual(self.observer.actions,[])


if __name__=='__main__':unittest.main()
