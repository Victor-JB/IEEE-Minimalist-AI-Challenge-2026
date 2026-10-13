"""Full-model spatial/hybrid preview: python -m luxoiii.preview (board observer optional; physical outputs require explicit activation)."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
from urllib.parse import unquote, urlparse, parse_qs
from .spatial import read_observer
from urllib.request import Request, urlopen

DESK = Path(__file__).parent / "desk"
CAD = Path(__file__).resolve().parents[3] / "arm/cad"


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path!='/control':return self.send_error(404)
        if self.headers.get('Content-Type')!='application/json':return self.send_error(415)
        origin=self.headers.get('Origin')
        if origin and origin!='http://'+self.headers.get('Host',''):return self.send_error(403)
        try:
            n=int(self.headers.get('Content-Length','0'))
            if not 0<n<=256:raise ValueError('Invalid control request')
            data=json.loads(self.rfile.read(n));action=data.get('action')
            if action not in ('enable','disable','heartbeat','stop','resume','swivel'):raise ValueError('Unknown control')
            if not self.server.observer_url:raise ConnectionError('Board observer unavailable')
            req=Request(self.server.observer_url+'/control',data=json.dumps({k:v for k,v in data.items() if k in ('action','angle')}).encode(),headers={'Content-Type':'application/json'})
            with urlopen(req,timeout=2) as response:body=response.read(4096)
            self.send_response(200)
        except Exception as exc:
            body=json.dumps(dict(ok=False,error=str(exc))).encode();self.send_response(400)
        self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
        self.end_headers();self.wfile.write(body)

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path == '/spatial':
            data = read_observer(self.server.observer_url, parse_qs(urlparse(self.path).query))
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass
            return
        if path in ("/", "/preview"):
            file, kind = DESK / "preview.html", "text/html; charset=utf-8"
        elif path == '/emotions.json':
            from .emotions import bundle
            body=json.dumps(bundle()).encode()
            self.send_response(200);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            return
        elif path == '/emotion-view.js':
            file,kind=DESK/'emotion_view.js','text/javascript'
        elif path == "/cad-view.js":
            file, kind = DESK / "cad_view.js", "text/javascript"
        elif re.fullmatch(r"/cad/[A-Za-z0-9_.-]+\.(json|stl)", path):
            file = CAD / path.rsplit("/", 1)[1]
            kind = "application/json" if file.suffix == ".json" else "model/stl"
        else:
            return self.send_error(404)
        if not file.is_file():
            return self.send_error(404)
        data = file.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument('--observer-url', default='', help='Read-only observer, usually http://127.0.0.1:18083 via SSH')
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.observer_url = args.observer_url.rstrip("/")
    print(f"luxoiii full model: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
