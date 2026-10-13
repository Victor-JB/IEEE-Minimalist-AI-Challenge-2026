"""Read-only full-model preview: python -m luxoiii.preview (read-only board observer optional)."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
from urllib.parse import unquote, urlparse, parse_qs
from .spatial import read_observer

DESK = Path(__file__).parent / "desk"
CAD = Path(__file__).resolve().parents[3] / "arm/cad"


class Handler(BaseHTTPRequestHandler):
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
