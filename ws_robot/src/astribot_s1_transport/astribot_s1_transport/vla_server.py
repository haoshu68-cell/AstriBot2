"""Inference-only adapter host: no ROS imports, world creation or actuator access."""
import argparse
import importlib
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .vla_contract import VERSION
from .vla_policy import ReferencePolicy


def make_server(host, port, adapter):
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 32 * 1024 * 1024:
                    raise ValueError('BODY_SIZE')
                endpoint = self.path.lstrip('/')
                if endpoint not in ('capabilities', 'reset', 'infer', 'feedback', 'cancel', 'close'):
                    raise ValueError('ENDPOINT')
                self.connection.settimeout(10.)
                packet = json.loads(self.rfile.read(size))
                if not isinstance(packet, dict) or packet.get('schema') != VERSION:
                    raise ValueError('SCHEMA')
                body = json.dumps(adapter.call(endpoint, packet), allow_nan=False).encode()
                self.send_response(200)
            except Exception as error:
                body = json.dumps(dict(error=str(error))).encode()
                self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass  # Inference packets/images are recorded by the task, not stdout.
    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--factory', help='Trusted Python module:factory returning PolicyAdapter; default is reference fixture.')
    args = parser.parse_args()
    if args.factory:
        module, name = args.factory.split(':', 1)
        adapter = getattr(importlib.import_module(module), name)()
    else:
        adapter = ReferencePolicy()
    server = make_server(args.host, args.port, adapter)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


if __name__ == '__main__':
    main()
