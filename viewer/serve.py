#!/usr/bin/env python3
"""Serve the H2AIR exterior geometry viewer for a model run folder.

Usage:
  python viewer/serve.py outputs/my_design
  python viewer/serve.py outputs/my_design --port 8765 --no-browser
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import threading
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = Path(__file__).resolve().parent


def find_cad_json(path: Path) -> Path:
    if path.is_file():
        if path.suffix.lower() != ".json":
            raise FileNotFoundError(f"Expected a cad_parameters.json file, got {path}")
        return path
    for candidate in (path / "details" / "cad_parameters.json", path / "cad_parameters.json"):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"No cad_parameters.json under {path}. Run the model first, or pass the JSON file path."
    )


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, cad_path: Path, **kwargs):
        self.cad_path = cad_path
        super().__init__(*args, directory=str(VIEWER), **kwargs)

    def do_GET(self):  # noqa: N802
        if self.path.split("?", 1)[0] in ("/cad_parameters.json", "/api/cad.json"):
            data = self.cad_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))


def main() -> int:
    ap = argparse.ArgumentParser(description="H2AIR exterior UAV geometry viewer")
    ap.add_argument(
        "run",
        nargs="?",
        default=str(ROOT / "outputs" / "my_design"),
        help="Run folder or cad_parameters.json (default: outputs/my_design)",
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    run_path = Path(args.run).expanduser().resolve()
    cad_path = find_cad_json(run_path)
    # Validate JSON early.
    doc = json.loads(cad_path.read_text(encoding="utf-8"))
    n = len(doc.get("parameters", []))
    print(f"CAD source: {cad_path} ({n} parameters, {doc.get('model_version', '?')})")

    mimetypes.add_type("application/javascript", ".js")
    mimetypes.add_type("text/css", ".css")

    handler = partial(Handler, cad_path=cad_path)
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}/"
    print(f"Viewer: {url}")
    print("Stop with Ctrl+C")

    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
