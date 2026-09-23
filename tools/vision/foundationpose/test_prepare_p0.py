"""CPU-only boundary tests; run with python3 -B -m unittest discover here."""
import contextlib
import hashlib
import http.server
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np
from PIL import Image


PREPARE = Path(__file__).with_name("prepare_p0.py")
REPO = PREPARE.parents[3]


@contextlib.contextmanager
def serve(payload, stall=False):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            if stall:
                time.sleep(0.4)
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield "http://127.0.0.1:%d/model.onnx" % server.server_port
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(PREPARE.is_file(), "CPU preparation entry point is missing")
        spec = importlib.util.spec_from_file_location("prepare_p0", PREPARE)
        self.p = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.p)

    def test_known_cad_preserves_volume_scale_and_business_origin(self):
        geometry = json.loads((REPO / "ws_robot/src/astribot_object_pose_core/models/asymmetric_union.visibility.json").read_text())
        vertices, faces, report = self.p.make_box_union(geometry)
        self.assertAlmostEqual(report["signed_volume_m3"], 0.001749, places=12)
        np.testing.assert_allclose(report["bounds_m"], [[-.09, -.05, -.04], [.145, .05, .09]])
        self.assertEqual(report["nonmanifold_edges"], 0)
        self.assertEqual(report["boundary_edges"], 0)
        self.assertEqual(report["euler_characteristic"], 2)
        center = np.array(report["mesh_center_in_object_m"])
        np.testing.assert_allclose(center, [.0275, 0, .025])
        self.assertTrue(np.allclose((np.array(vertices) - center) + center, vertices))

    def test_rejects_millimeter_geometry_without_silent_rescaling(self):
        with self.assertRaisesRegex(ValueError, "units"):
            self.p.make_box_union({"schema": "astribot.box_union/1", "units": "mm", "boxes": []})

    def test_rejects_corner_contact_nonmanifold_solid(self):
        geometry = {"schema": "astribot.box_union/1", "units": "m", "object_frame": "fixture", "boxes": [
            {"center_m": [0, 0, 0], "size_m": [1, 1, 1]},
            {"center_m": [1, 1, 1], "size_m": [1, 1, 1]}]}
        with self.assertRaisesRegex(ValueError, "manifold|connected"):
            self.p.make_box_union(geometry)

    def test_corrupt_download_never_publishes_final_file(self):
        with tempfile.TemporaryDirectory() as tmp, serve(b"<html>bad gateway</html>") as url:
            dest = Path(tmp) / "model.onnx"
            with self.assertRaisesRegex(ValueError, "SHA256|size"):
                self.p.download_atomic(url, dest, expected_sha256="0" * 64, expected_bytes=24, timeout_s=2)
            self.assertFalse(dest.exists())
            self.assertEqual(list(Path(tmp).glob("*.part")), [])

    def test_download_total_timeout_is_bounded(self):
        with tempfile.TemporaryDirectory() as tmp, serve(b"model", stall=True) as url:
            start = time.monotonic()
            with self.assertRaises((ValueError, RuntimeError)):
                self.p.download_atomic(url, Path(tmp) / "model.onnx", expected_sha256="0" * 64, expected_bytes=5, timeout_s=.1)
            self.assertLess(time.monotonic() - start, 1.5)

    def test_successful_download_then_cache_tamper_is_rejected(self):
        payload = b"small fixture payload"
        with tempfile.TemporaryDirectory() as tmp, serve(payload) as url:
            dest = Path(tmp) / "model.onnx"
            digest = hashlib.sha256(payload).hexdigest()
            result = self.p.download_atomic(url, dest, expected_sha256=digest, expected_bytes=len(payload), timeout_s=2)
            self.assertEqual(result["sha256"], digest)
            dest.write_bytes(b"altered")
            with self.assertRaisesRegex(ValueError, "SHA256|size"):
                self.p.download_atomic(url, dest, expected_sha256=digest, expected_bytes=len(payload), timeout_s=2)
            self.assertEqual(dest.read_bytes(), b"altered")

    def test_immutable_rerun_preserves_mtime_and_rejects_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "manifest.json"
            self.p.write_immutable(dest, b"first")
            before = dest.stat().st_mtime_ns
            self.p.write_immutable(dest, b"first")
            self.assertEqual(dest.stat().st_mtime_ns, before)
            with self.assertRaisesRegex(ValueError, "existing|immutable"):
                self.p.write_immutable(dest, b"different")
            self.assertEqual(dest.read_bytes(), b"first")

    def test_wrong_mask_shape_is_rejected_before_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "clear"
            dest.mkdir()
            source = REPO / "docs/evidence/grasp_pose_sim_20260921/simulation/snapshots/clear"
            for name in ["rgb.png", "depth.npz", "camera_info.json"]:
                (dest / name).write_bytes((source / name).read_bytes())
            Image.fromarray(np.full((2, 3), 255, dtype=np.uint8)).save(dest / "mask.png")
            with self.assertRaisesRegex(ValueError, "shape|dimensions"):
                self.p.validate_snapshot(dest)

    def test_algorithm_export_excludes_truth_and_records_fixture_provenance(self):
        source = REPO / "docs/evidence/grasp_pose_sim_20260921/simulation/snapshots/clear"
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            report = self.p.export_snapshot(source, output)
            algorithm = output / "algorithm_inputs" / "clear"
            self.assertEqual({x.name for x in algorithm.iterdir()}, {"rgb.png", "depth.npz", "mask.png", "camera.json"})
            text = (algorithm / "camera.json").read_text()
            self.assertNotIn("truth", text)
            self.assertNotIn("world_from", text)
            self.assertEqual(report["mask_source"], "hsv_color_fixture")
            self.assertTrue((output / "evaluation_only" / "clear" / "truth.json").is_file())


if __name__ == "__main__":
    unittest.main()
