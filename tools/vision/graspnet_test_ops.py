#!/usr/bin/env python3
"""Compatibility tests use literal expectations from upstream CUDA semantics."""
import unittest
import numpy as np
import torch
import graspnet_torch_ops as ops


def cuda_fps_reference(points, samples):
    """Literal CPU interpretation of pinned sampling_gpu.cu's lane/tree loops."""
    batch, count, _ = points.shape
    threads = min(512, 1 << (count.bit_length() - 1))
    output = np.zeros((batch, samples), dtype=np.int64)
    for b in range(batch):
        temporary = np.full(count, 1.e10, dtype=np.float32)
        old = 0
        for sample in range(1, samples):
            best = np.full(threads, -1., dtype=np.float32)
            best_index = np.zeros(threads, dtype=np.int64)
            for lane in range(threads):
                for index in range(lane, count, threads):
                    value = points[b, index]
                    if np.sum(value * value, dtype=np.float32) <= 1.e-3:
                        continue
                    delta = value - points[b, old]
                    distance = min(np.sum(delta * delta, dtype=np.float32), temporary[index])
                    temporary[index] = distance
                    if distance > best[lane]:
                        best[lane], best_index[lane] = distance, index
            offset = threads // 2
            while offset:
                for lane in range(offset):
                    if best[lane + offset] > best[lane]:
                        best[lane] = best[lane + offset]
                        best_index[lane] = best_index[lane + offset]
                offset //= 2
            old = best_index[0]
            output[b, sample] = old
    return output


class PointOperatorsTest(unittest.TestCase):
    def test_ball_first_indices_strict_radius_and_padding(self):
        points = torch.tensor([[[0., 0., 1.], [.1, 0., 1.], [.2, 0., 1.], [.01, 0., 1.]]])
        got = ops.ball_query(.1, 4, points, points[:, :1])
        self.assertEqual(got.tolist(), [[[0, 3, 0, 0]]])

    def test_cylinder_uses_rotated_x_axis_and_strict_endpoints(self):
        points = torch.tensor([[[0., 0., 1.], [0., .02, 1.], [0., .04, 1.], [.04, 0., 1.]]])
        rot = torch.tensor([[[0., -1., 0., 1., 0., 0., 0., 0., 1.]]])
        got = ops.cylinder_query(.03, -.01, .04, 4, points, points[:, :1], rot)
        self.assertEqual(got.tolist(), [[[0, 1, 0, 0]]])

    def test_no_neighbors_uses_zero_like_upstream_initialized_output(self):
        points = torch.tensor([[[0., 0., 1.], [0., 0., 2.]]])
        got = ops.ball_query(.1, 2, points, points[:, :1] + 10)
        self.assertEqual(got.tolist(), [[[0, 0]]])

    def test_fps_starts_zero_then_farthest_and_skips_origin(self):
        points = torch.tensor([[[0., 0., 1.], [0., 0., 0.], [0., 0., 3.], [0., 0., 1.5]]])
        self.assertEqual(ops.furthest_point_sample(points, 3).tolist(), [[0, 2, 3]])

    def test_fps_ties_follow_cuda_thread_lane_order(self):
        points = torch.zeros((1, 513, 3))
        points[:, :, 2] = 1.
        points[:, 1, 0] = 1.
        points[:, 512, 0] = -1.
        self.assertEqual(ops.furthest_point_sample(points, 2).tolist(), [[0, 512]])

    def test_fps_ties_follow_left_subtree_not_lowest_thread(self):
        # sampling_gpu.cu reduces offset 256,128,...,1. Its left subtree
        # contains lane 256 before lane 1, despite lane 1's lower index.
        points = torch.zeros((1, 513, 3))
        points[:, :, 2] = 1.
        points[:, 1, 0] = 1.
        points[:, 256, 0] = -1.
        self.assertEqual(ops.furthest_point_sample(points, 2).tolist(), [[0, 256]])

    def test_fps_left_subtree_for_every_cuda_block_size(self):
        for threads in (4, 8, 16, 32, 64, 128, 256, 512):
            with self.subTest(threads=threads):
                points = torch.zeros((1, threads + 1, 3))
                points[:, :, 2] = 1.
                points[:, 1, 0] = 1.
                points[:, threads // 2, 0] = -1.
                self.assertEqual(ops.furthest_point_sample(points, 2).tolist(),
                                 [[0, threads // 2]])

    def test_fps_matches_pinned_kernel_reference_on_tie_rich_batches(self):
        devices = ['cpu'] + (['cuda'] if torch.cuda.is_available() else [])
        rng = np.random.default_rng(391)
        for count in (1, 2, 3, 7, 8, 9, 31, 64, 127, 255, 512, 513, 1025):
            # Integer coordinates make distance ties exact, independent of FMA.
            values = rng.integers(-2, 3, size=(2, count, 3)).astype(np.float32)
            values[:, :, 2] += 3
            if count > 2:
                values[:, 2] = 0  # CUDA skips near-origin points.
            expected = cuda_fps_reference(values, 12)
            for device in devices:
                with self.subTest(count=count, device=device):
                    actual = ops.furthest_point_sample(torch.from_numpy(values).to(device), 12)
                    np.testing.assert_array_equal(actual.cpu().numpy(), expected)

    def test_fps_all_excluded_points_retain_zero_index(self):
        self.assertEqual(ops.furthest_point_sample(torch.zeros((2, 9, 3)), 4).tolist(),
                         [[0, 0, 0, 0], [0, 0, 0, 0]])

    def test_group_interpolate_and_three_nearest(self):
        features = torch.tensor([[[10., 20., 30., 40.]]])
        idx = torch.tensor([[[2, 0, 1]]])
        self.assertEqual(ops.grouping_operation(features, idx).tolist(), [[[[30., 10., 20.]]]])
        weights = torch.tensor([[[.5, .25, .25]]])
        self.assertEqual(ops.three_interpolate(features, idx, weights).item(), 22.5)
        known = torch.tensor([[[0., 0., 0.], [1., 0., 0.], [3., 0., 0.], [5., 0., 0.]]])
        distance, indices = ops.three_nn(torch.tensor([[[.25, 0., 0.]]]), known)
        self.assertEqual(indices.tolist(), [[[0, 1, 2]]])
        self.assertTrue(torch.allclose(distance, torch.tensor([[[.25, .75, 2.75]]])))


if __name__ == '__main__':
    unittest.main()
