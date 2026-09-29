#!/usr/bin/env python3
"""Deterministic union-of-boxes CAD surface in metres; no simulator inputs."""
import argparse
import math
from pathlib import Path

BOXES = [((0., 0., 0.), (.18, .10, .08)),
         ((.035, .015, .065), (.070, .060, .050)),
         ((.115, -.020, -.005), (.060, .045, .040))]


def points(boxes=BOXES, step=.006):
    for bi, (center, size) in enumerate(boxes):
        for axis in range(3):
            u, v = (axis+1) % 3, (axis+2) % 3
            nu, nv = max(2, math.ceil(size[u]/step)), max(2, math.ceil(size[v]/step))
            for sign in (-1, 1):
                for i in range(nu):
                    for j in range(nv):
                        p = list(center)
                        p[axis] += sign*size[axis]/2
                        p[u] += ((i+.5)/nu-.5)*size[u]
                        p[v] += ((j+.5)/nv-.5)*size[v]
                        if any(all(abs(p[k]-other_center[k]) < other_size[k]/2+1e-8
                                   for k in range(3))
                               for oi, (other_center, other_size) in enumerate(boxes) if oi != bi):
                            continue
                        normal = [0., 0., 0.]
                        normal[axis] = float(sign)
                        yield p + normal


def write(path, rows):
    with Path(path).open('w') as stream:
        stream.write('# x y z nx ny nz; metres, object frame; outward union-surface normals\n')
        for row in rows:
            stream.write(' '.join(f'{value:.9f}' for value in row) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--box-only', action='store_true')
    args = parser.parse_args()
    write(args.output, points(BOXES[:1] if args.box_only else BOXES))


if __name__ == '__main__':
    main()
