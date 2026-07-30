"""Binary-STL bounding box, no dependencies.

Needed because Part.Shape().read() is BREP-only and rejects .stl. The point is
to check whether the URDF's mesh asset shares a coordinate frame with the STEP
of nominally the same part - if the bboxes differ, the URDF camera pose cannot
be derived from the STEP measurements without a correction.
"""
import struct, sys

for path in sys.argv[1:]:
    with open(path, "rb") as fh:
        blob = fh.read()
    n = struct.unpack_from("<I", blob, 80)[0]
    if 84 + n * 50 != len(blob):
        print("%s: not binary STL (or truncated)" % path)
        continue

    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for i in range(n):
        base = 84 + i * 50 + 12          # skip the facet normal
        for v in range(3):
            for a in range(3):
                c = struct.unpack_from("<f", blob, base + v * 12 + a * 4)[0]
                if c < lo[a]:
                    lo[a] = c
                if c > hi[a]:
                    hi[a] = c

    print(path.rsplit("\\", 1)[-1].rsplit("/", 1)[-1])
    print("  triangles: %d" % n)
    for a, ax in enumerate("XYZ"):
        print("  %s  %10.4f .. %10.4f   len %9.4f  mid %9.4f"
              % (ax, lo[a], hi[a], hi[a] - lo[a], (lo[a] + hi[a]) / 2))
