"""Drive FreeCAD's XML-RPC to measure SO-101 wrist geometry from official STEP.

Reads shapes directly with Part.Shape().read() - no document, no GUI clutter.
Extracts every cylindrical face (radius, axis, centre) since bolt holes are
cylinders; that is far more reliable than eyeballing a render.
"""
import xmlrpc.client, sys

S = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

PROBE = r'''
import Part
from collections import defaultdict

path = r"{path}"
sh = Part.Shape()
sh.read(path)

bb = sh.BoundBox
print("FILE      :", path.rsplit("\\", 1)[-1])
print("BBOX mm   : X %.3f..%.3f (%.3f)  Y %.3f..%.3f (%.3f)  Z %.3f..%.3f (%.3f)"
      % (bb.XMin, bb.XMax, bb.XLength, bb.YMin, bb.YMax, bb.YLength,
         bb.ZMin, bb.ZMax, bb.ZLength))
print("VOLUME    : %.1f mm^3   FACES: %d   SOLIDS: %d"
      % (sh.Volume, len(sh.Faces), len(sh.Solids)))

# ---- cylindrical faces, grouped by radius -------------------------------
groups = defaultdict(list)
for f in sh.Faces:
    s = f.Surface
    if s.TypeId == "Part::GeomCylinder":
        groups[round(s.Radius, 3)].append((s.Center, s.Axis, f.Area))

print("")
print("CYLINDRICAL FACES BY RADIUS  (bolt holes are cylinders)")
print("  radius   dia    n   axis(dominant)      centres (x, y, z)")
for r in sorted(groups):
    items = groups[r]
    ax = items[0][1]
    axs = "(%.2f, %.2f, %.2f)" % (ax.x, ax.y, ax.z)
    print("  %6.3f  %6.3f  %3d  %-18s" % (r, r * 2, len(items), axs))
    if len(items) <= 26:
        for c, a, area in items:
            print("           centre (%9.3f, %9.3f, %9.3f)  axis (%5.2f,%5.2f,%5.2f)  A=%.1f"
                  % (c.x, c.y, c.z, a.x, a.y, a.z, area))
print("=" * 78)
'''


def probe(path):
    r = S.execute_code(PROBE.format(path=path))
    msg = r.get("message", str(r)) if isinstance(r, dict) else str(r)
    print(msg)
    sys.stdout.flush()


for p in sys.argv[1:]:
    probe(p)
