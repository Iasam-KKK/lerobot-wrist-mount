"""Prove the STEP-measured M3 hole pair really exists in the printed/URDF mesh.

The STEP and the STL are NOT the same geometry (bboxes differ), so STEP hole
coordinates cannot simply be assumed valid in the URDF frame. But Z-max agrees
to 1 micron, which hints at a shared origin.

Test: sample points along each nominal hole axis and ask the mesh whether they
are inside solid material. A real through-hole reads OUTSIDE all along its axis,
while the material between the two holes reads INSIDE. If that pattern holds,
the two files share an origin and the STEP coordinates transfer.
"""
import xmlrpc.client

S = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

CODE = r'''
import Mesh, Part
from FreeCAD import Vector

m = Mesh.Mesh(r"{stl}")
sh = Part.Shape()
sh.makeShapeFromMesh(m.Topology, 0.05)
print("mesh facets: %d   shape solid: %s" % (m.CountFacets, sh.isClosed()))

TOL = 1e-6
def inside(p):
    return sh.isInside(Vector(*p), TOL, True)

# STEP-measured features, expressed in STEP coordinates (mm)
probes = [
    ("M3 hole A  (-5.000, y, 24.350)",  -5.000, 24.350, "VOID"),
    ("M3 hole B  ( 3.100, y, 24.350)",   3.100, 24.350, "VOID"),
    ("between holes (-0.950, y, 24.350)", -0.950, 24.350, "SOLID"),
]

for label, x, z, expect in probes:
    hits = []
    for i in range(9):
        y = -20.4 + i * 0.6          # walk +Y, into the part from the face
        hits.append("I" if inside((x, y, z)) else "o")
    got = "".join(hits)
    n_solid = got.count("I")
    verdict = "SOLID" if n_solid > 4 else "VOID"
    ok = "PASS" if verdict == expect else "**FAIL**"
    print("%-34s  %s  solid=%d/9  -> %-5s expect %-5s %s"
          % (label, got, n_solid, verdict, expect, ok))

print("")
print("(I = inside material, o = void; sampling y = -20.4 .. -15.6 step 0.6)")

# Cross-check: the servo horn square, 4 x M3 at z=5.950
print("servo horn pattern check, z=5.950, axis Z:")
for x, y in [(4.950, 4.732), (-4.950, 4.732), (4.950, -5.168), (-4.950, -5.168)]:
    col = "".join("I" if inside((x, y, 5.0 + k * 0.4)) else "o" for k in range(6))
    print("   (%+7.3f, %+7.3f)  %s" % (x, y, col))
'''

stl = (r"E:\ME\UAV\projects\02-lerobot-wrist-mount\cad\reference"
       r"\urdf-assets\print_Wrist_Roll_Follower_SO101.stl")
r = S.execute_code(CODE.format(stl=stl))
print(r.get("message", r) if isinstance(r, dict) else r)
