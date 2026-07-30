"""Build a FreeCAD document that shows the derived camera placement, so the
transform can be checked by eye instead of trusted on faith.

Places the official camera mount onto the official wrist part using the SAME
rotation+translation the URDF encodes, then draws the optical axis out to the
gripper TCP. If the derivation is right, the mount seats flat on the bolt face
and the axis lands on the jaws.
"""
import math, xmlrpc.client

S = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)

# ---- same measured inputs as camera_pose2.py -------------------------------
WRIST_A, WRIST_B = (-5.000, -20.718, 24.350), (3.100, -20.718, 24.350)
CAM_A, CAM_B_HOLE = (-4.000, -8.150, 18.100), (-4.000, -8.150, 10.000)
INTO_WRIST = (0.0, 1.0, 0.0)
PATTERN = [(-35.338598, -40.014256, 4.0), (-35.338598, -40.014256, 31.0),
           (-59.808909, -28.603563, 4.0), (-59.808909, -28.603563, 31.0)]
CAM_AXIS = (-0.422618261741, -0.906307787037, 0.0)
SIGN = -1                      # resolved by the TCP-visibility test
TCP_MESH = (-7.9, 0.0, 99.077)  # gripper_frame_link mapped back into mesh coords


def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def add(a, b): return tuple(x + y for x, y in zip(a, b))
def scale(a, k): return tuple(x * k for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return scale(a, 1.0 / norm(a))
def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def basis(x_axis, y_hint):
    ex = unit(x_axis)
    ey = unit(sub(y_hint, scale(ex, dot(y_hint, ex))))
    return [ex, ey, cross(ex, ey)]


cam_mid = scale(add(CAM_A, CAM_B_HOLE), 0.5)
wr_mid = scale(add(WRIST_A, WRIST_B), 0.5)
cb = basis((1, 0, 0), sub(CAM_A, CAM_B_HOLE))
wb0 = basis(INTO_WRIST, sub(WRIST_A, WRIST_B))
wb = [wb0[0], scale(wb0[1], -1), scale(wb0[2], -1)]      # flip = True

# R maps cam-mount coords -> wrist mesh coords
R = [[sum(wb[i][a] * cb[i][b] for i in range(3)) for b in range(3)] for a in range(3)]
t = sub(wr_mid, tuple(sum(R[a][b] * cam_mid[b] for b in range(3)) for a in range(3)))

pat = scale([sum(p[i] for p in PATTERN) for i in range(3)], 0.25)
pat_mesh = add(tuple(sum(R[a][b] * pat[b] for b in range(3)) for a in range(3)), t)
n_mesh = unit(tuple(sum(R[a][b] * SIGN * CAM_AXIS[b] for b in range(3)) for a in range(3)))
axis_end = add(pat_mesh, scale(n_mesh, 130.0))

REF = r"E:\ME\UAV\projects\02-lerobot-wrist-mount\cad\reference"
OUT = r"E:\ME\UAV\projects\02-lerobot-wrist-mount\cad\camera_placement_check.FCStd"

CODE = r'''
import FreeCAD as App, Part, Mesh
from FreeCAD import Vector, Matrix

for d in list(App.listDocuments()):
    if d == "camera_placement_check":
        App.closeDocument(d)
doc = App.newDocument("camera_placement_check")

# --- wrist: the part the URDF actually uses (print STL) ---
wm = Mesh.Mesh(r"{ref}\urdf-assets\print_Wrist_Roll_Follower_SO101.stl")
ws = Part.Shape(); ws.makeShapeFromMesh(wm.Topology, 0.05)
wo = doc.addObject("Part::Feature", "Wrist_Roll_Follower")
wo.Shape = ws
wo.ViewObject.ShapeColor = (0.62, 0.65, 0.70)

# --- camera mount, placed by the derived transform ---
cs = Part.Shape()
cs.read(r"{ref}\existing-cam-mounts\SO-ARM101_camera_wrist_mount.step")
cs.transformShape(Matrix({m00},{m01},{m02},{tx},
                         {m10},{m11},{m12},{ty},
                         {m20},{m21},{m22},{tz},
                         0,0,0,1))
co = doc.addObject("Part::Feature", "Camera_Mount_PLACED")
co.Shape = cs
co.ViewObject.ShapeColor = (0.95, 0.62, 0.15)

# --- optical axis, drawn from the M2 pattern centre ---
ax = doc.addObject("Part::Feature", "Optical_Axis_130mm")
ax.Shape = Part.makeLine(Vector({px},{py},{pz}), Vector({ex},{ey},{ez}))
ax.ViewObject.LineColor = (1.0, 0.1, 0.1)
ax.ViewObject.LineWidth = 4

cam = doc.addObject("Part::Feature", "Camera_Sensor_Centre")
cam.Shape = Part.makeSphere(2.5, Vector({px},{py},{pz}))
cam.ViewObject.ShapeColor = (1.0, 0.1, 0.1)

tcp = doc.addObject("Part::Feature", "Gripper_TCP")
tcp.Shape = Part.makeSphere(4.0, Vector({tcpx},{tcpy},{tcpz}))
tcp.ViewObject.ShapeColor = (0.1, 0.85, 0.2)

# What actually matters is not "does the axis hit the TCP" - at 9 deg it
# provably cannot - but "is the TCP inside the frame". So draw the cone.
import math as _m
_L = 150.0
_r = _L * _m.tan(_m.radians({vhalf}))
fov = doc.addObject("Part::Feature", "FOV_Cone_vertical")
fov.Shape = Part.makeCone(0.0, _r, _L, Vector({px},{py},{pz}), Vector({nx},{ny},{nz}))
fov.ViewObject.ShapeColor = (0.25, 0.7, 1.0)
fov.ViewObject.Transparency = 78

doc.recompute()
doc.saveAs(r"{out}")

import math
v = Vector({tcpx}-{px}, {tcpy}-{py}, {tcpz}-{pz})
n = Vector({nx},{ny},{nz})
ang = math.degrees(v.getAngle(n))
print(r"saved: {out}")
print("camera centre (mesh mm): %.3f %.3f %.3f" % ({px},{py},{pz}))
print("TCP distance            : %.2f mm" % v.Length)
print("TCP off optical axis    : %.3f deg" % ang)
print("TCP lateral offset      : %.2f mm  (axis MUST miss the 4mm ball)"
      % (v.Length*math.sin(math.radians(ang))))
print("fraction of half-frame  : %.2f  (1.0 = at the frame edge)"
      % (ang/{vhalf}))
print("mount/wrist interference: %.1f mm^3 (0 = seats cleanly)" % ws.common(cs).Volume)
'''

code = CODE.format(
    ref=REF, out=OUT,
    m00=R[0][0], m01=R[0][1], m02=R[0][2], tx=t[0],
    m10=R[1][0], m11=R[1][1], m12=R[1][2], ty=t[1],
    m20=R[2][0], m21=R[2][1], m22=R[2][2], tz=t[2],
    px=pat_mesh[0], py=pat_mesh[1], pz=pat_mesh[2],
    ex=axis_end[0], ey=axis_end[1], ez=axis_end[2],
    nx=n_mesh[0], ny=n_mesh[1], nz=n_mesh[2],
    vhalf=math.degrees(math.atan(math.tan(1.204277/2)*480/640)),
    tcpx=TCP_MESH[0], tcpy=TCP_MESH[1], tcpz=TCP_MESH[2])

r = S.execute_code(code)
print(r.get("message", r) if isinstance(r, dict) else r)
