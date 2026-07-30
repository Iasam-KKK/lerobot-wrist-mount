"""camera_link pose in gripper_link - corrected, with the orientation MEASURED.

Two things were previously assumed and are now pinned by measurement:

1. Bolt-axis direction. The Phi5.40 counterbore spans X -4.000..-2.000 and the
   Phi3.30 through-hole X -2.000..0.000, so the screw head faces -X and the
   wrist is on +X. Therefore cam +X points INTO the wrist, i.e. mesh +Y.
   (The first pass had this backwards.)

2. Optical direction. FreeCAD's cylinder .Axis carries an arbitrary sign, so it
   cannot be read as an outward normal. Both signs are enumerated and resolved
   by which one actually points at the gripper TCP.

Remaining freedom - the 180 deg flip about the bolt axis - is resolved the same
way. Every candidate is also checked for the camera sitting outside the wrist
solid, so a pose that buries the camera in the arm cannot win.
"""
import math, xmlrpc.client

MM = 1e-3

WRIST_HOLE_A = (-5.000, -20.718, 24.350)      # wrist mesh frame, mm
WRIST_HOLE_B = (3.100, -20.718, 24.350)
INTO_WRIST = (0.0, 1.0, 0.0)                  # measured, see docstring note 1

CAM_HOLE_A = (-4.000, -8.150, 18.100)         # cam-mount STEP frame, mm
CAM_HOLE_B = (-4.000, -8.150, 10.000)
CAM_PATTERN = [(-35.338598, -40.014256, 4.0), (-35.338598, -40.014256, 31.0),
               (-59.808909, -28.603563, 4.0), (-59.808909, -28.603563, 31.0)]
CAM_AXIS = (-0.422618261741, -0.906307787037, 0.0)     # sign unresolved

URDF_T = (0.0, -0.000218214, 0.000949706)     # <visual> origin, metres
R_URDF = [(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)]   # rpy = (-pi,0,0)

TCP = (-0.0079, -0.000218121, -0.0981274)     # gripper_frame_link in gripper_link
HFOV, W, H = 1.204277, 640, 480


def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def add(a, b): return tuple(x + y for x, y in zip(a, b))
def scale(a, k): return tuple(x * k for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return scale(a, 1.0 / norm(a))
def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def matvec(M, v): return tuple(dot(r, v) for r in M)


def basis(x_axis, y_hint):
    ex = unit(x_axis)
    ey = unit(sub(y_hint, scale(ex, dot(y_hint, ex))))
    return [ex, ey, cross(ex, ey)]


cam_mid = scale(add(CAM_HOLE_A, CAM_HOLE_B), 0.5)
wr_mid = scale(add(WRIST_HOLE_A, WRIST_HOLE_B), 0.5)
pat = scale([sum(p[i] for p in CAM_PATTERN) for i in range(3)], 0.25)

cam_B = basis((1, 0, 0), sub(CAM_HOLE_A, CAM_HOLE_B))
wr_B = basis(INTO_WRIST, sub(WRIST_HOLE_A, WRIST_HOLE_B))

d_local = tuple(dot(sub(pat, cam_mid), cam_B[i]) for i in range(3))
n_local = tuple(dot(CAM_AXIS, cam_B[i]) for i in range(3))

half_h = HFOV / 2.0
half_v = math.atan(math.tan(half_h) * H / W)

S = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)
PROBE = r'''
import Mesh, Part
from FreeCAD import Vector
m = Mesh.Mesh(r"E:\ME\UAV\projects\02-lerobot-wrist-mount\cad\reference\urdf-assets\print_Wrist_Roll_Follower_SO101.stl")
sh = Part.Shape(); sh.makeShapeFromMesh(m.Topology, 0.05)
pts = {pts}
print(",".join("1" if sh.isInside(Vector(*p), 1e-6, True) else "0" for p in pts))
'''

cands = []
for flip in (False, True):
    wb = [wr_B[0], scale(wr_B[1], -1 if flip else 1), scale(wr_B[2], -1 if flip else 1)]
    p_mesh = add(wr_mid, tuple(sum(d_local[i]*wb[i][a] for i in range(3)) for a in range(3)))
    for sgn in (+1, -1):
        n_mesh = unit(tuple(sum(sgn*n_local[i]*wb[i][a] for i in range(3)) for a in range(3)))
        p_link = add(matvec(R_URDF, scale(p_mesh, MM)), URDF_T)
        n_link = matvec(R_URDF, n_mesh)
        v = unit(sub(TCP, p_link))
        cands.append(dict(flip=flip, sgn=sgn, p_mesh=p_mesh, p_link=p_link, n_link=n_link,
                          ang=math.degrees(math.acos(max(-1, min(1, dot(v, n_link))))),
                          dist=norm(sub(TCP, p_link)) * 1000))

pts = [[round(c, 4) for c in cd["p_mesh"]] for cd in cands]
res = S.execute_code(PROBE.format(pts=repr(pts)))
msg = res.get("message", "") if isinstance(res, dict) else str(res)
flags = msg.strip().split("Output:")[-1].strip().split(",")
for cd, f in zip(cands, flags):
    cd["buried"] = (f.strip() == "1")

print("FOV half-angles: h +-%.2f deg  v +-%.2f deg\n" % (math.degrees(half_h), math.degrees(half_v)))
print("flip  sgn   camera xyz (gripper_link, m)          TCP off-axis   dist   buried?")
for cd in cands:
    print("%-5s %+d   (%+.6f, %+.6f, %+.6f)  %6.2f deg  %6.1fmm  %s"
          % (cd["flip"], cd["sgn"], cd["p_link"][0], cd["p_link"][1], cd["p_link"][2],
             cd["ang"], cd["dist"], "YES - inside arm" if cd["buried"] else "no"))

ok = [c for c in cands if not c["buried"]]
best = min(ok, key=lambda c: c["ang"])
n = best["n_link"]
pitch = math.asin(-n[2])
yaw = math.atan2(n[1] / math.cos(pitch), n[0] / math.cos(pitch))
print("\n=== SELECTED (not buried, smallest TCP off-axis angle) ===")
print("  flip=%s  axis_sign=%+d" % (best["flip"], best["sgn"]))
print("  xyz  = %.7f %.7f %.7f" % best["p_link"])
print("  rpy  = 0 %.9f %.9f   (pitch %.3f deg, yaw %.3f deg)"
      % (pitch, yaw, math.degrees(pitch), math.degrees(yaw)))
print("  optical axis = (%+.6f, %+.6f, %+.6f)" % n)
print("  TCP %.2f deg off-axis at %.1f mm  -> %s"
      % (best["ang"], best["dist"],
         "INSIDE FOV" if best["ang"] < math.degrees(half_v) else "OUTSIDE FOV"))
print("  tilt off jaw axis = %.4f deg" % math.degrees(math.acos(abs(n[2]))))
