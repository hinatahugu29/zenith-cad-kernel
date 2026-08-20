"""書き出した歯車を読み直して、歯面がまだインボリュートの上にあるか。

`verify_showcase` で OCC が読む体積が、閉じた式から 7.1e-5 ずれていました。
カーネル自身は 1e-8 で乗っているので、原因は次のどちらかです。

1. STEP に書き出すときに形が落ちている
2. OCC の体積計算のほうが粗い

形が落ちているなら、読み直した面の上の点がインボリュートから離れます。
離れていなければ、ずれているのは形ではなく測り方です。

答えは 2 でした（2026年8月21日）:

    closed form       47935.888939
    OCC Volume        47939.307641  (rel 7.132e-05)
    tessellated 0.01     47931.931129  (rel 8.256e-05)
    tessellated 0.001    47935.743525  (rel 3.034e-06)
    tessellated 0.0001   47935.842920  (rel 9.600e-07)
    tessellated 1e-05    47935.875571  (rel 2.789e-07)
    flank deviation   7.6498e-08  over 1680 points

**同じ読み込んだ形**をこちらでテセレーションして発散定理で積み直すと、
細かくするほど閉じた式に寄っていきます。歯面の点もインボリュートから
7.6e-8 しか離れていません。形は無傷で、粗いのは OCC の `shape.Volume` の
ほうです。

これまでのショーケースは平面・円柱・円錐・トーラスばかりで、OCC はそれらを
厳密に積むので気づきませんでした。**スプライン面が主体の立体では、
`verify_showcase` の体積の列を 1e-9 級の物差しとして読んではいけません。**
あの列が見ているのは「読めて、閉じていて、だいたい合っている」ことです。

なお、歯面を測るときは平面のキャップを外すこと。キャップのパラメータ矩形は
歯車の輪郭の外まで広がっているので、外さないと 3.0e-1 という無関係な値が
出ます（一度そう出しました）。`isPartOfDomain` は当てになりませんでした。
"""

import math
import os
import sys

sys.path.append(r"C:\Program Files\FreeCAD 1.1\bin")

import FreeCAD  # noqa: E402
import Part  # noqa: E402

STEP = os.path.join("target", "showcase", "06_spur_gear_m3_z24.step")

MODULE, TEETH, ALPHA_DEG, THICKNESS, BORE = 3.0, 24, 20.0, 12.0, 8.0


def involute_of(angle):
    return math.tan(angle) - angle


def main():
    z = float(TEETH)
    alpha = math.radians(ALPHA_DEG)
    pitch_radius = MODULE * z * 0.5
    base_radius = pitch_radius * math.cos(alpha)
    tip_radius = pitch_radius + MODULE
    root_radius = min(max(pitch_radius - 1.25 * MODULE, BORE + 0.5 * MODULE), base_radius)
    half_at_base = math.pi / (2.0 * z) + involute_of(alpha)
    half_at_tip = half_at_base - involute_of(math.acos(base_radius / tip_radius))
    tip_t = math.tan(math.acos(base_radius / tip_radius))
    pitch_angle = 2.0 * math.pi / z

    area = z * (
        root_radius**2 * (math.pi / z - half_at_base)
        + base_radius**2 * tip_t**3 / 3.0
        + tip_radius**2 * half_at_tip
    )
    closed_form = area * THICKNESS

    shape = Part.Shape()
    shape.read(STEP)
    solids = shape.Solids
    print("solids            ", len(solids))
    solid = solids[0]
    print("faces             ", len(solid.Faces))
    print("valid / closed    ", solid.isValid(), solid.Shells[0].isClosed())
    print("closed form       %.6f" % closed_form)
    print("OCC Volume        %.6f  (rel %.3e)" % (
        solid.Volume, abs(solid.Volume - closed_form) / closed_form))

    # OCC 自身のテセレーションから体積を積み直す。発散定理で、三角形ごとに
    # (1/3) * 重心 . 法線 * 面積 を足す。OCC の Volume とは別の道である。
    for deviation in (1e-2, 1e-3, 1e-4, 1e-5):
        total = 0.0
        for face in solid.Faces:
            points, facets = face.tessellate(deviation)
            sign = -1.0 if face.Orientation == "Reversed" else 1.0
            for a, b, c in facets:
                pa, pb, pc = points[a], points[b], points[c]
                cross = (pb - pa).cross(pc - pa)
                total += sign * (pa + pb + pc).dot(cross) / 18.0
        print("tessellated %-8s %.6f  (rel %.3e)" % (
            deviation, total, abs(total - closed_form) / closed_form))

    # 歯面の点が、まだインボリュートの上にあるか。
    worst = 0.0
    checked = 0
    for face in solid.Faces:
        # 上下のキャップは平面で、パラメータ矩形が歯車の外まで広がっている。
        # ここで外さないと、輪郭の外の点を歯面のつもりで測ってしまう。
        if type(face.Surface).__name__ == "Plane":
            continue
        u0, u1, v0, v1 = face.ParameterRange
        for i in range(9):
            for j in range(5):
                u = u0 + (u1 - u0) * i / 8.0
                v = v0 + (v1 - v0) * j / 4.0
                if not face.isPartOfDomain(u, v):
                    continue
                p = face.valueAt(u, v)
                radius = math.hypot(p.x, p.y)
                if radius <= base_radius + 1e-6 or radius >= tip_radius - 1e-6:
                    continue
                t = math.sqrt(max((radius / base_radius) ** 2 - 1.0, 0.0))
                offset = half_at_base - (t - math.atan(t))
                angle = math.atan2(p.y, p.x)
                nearest = float("inf")
                for tooth in range(TEETH):
                    for side in (-1.0, 1.0):
                        gap = (angle - (tooth * pitch_angle + side * offset)) % (2 * math.pi)
                        if gap > math.pi:
                            gap -= 2 * math.pi
                        nearest = min(nearest, abs(gap))
                worst = max(worst, nearest * radius)
                checked += 1
    print("flank deviation   %.4e  over %d points" % (worst, checked))


main()
