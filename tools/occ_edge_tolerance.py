# linkrods の稜ごとの公差を OCC（FreeCAD 1.1）で読む（4-518）。
# 実行: FreeCAD の python で FreeCAD を import してから exec する（HANDOVER 4-518）。
import Part
s = Part.Shape(); s.read(r"E:/CAD-Kernel/reference/OCCT/data/step/linkrods.step")
sol = max(s.Solids, key=lambda x: len(x.Faces))
faces = sol.Faces
tols = []
for i, e in enumerate(sol.Edges):
    owners = [j for j, f in enumerate(faces) if any(e.isSame(g) for g in f.Edges)]
    tols.append((e.Tolerance, i, owners))
tols.sort(reverse=True)
print("faces", len(faces), "edges", len(sol.Edges), "vertex tol max", max(v.Tolerance for v in sol.Vertexes))
print("face tol max", max(f.Tolerance for f in faces))
for t in tols[:25]: print("%.3e edge %d faces %s" % t)
print("edges tol>1e-5:", sum(1 for t in tols if t[0] > 1e-5))
