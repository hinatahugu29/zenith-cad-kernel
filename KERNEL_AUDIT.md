# Zenith Kernel Audit

Last updated: 2026-08-19

This note captures implementation risks found while checking the Rust kernel for the OCCT replacement path.

## Immediate Finding

### Cylinder and other curved planar caps

Status: fixed for current planar cap tessellation path; broader exact-trim work remains.

The cylinder builder already creates the side surface and cap boundaries with rational NURBS quarter arcs. The visible square cap problem came from tessellation, not from the primitive topology itself.

Before this pass, planar face tessellation used `face.outer_wire.vertices()`, which returns only the start vertex of each oriented edge. A cylinder cap has four arc edges, so the cap became a four-point polygon during mesh generation. The same failure mode affects:

- cylinder top and bottom caps
- cone top and bottom caps
- drilled hole top and bottom faces
- rounded/filleted box caps
- any future sketch profile that contains arcs or splines on a planar face

Changes made:

- Added orientation-aware curve sampling on `OrientedEdge`.
- Added `Wire::sample_points()` for display/tessellation boundary extraction.
- Updated planar face tessellation to triangulate sampled curve boundaries instead of topology vertices.
- Added a regression test asserting cylinder caps sample more than four points.
- Added numeric cylinder checks proving each side NURBS patch stays on the analytic cylinder radius and cap p-curves share the same circular edge geometry instead of drifting into a square boundary.

Relevant files:

- `crates/zenith_algo/src/primitive.rs`
- `crates/zenith_topo/src/edge.rs`
- `crates/zenith_topo/src/wire.rs`
- `crates/zenith_tess/src/surface_tess.rs`
- `crates/zenith_algo/tests/modeling_test.rs`

Verification note: test execution is currently blocked by repeated Windows file-lock errors while Cargo writes dependency artifacts. `cargo fmt --all` completed.

## High-Risk Areas Before OCCT Replacement

### 1. Boolean operations are mesh booleans, not B-Rep booleans

`BooleanEngine::boolean_solids()` tessellates solids and then filters triangles by centroid inside/outside tests. It does not compute exact surface/surface intersections, split edges, create new trimming curves, or return a valid B-Rep solid.

Impact:

- acceptable only as a preview or temporary mesh result
- not suitable for exact CAD editing
- cannot reliably feed STEP export, feature history, fillet/chamfer, shelling, or persistent topology

Replacement target:

- implement exact B-Rep boolean pipeline: broad phase, curve/surface intersection, edge/face splitting, classification, shell reconstruction, validation
- keep mesh boolean only as preview/fallback

Current hardening:

- Added `BooleanEngine::boolean_solids_mesh_preview()` to label the current tessellation/ray-cast path as display-only mesh output.
- Added `BooleanEngine::boolean_solids_exact()` as the future exact B-Rep entry point; it validates inputs and fails explicitly instead of silently returning a mesh.
- Added a regression test proving exact B-Rep boolean does not fall back to preview mesh output.
- Added a `BrepIntersectionBuilder` scaffold that enumerates face-pair support intersections for exact boolean pre-processing.
- Added plane/plane support-intersection candidates with line and coincident classifications, plus tests proving candidate lines lie on both source planes.
- Added face-boundary AABB broad phase filtering before support-surface intersection, avoiding obvious disjoint face pairs in the future exact boolean pipeline.
- Plane/plane line candidates now carry a finite segment clipped to the overlap of both face-boundary AABBs, giving the future split stage a bounded starting interval.
- Plane/plane line candidates are now further clipped against each planar face's outer p-curve trim polygon, so the future split stage receives a trim-bounded segment rather than only an AABB-bounded segment.
- Trim-bounded plane/plane line candidates can now be promoted into linear topology `Edge` candidates while preserving source face indices for the future split stage.
- Added an initial planar face split step for linear intersection edges crossing an outer-loop-only planar face, producing two new p-curve-backed planar faces for the future exact boolean reconstruction stage.
- Added collection of paired planar split candidates, so an intersection edge can now produce split source faces on both boolean operands before classification.
- Added initial split-face classification against a solid as `Inside`, `Outside`, or `Boundary` using a representative face point, current tessellated-solid ray casting, and mesh boundary distance. This is a replaceable approximation until exact B-Rep point-in-solid classification is implemented.
- Added boolean-operation face-piece selection for classified split faces, including `Difference` handling that keeps A outside pieces and reversed B inside pieces before future shell reconstruction.
- Added an exact boolean preparation report that exposes face-pair, intersection-edge, planar-split, classified-split, and selected-face-piece counts before the final shell reconstruction stage exists.
- Added selected-face stitching diagnostics that count unmatched, non-manifold, and same-direction edge uses before attempting to build a result shell.
- Added a guarded reconstruction helper that turns stitchable selected face pieces into a validated `Solid`, giving the exact boolean path a final shell-construction gate for limited closed cases.
- Added sequential multi-edge splitting for outer-loop-only planar faces, allowing one face to be cut by more than one linear intersection edge before classification.
- Added face-index grouped batch splitting from collected intersection edges, so multiple intersecting opponent faces can split the same planar source face before boolean classification.
- Added an integrated selection pass that feeds batch-split face fragments, or original unsplit faces, through classification, boolean-operation selection, and stitching diagnostics.
- The exact boolean entry point can now return a validated B-Rep `Solid` for guarded limited cases: identical-object union/intersection, or future selected face pieces that already stitch into a closed manifold.
- Added a guarded planar cap builder for closed intersection edge loops, ordering unordered/reversed linear edges into a closed wire before creating a p-curve-backed planar cutting face.
- Added extraction of multiple closed intersection edge loops from unordered edge sets, then generation of planar cutting cap faces for each valid loop.
- Added a shell-assembly integration pass that appends generated planar cap faces to selected boolean face pieces and reports stitching diagnostics both before and after cap insertion.
- Exact boolean now attempts to build a validated B-Rep solid from cap-augmented selected face assemblies when the stitched result is closed, and cap insertion chooses the orientation that minimizes stitching errors.
- Added a B-Rep translation utility for solids/shells/faces/curves/surfaces and covered exact contained intersections, so non-intersecting face-pair cases can still return a validated B-Rep result when the boolean selection is closed.
- Exact boolean now handles no-intersection containment/disjoint cases explicitly: contained union/intersection and disjoint difference return existing B-Rep solids, contained subtraction returns an `inner_shells` cavity solid, and empty intersections or disjoint multi-body unions report dedicated unsupported-result errors.
- Solid tessellation now flips `inner_shells` while merging meshes so cavity mass properties subtract from the outer shell, and STEP export emits `BREP_WITH_VOIDS` with `ORIENTED_CLOSED_SHELL` entries for inner-shell cavities.
- STEP import now recognizes `BREP_WITH_VOIDS`, resolves outer and oriented inner closed shells, and round-trips contained-difference cavity solids with valid topology and subtractive mass properties.
- Added a guarded exact intersection path for axis-aligned rectangular boxes that returns the overlapping volume as a validated B-Rep box, covering the first partial-overlap solid-producing boolean case.
- Extended the guarded axis-aligned box path to return exact B-Rep boxes for single-box unions and edge-trimming differences when the result can be represented without general face graph assembly.
- Added connected-cell orthogonal B-Rep assembly for axis-aligned box booleans, allowing L-shaped unions and corner-notch differences to return validated planar solids with checked volumes.
- Moved guarded axis-aligned and orthogonal box boolean assembly into a dedicated module, keeping the main exact boolean engine focused on orchestration and general B-Rep preparation.
- Added a first guarded curved-surface intersection case: horizontal planes can now intersect recognized cylinder-side NURBS patches and produce circular arc edge candidates for future curved-face splitting.
- Planar face splitting can now preserve a curved split edge when its endpoints lie on the face boundary, enabling arc-bounded planar fragments instead of forcing every split loop back into straight polygon edges.
- Batch planar splitting now accepts those guarded curved split edges on the planar operand, and tessellation keeps the arc boundary sampled instead of collapsing the fragment into a straight chord.
- Exact boolean preparation reports now include skipped batch split counts, making curved-intersection gaps visible instead of hiding them behind a generic unsupported result.
- Recognized cylinder-side NURBS faces can now be split by a horizontal circular arc edge into upper and lower NURBS face fragments with valid p-curves, allowing plane-cylinder split candidates to carry fragments for both operands.
- Slab-vs-cylinder exact boolean preparation now reaches cylinder-side split counts from real solid operands, proving the curved split path is wired through the public preparation report instead of only isolated helper tests.
- Added a guarded exact B-Rep intersection for a Z-axis cylinder fully covered in XY by a horizontal slab, returning a valid shortened cylinder solid with NURBS side faces and STEP-exportable topology.
- Extended the guarded cylinder-slab exact path to support end-trimming differences when the slab removes the top or bottom of the cylinder, while explicitly rejecting middle cuts that require compound multi-solid results.
- Added a multi-solid exact boolean result API so middle cylinder-slab differences can return two valid disjoint cylinder solids while the legacy single-solid API reports that callers should use the multi-solid path.
- STEP export can now write multi-solid exact boolean results as multiple B-Rep representation items in one file, so compound outputs do not have to collapse back to the legacy single-solid API at the first interchange boundary.
- STEP import now exposes multi-solid file/string APIs, resolves ordered B-Rep items from `ADVANCED_BREP_SHAPE_REPRESENTATION`, and round-trips the two-cylinder middle slab difference with NURBS faces, topology validation, z-spans, and volume checks intact.
- A plane oblique to the cylinder axis now produces its exact elliptical section. Projecting the base section arc along the axis onto the cutting plane is an affine map, and rational NURBS are closed under affine maps, so the ellipse comes out with the same degree, knots, and weights as the circle - no approximation and no new curve class. The arc is accepted only when its projected control points stay inside the patch's axial band, which bounds the curve exactly by the convex hull property. Cylinder-side splitting was generalised to match: the split edge's endpoints only have to sit on the two boundary rulings at interior heights, so a horizontal section and a slanted elliptical one go through the same path. Cutting a cylinder with a plane tilted 20 degrees now returns a validated six-face solid whose volume matches the analytic result.
- Fixed planar triangle winding for good: it is now decided by comparing each facet against the face's effective normal, instead of being inherited from the trim-loop winding or from the triangulation library's output order. The previous rule assumed earcut preserves the input polygon's winding, which it does not, so a reversed face could tessellate identically to the original and contribute the wrong sign to volume. The oblique cut exposed this: its cap face read -1560 instead of +1560 and the solid measured a third of its true volume.
- Cylinder recognition no longer assumes a Z axis. `recognize_cylinder_patch()` derives the ruling direction, base circle centre, radius, and height of a patch of any orientation, and every consumer - support intersection, section splitting, ruling splitting, and the edge-on-patch test - now works in that patch frame instead of in world XY/Z. A cylinder rotated off axis cuts exactly like an axis-aligned one, with the same face count and volume.
- Added `BrepTransform::transform_solid/shell/face()` for rigid transforms. Rotation plus translation maps every supported geometry class onto itself and keeps rational weights, so a rotated cylinder stays an exact cylinder; non-rigid transforms are rejected rather than silently turning circles into ellipses the recognizers cannot model.
- The exact B-Rep boolean now returns a validated solid for a cylinder cut lengthwise by a box: 7 faces, 4 of them still NURBS side patches, with a tessellated volume matching the analytic circular-segment result. This is the first exact boolean case whose result is bounded by both curved and planar faces and whose cut face is produced by imprinting rather than by a generated cap.
- Planar face splitting is now boundary-curve aware. `split_planar_face_by_edge()` locates the split endpoints on the real boundary curves by ternary search and subdivides the boundary edge exactly with `split_bezier_at()`, so a chord landing on a circular cap keeps its arcs. It used to search a coarsely sampled boundary polygon and rebuild both halves as polylines, which meant a chord on a curved boundary was never recognised as a boundary hit at all.
- Added `split_planar_face_by_interior_loop()`: a closed intersection loop that never reaches the face boundary now imprints the face into the region inside the loop plus the remainder carrying the loop as a hole. A boundary-to-boundary split cannot express this case, which is what a cutting tool's own face always needs.
- Boolean cap faces are now only added when they improve the stitching report. With interior-loop imprinting in place the cutting face usually comes from the operand itself, and adding a generated cap on top produced a duplicated face.
- Fixed two orientation defects that this case exposed: `representative_face_point()` returned the centroid of a pierced face, which can sit inside the hole and invert the inside/outside classification, and planar tessellation flipped the trim-loop winding a second time by face orientation, so reversing a face left its mesh, and therefore its volume contribution, unchanged. Winding now follows the trim loop and is corrected only when the loop disagrees with the face orientation.
- Planar trim clipping of intersection lines is now analytic instead of polyline-based: each degree 1 or 2 p-curve span is solved against the cut line in Bernstein form, so a chord across a circular face ends exactly on the arc rather than on a sampled chord that sat a sagitta short of it (about 3e-2 for a radius 10 cap at the previous sampling density). Inside/outside classification between crossings still uses a densely sampled polygon, where the test points are far from the boundary; degree 3 and higher spans still fall back to polyline crossings.
- Intersection line segments are now clipped to the unpadded face bounding-box overlap, with the tolerance-padded overlap kept only as a fallback. The padding used to leave the segment overshooting the face by the linear tolerance, which prevented intersection edges from closing into a loop. With both fixes a lengthwise cylinder cut now produces an exactly closed rectangular cap loop and a cap face.
- Cylinder-side patches can now be split along a vertical ruling edge, not only along horizontal section arcs. The two halves share the original NURBS support surface and are trimmed by narrowed wires whose horizontal arcs are exact rational Bezier sub-arcs, so a plane cutting a cylinder lengthwise now reaches the classification stage instead of being reported as a skipped split. Shell reconstruction for that case is still open: the half-cylinder difference reports unmatched edge uses because the planar cap builder cannot yet close a loop that mixes arcs and lines.
- Added `NurbsCurve3::split_bezier_at()`: exact rational de Casteljau subdivision of a single-span Bezier curve, keeping weights so true-circle arcs stay true circles after splitting. This is the first curve-trimming primitive available to the boolean and trimming pipelines.
- Plane/cylinder support intersection is no longer limited to horizontal section arcs: a plane parallel to the cylinder axis now produces the vertical ruling line where it meets a recognized cylinder-side patch, clipped to the patch Z span and rejected when the ruling falls outside the patch angular span, when the plane misses the radius, or when one plane would cut the same patch twice.
- `Shape::Compound` is now connected to exact boolean and STEP boundaries: boolean results can become `Shape`, shape trees can expose/flatten contained solids, and STEP import/export can round-trip compound solids without forcing callers back through the legacy single-solid path.
- NURBS face tessellation now switches to p-curve trim loops when inner loops are present, preventing trimmed NURBS holes from being filled by the old full-surface grid path while preserving existing cylinder, sphere, torus, and other full-surface grid paths.

### 2. Planar trimming is still mesh-only

The new sampling fix makes curved planar caps display correctly, and NURBS tessellation now has a first guarded p-curve trim path. Trimming itself is still not yet a complete first-class geometric object: a robust kernel needs stronger 2D p-curve orientation rules, loop containment, adaptive interior refinement, and boolean split integration.

Impact:

- curved caps look better now
- exact area, exact intersections, STEP round-trip, and editing still need real trims

Replacement target:

- introduce `TrimmedFace` semantics: surface + outer loop + inner loops + 2D p-curves
- store 3D edge curve and per-face 2D curve mapping
- make tessellation consume trims adaptively

### 3. STEP import is skeletal

The exporter can write several B-Rep entities, including circles and B-spline surfaces. The importer currently handles only a narrow subset and falls back to lines or default planes when unsupported entities appear.

Impact:

- Seamless_CAD compatibility cannot rely on imported STEP fidelity yet
- exported shapes may not round-trip into the same kernel accurately
- OCCT replacement will be brittle for real customer data

Replacement target:

- support `CIRCLE`, `TRIMMED_CURVE`, `EDGE_CURVE`, oriented edges, face bounds, B-spline curves, B-spline surfaces, and placement transforms properly
- add round-trip tests for box, cylinder, hole, fillet, loft, sweep

### 4. IGES export is placeholder-level

The IGES exporter currently ignores the actual solid body and emits a minimal Type 186-style placeholder.

Impact:

- useful as a smoke test only
- not suitable for interoperability

Replacement target:

- either defer IGES intentionally, or implement real bounded surfaces and topology export after STEP is solid

### 5. Shell closure is asserted by flag, with first validator now added

`Shell::closed(faces)` marks a shell as closed. `Solid::new()` checks the flag, but the flag does not prove manifoldness, edge pairing, loop closure, orientation consistency, or tolerance validity.

Impact:

- invalid solids can be created silently
- downstream mass properties, STEP export, selection, and booleans can trust a false invariant

Current hardening:

- Added `Shell::validate_closed()` for minimum topological checks.
- Added `Shell::is_topologically_closed()` and `Solid::is_topologically_valid()`.
- Added tests proving box and cylinder pass, and a box with one missing face fails.
- Added `Face::validate_boundary_on_surface()` and integrated it into shell validation.
- Added a test proving a closed wire on the wrong plane is rejected as off-surface.
- Added Plane p-curve derivation from 3D boundary curves.
- Added Plane p-curve validation comparing `surface(uv)` against the owning 3D edge.
- Moved planar face tessellation to consume p-curve UV loops instead of re-projecting raw 3D wire points.
- Added tests for cylinder cap p-curves, off-plane p-curve rejection, and inner hole p-curve loops.
- Added optional `Face::pcurves` storage, with stored p-curves used by validation and planar tessellation when present.
- Plane faces now attach p-curves by default during `Face::new()`.
- Plane p-curve projection now solves the non-orthogonal plane basis correctly instead of assuming orthonormal axes.
- Added a hollow-box volume regression that protects non-orthogonal planar rim faces.
- Added NURBS boundary p-curve derivation for edges that match a NURBS face's outer iso-param boundaries.
- NURBS faces now attach boundary p-curves by default when derivation succeeds.
- Added tests proving cylinder side NURBS faces derive and store boundary p-curves.
- Added a projected NURBS p-curve fallback: if an edge is not on an outer iso-boundary, sampled 3D edge points are projected to the NURBS surface and stored as a degree-1 UV p-curve.
- Added generic `Face::validate_pcurves()` for Plane and NURBS faces.
- Shell validation now checks stored p-curves and reports mismatch counts/distances.
- Added regression tests for projected NURBS p-curves and corrupted stored p-curves.
- Shell validation now rejects shared edge uses that run in the same direction instead of opposite directions.
- Added a regression test that reverses one box face loop and proves same-direction shared edges are detected.
- Shell validation now checks that each oriented edge's curve endpoints match its oriented start/end vertices.
- Added a regression test proving a closed shell with a shifted edge curve is rejected even when the topological vertex loop still closes.
- Shell validation now rejects degenerate edge uses whose start/end vertices collapse within linear tolerance.
- Added a regression test proving a zero-length edge is reported even when its curve endpoints remain coherent with the collapsed vertices.
- Shell validation now checks planar face orientation consistency through p-curve outer-loop winding.
- Added a regression test proving a closed box shell with one inward planar face orientation is rejected without relying on broken edge pairing.
- Shell validation now rejects non-finite edge-use vertices and curve samples before NaN/Inf can leak into downstream tessellation or booleans.
- Added a regression test proving a NaN curve control point is reported even when the owning topological vertices are finite.
- Shell validation now reports duplicate face boundary signatures and duplicate directed edge uses so copied/overlapping topology is diagnosable before boolean or export steps.
- Added a regression test proving a duplicated box face is reported through dedicated duplicate face/edge-use counters.
- Shell validation now rejects degenerate planar faces whose p-curve outer loop collapses to near-zero signed area.
- Added a regression test proving a planar face with collapsed UV trim area is rejected even when the rest of the shell remains present.
- Added `Solid::try_new()` / `Solid::try_simple()` so callers can create solids through the validation gate and receive structured reports on failure.
- Added an internal `zenith_algo::validated_solid()` helper and moved normal `zenith_algo` solid generation paths onto the validated generation gate.
- Fixed `DirectModeling::push_pull_face()` plane reconstruction so moved planar faces rebuild their supporting plane from shifted boundary points, keeping p-curves, faces, and wires coherent.
- Fixed `ExtrudeBuilder` bottom cap orientation and moved it onto `validated_solid()`.
- Fixed `HoleBuilder` inner cap loop orientation and moved it onto `validated_solid()`.
- STEP import now uses `Solid::try_simple()` so imported solids pass through the same validation gate before being accepted.
- STEP import now splits entity arguments at top-level commas only, so nested lists such as `ADVANCED_FACE('',(#bound1,#bound2),#surface,.T.)` and point tuples are not corrupted by naive comma splitting.
- STEP import now preserves simple `TRIMMED_CURVE` of `CIRCLE` arcs as degree-2 rational NURBS instead of always falling back to straight line edges.
- STEP `TRIMMED_CURVE` import now reads trim point references and can derive circle trim endpoints from `PARAMETER_VALUE` ranges when explicit point refs are absent.
- STEP `TRIMMED_CURVE` import now honors `sense_agreement`; false-sense circle trims are imported with reversed curve direction.
- STEP import now reconstructs direct and complex `B_SPLINE_CURVE_WITH_KNOTS` edge curves, including `RATIONAL_B_SPLINE_CURVE` weights.
- STEP import now honors `EDGE_CURVE` same-sense flags by reversing NURBS curve parameter direction when the STEP edge runs opposite to the underlying geometry.
- STEP import now reconstructs direct and complex `B_SPLINE_SURFACE_WITH_KNOTS` entities, including rational surface weights from `RATIONAL_B_SPLINE_SURFACE`.
- STEP complex entity parsing now matches exact entity names instead of prefix substrings, avoiding confusion between `B_SPLINE_CURVE` and `B_SPLINE_CURVE_WITH_KNOTS` when entity order varies.
- Added a cylinder STEP round-trip regression proving imported cylinder side faces remain NURBS, the imported solid passes topology validation, and tessellated volume remains in range.
- STEP export now emits face-local `PCURVE` / `SURFACE_CURVE` entities when p-curves are available, keeping 3D edge curves and surface trim curves paired in the file.
- STEP import now resolves `SURFACE_CURVE` through its 3D curve component so p-curve-enriched exports still round-trip through the current importer.
- Planar face tessellation now samples curved p-curve loop segments adaptively using an internal chordal-deflection-like target derived from `TessellationParams`.
- Added a regression test proving cylinder cap tessellation keeps curved boundaries with coarse settings and refines them when tessellation divisions increase.
- Full `modeling_test` currently passes with strict generation enabled for normal `zenith_algo` solid builders and STEP round-trip import.

Replacement target:

- extend topological validation with face orientation consistency, NURBS-surface p-curve derivation, tolerance accumulation, self-intersection detection, and bounding-box sanity
- make constructors return `Result` for validated solids in production paths

### 6. Direct modeling currently rebuilds many edges as lines

Status: fixed for rigid and ruled edits; general edits now fail loudly instead of degrading.

`push_pull_face()` used to move selected vertices and recreate every affected edge with `Edge::line_between()`, destroying arcs and splines. Worse, it left curved face geometry untouched while moving that face's wire, so a cylinder came out with its surfaces no longer matching their boundaries.

It now classifies each element by how the edit moves it:

- an edge whose endpoints both stay is kept exactly as it is, and one whose endpoints both move is translated with its curve intact
- an edge with one endpoint moving must be linear; a curved one would need the adjacent surfaces extended and re-trimmed, so it is refused rather than replaced by a chord
- a face whose whole boundary moves is transported rigidly, geometry included
- a planar face only partly moving keeps its plane, provided the push slides along it
- a face linear in `v` - a cylinder or cone side patch - is extended by translating the control row on the moving side, which keeps it exact
- anything else returns an error naming what is missing

Shared edges are rebuilt once and keyed by their original id, so faces keep sharing the same edge instead of each holding a private copy.

Pulling a cylinder's cap now yields an exact taller cylinder: all four side patches stay NURBS, all sixteen circular arc uses survive, the boundary stays on the analytic cylinder to 1e-9, and the volume matches the analytic value.

`taper_face()` had a worse version of the same problem: it rotated only the target face and left every neighbour referencing the old edges, so it failed shell validation on every input - the operation could not succeed at all, and nothing exercised it. It now shares the push-pull machinery: the rotation is applied as a rigid transform to whatever moves with the face, shared edges are rebuilt once by id, and a partly-moved planar neighbour has its plane refitted through the new boundary with Newell's method, oriented to keep its outward sense. An edit that leaves a neighbour non-planar, or that crosses a curved face, is refused. Tapering a box top face now returns a valid solid whose volume matches the analytic trapezoidal prism.

Remaining target:

- for general edits, solve adjacent surface extensions and re-trim

### 7b. Surfaces of revolution were distorted near the axis

Status: fixed.

`make_sphere()` built a patch whose points drifted up to 0.2 mm from the true sphere at radius 15, about 1.3 percent, and exact B-Rep integration measured its volume as 13922 against an analytic 14137. The profile curve and the boundary iso-curves were exact; only the interior was wrong.

The cause was in `RevolveBuilder::revolve_curve()`. A profile control point sitting on the rotation axis does not move when revolved, and the code gave every column of that row the same weight. Every other row carries the rational arc's alternating weights `(1, cos(dtheta/2), 1, ...)`. That mismatch makes the tensor-product denominator non-separable, so the surface stops being the revolved profile anywhere the on-axis row has influence - which is exactly the region near each pole.

On-axis rows now carry the same arc weight pattern as the rest. The sphere is exact to 1e-9 across its parameter range, its integrated volume, area, and centroid match the analytic values, and a revolved cone keeps radius exactly proportional to height.

This affected every surface of revolution whose profile touches its axis, not only spheres.

### 7. Cone apex is approximated as a tiny frustum

Status: fixed.

`make_cone()` used to clamp the top radius with `r_top.max(0.001)`, so a cone was always a very thin frustum and its volume was off by about 0.01 percent with a spurious top face.

`r_top <= 1e-6` now builds true apex topology: the side patch's `v = 1` row collapses to the apex point, the two rulings meet there, and the side wire closes with three edges - bottom arc plus two rulings - with no top face. The degenerate row keeps the same rational weight pattern as the rest of the patch, the same requirement that the revolve fix above turned on. Volume, area, and centroid now match the analytic cone to machine precision, and a positive `r_top` still builds an exact frustum with no clamping.

Fixing this exposed a second defect: UV trim-loop sampling dropped each segment's first point unconditionally, assuming it duplicated the previous segment's last point. A face with a degenerate edge has a real jump in UV there, so the cone's trim domain came out as half its square and every integral over it was short by a third. Both UV loop samplers now drop a point only when it actually coincides with the previous one.

### 8. Mass properties are mesh-derived

Status: an exact B-Rep path now exists; the mesh path remains as preview.

`MassCalculator::compute_from_mesh()` derives area and volume from triangles. This is useful for preview and coarse tests, but it depends on tessellation quality.

Impact:

- previous square caps directly distorted cylinder volume
- precision changes with tessellation parameters
- exact CAD measurement needs analytic or trimmed-surface integration

Replacement target:

- keep mesh mass as preview
- add exact or high-accuracy face integration for planes, cylinders, cones, spheres, tori, and NURBS patches

Changes made:

- Added `Surface3::evaluate_with_derivatives()` so any surface can report the area element `dS/du x dS/dv`. The default is a central difference; planes and NURBS surfaces override it with their analytic derivatives.
- Added `zenith_tess::face_uv_triangulation()`, exposing the trimmed parameter domain the tessellator already builds, so integration and display share one notion of what a face covers.
- Added `MassCalculator::compute_from_brep()`, which integrates volume, area, centroid, and the inertia diagonal over the faces by the divergence theorem, evaluating the surface inside each domain triangle with a degree-4 quadrature rule instead of reusing linearized triangle vertices. Void shells are subtracted.
- Planar faces take an analytic path instead: on a plane every integrand is polynomial in `(u, v)`, so Green's theorem turns the domain integrals into line integrals along the p-curves, evaluated with 10-point Gauss-Legendre. This removes the polygonal approximation of curved trim boundaries, which was the dominant error - a circular cylinder cap was integrating as an inscribed polygon.
- Added `NurbsCurve2::evaluate_derivative()` for the rational p-curve tangent those line integrals need.
- A 10 x 30 cylinder now returns its analytic volume, area, centroid, and inertia to machine precision, where the mesh path at the same tessellation settings is short by about 0.9 percent.

## Medium-Risk Areas

### Surface tessellation is still mostly uniform

`TessellationParams` exposes only `u_divisions` and `v_divisions`. Planar trimmed-face boundaries now use adaptive p-curve subdivision internally, but NURBS/Coons/Gordon/Triangular surface interiors still use uniform parameter grids. Large parts, small radii, high curvature, and very flat surfaces therefore still need a real deflection policy.

NURBS face tessellation now follows the stored p-curve trim loops in general, not only for holes or axis-aligned sub-rectangles. The loops are triangulated in UV, then refined by Rivara longest-edge bisection until every triangle is no coarser than the requested parameter grid and its 3D chord stays inside a deflection target derived from the patch size. A single shared midpoint table means refinement never leaves T-junction cracks, and triangles are oriented by the face's effective surface normal instead of by the trim-loop winding. Faces whose p-curves cannot be triangulated - sphere poles, for instance - still fall back to the full-range grid.

Interiors are therefore adaptive for trimmed NURBS faces, but `TessellationParams` still exposes only `u_divisions` and `v_divisions`: the deflection target is derived from them rather than requested directly, and Coons/Gordon/Triangular patches remain on the uniform grid.

Target:

- add public chordal deflection, angular deflection, min/max segment count, and adaptive subdivision controls
- make surface interiors adaptive, not only planar p-curve boundaries
- preserve stable IDs between remeshes where possible for selection

### Surface/surface intersection is early-stage

The geometry crate contains surface intersection/refinement code, but it is sampling-driven and not yet integrated into B-Rep topology operations.

Target:

- promote SSI/CSI into the boolean and trimming pipeline
- harden with NURBS/NURBS, plane/NURBS, cylinder/NURBS, tangent, overlap, and near-degenerate cases

### Feature history has useful direction but weak generality

The feature tree can recompute simple primitive and direct-edit operations. Some paths are still box-specific, including hardcoded dimensions for one fillet recompute path.

Target:

- store operation parameters and target references explicitly
- separate parametric feature replay from direct modeling commands
- strengthen topological naming using geometric signatures plus ancestry and adjacency

## Strategic Read

The promising parts are real: NURBS curves/surfaces, Coons/Gordon/Triangular surfaces, sweep/loft/revolve, and early persistent signatures give this kernel a route beyond a thin OCCT clone.

The critical path is:

1. Make topology and trimmed faces trustworthy.
2. Make tessellation a consumer of exact geometry, not the source of truth.
3. Build exact B-Rep booleans and surface intersections.
4. Replace Seamless_CAD through a compatibility server while the native kernel matures.
5. Use freeform patches, G2 blends, direct face edits, and curve-network surfaces as the differentiator.

The next best engineering target is a `TrimmedFace`/p-curve layer plus adaptive tessellation. That directly supports cylinder caps, holes, curve patches, STEP fidelity, and future Plasticity-like modeling.

## Conformance Sweep

Every primitive is now covered by one table-driven test asserting it is a valid solid, that exact B-Rep integration reproduces its analytic volume and area, and that a STEP round-trip returns a valid solid with the same volume. Box, cylinder, sphere, cone, frustum, and torus all pass to within 1e-8 relative; the residual is quadrature error on the doubly curved surfaces, not geometry error.

Extending the sweep to `extrude_wire()` found two more defects. Its ruled side surface built the control grid transposed - `control_points` is indexed `[u][v]`, but the profile's points were laid out along `v` - so every side normal pointed inward, and a profile with arcs could not be built at all because the `u` direction had fewer control points than its degree. Separately, the top cap's edges were always straight lines between the moved vertices, so extruding a circle produced a chord polygon on top that did not match the curved sides. The grid is now laid out profile-along-`u`, and the top loop is the profile translated by the extrusion vector. Extruding a rectangle gives its exact prism volume, and extruding a four-arc circle now gives an exact cylinder, arcs intact.

The same sweep over the modeling operations - hollow box, drilled box, filleted box, single-edge fillet, chamfered box, thickened face - found that `make_drilled_box()` built the hole's cylindrical wall with its normal pointing away from the axis. A hole has its material outside the wall, so the normal must point toward the axis. The solid still validated, because topological validation only checks that each edge is used once in each direction and only ties orientation to loop winding for planar faces; it took an exact integral to see it. The drilled box measured 13892 where the analytic answer is 12321 - larger than the undrilled box. The wall patches are now parameterized with `u` reversed, which turns the normal inward while leaving the wire winding, and therefore every edge pairing, untouched.

Building the sweep found that STEP export wrote reals with `{:.6}`, six decimal places. That capped interchange fidelity at roughly 1e-7 relative and was the dominant round-trip error for every curved primitive. Reals are now written with twelve decimals, which drops the round-trip volume error from about 5e-8 to about 1e-13.

## Current Kernel Hardening Queue

1. Replace projected degree-1 UV polylines with fitted/interpolated p-curves where exact curve class is needed.
2. Extend STEP import from self-authored B-spline curves/surfaces and circle trim ranges to broader AP schema variants, including conic variants beyond circles and more real-world complex entity combinations.
3. Make p-curves explicit in STEP import/export paths instead of internal-only topology metadata.
4. Extend tessellation from internal planar p-curve adaptive sampling to full chordal/angle deflection for surface interiors.
5. Split mesh booleans from exact B-Rep booleans and start the exact intersection/classification pipeline.
6. Add a broader invalid-B-Rep test suite around imported and edited topology.
7. Extend exact boolean coverage beyond the recognised cases. Plane/cylinder now covers perpendicular, parallel, and oblique planes, but an oblique section that leaves the patch's axial band is rejected instead of being clipped, a plane that cuts one patch twice is rejected rather than producing two rulings, and NURBS/NURBS pairs (cylinder against cylinder, sphere, cone, or freeform) are still entirely unsupported.
8. Expose real tessellation controls. Trimmed NURBS interiors are adaptive now, but chordal deflection, angular deflection, and min/max segment counts are still derived from `u_divisions`/`v_divisions` instead of being requested, and Coons/Gordon/Triangular patches still use uniform grids with no trim awareness.
