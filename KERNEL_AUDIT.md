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

### 2. Planar trimming is still mesh-only

The new sampling fix makes curved planar caps display correctly, but trimming itself is not yet a first-class geometric object. A robust kernel needs 2D p-curves on face parameter space, orientation rules, loop containment, and validation.

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
- Planar face tessellation now samples curved p-curve loop segments adaptively using an internal chordal-deflection-like target derived from `TessellationParams`.
- Added a regression test proving cylinder cap tessellation keeps curved boundaries with coarse settings and refines them when tessellation divisions increase.
- Full `modeling_test` currently passes with strict generation enabled for normal `zenith_algo` solid builders and STEP round-trip import.

Replacement target:

- extend topological validation with face orientation consistency, NURBS-surface p-curve derivation, tolerance accumulation, self-intersection detection, and bounding-box sanity
- make constructors return `Result` for validated solids in production paths

### 6. Direct modeling currently rebuilds many edges as lines

`push_pull_face()` moves selected vertices and recreates all affected edges with `Edge::line_between()`. This destroys arcs/splines on affected boundaries.

Impact:

- box cases work
- cylinder, holes, fillets, and freeform patches will degrade under editing
- this conflicts directly with Plasticity-like direct editing goals

Replacement target:

- transform existing curves when the edit is rigid/affine
- rebuild analytic curves as analytic curves, not lines
- for general edits, solve adjacent surface extensions and re-trim

### 7. Cone apex is approximated as a tiny frustum

`make_cone()` clamps the top radius with `r_top.max(0.001)` to avoid singular topology.

Impact:

- stable for preview
- mathematically inaccurate for true cones
- may leak into measurements and export

Replacement target:

- explicitly model singular apex topology, or expose the current behavior as `make_frustum`
- add true cone tests and STEP export expectations

### 8. Mass properties are mesh-derived

`MassCalculator::compute_from_mesh()` derives area and volume from triangles. This is useful for preview and coarse tests, but it depends on tessellation quality.

Impact:

- previous square caps directly distorted cylinder volume
- precision changes with tessellation parameters
- exact CAD measurement needs analytic or trimmed-surface integration

Replacement target:

- keep mesh mass as preview
- add exact or high-accuracy face integration for planes, cylinders, cones, spheres, tori, and NURBS patches

## Medium-Risk Areas

### Surface tessellation is still mostly uniform

`TessellationParams` exposes only `u_divisions` and `v_divisions`. Planar trimmed-face boundaries now use adaptive p-curve subdivision internally, but NURBS/Coons/Gordon/Triangular surface interiors still use uniform parameter grids. Large parts, small radii, high curvature, and very flat surfaces therefore still need a real deflection policy.

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

## Current Kernel Hardening Queue

1. Replace projected degree-1 UV polylines with fitted/interpolated p-curves where exact curve class is needed.
2. Extend STEP import from self-authored B-spline curves/surfaces and circle trim ranges to broader AP schema variants, including conic variants beyond circles and more real-world complex entity combinations.
3. Make p-curves explicit in STEP import/export paths instead of internal-only topology metadata.
4. Extend tessellation from internal planar p-curve adaptive sampling to full chordal/angle deflection for surface interiors.
5. Split mesh booleans from exact B-Rep booleans and start the exact intersection/classification pipeline.
6. Add a broader invalid-B-Rep test suite around imported and edited topology.
