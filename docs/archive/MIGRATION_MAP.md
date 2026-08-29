# Seamless_CAD OCCT Replacement Migration Map

Goal: replace the OCCT-backed `cad_server.exe` used by Seamless_CAD with the in-repo Rust CAD kernel while preserving the Blender add-on workflow.

Current folders:

- `crates/`: Rust kernel workspace (`zenith_math`, `zenith_geom`, `zenith_topo`, `zenith_algo`, `zenith_tess`, `zenith_io`, `zenith_py`).
- `reference/OCCT/`: OCCT source reference. Version file reports `8.1.0 dev1`.
- `reference/CAD_8_1_5_1/`: Seamless_CAD reference package. Blender Python add-on talks to `cad_server.exe` on `127.0.0.1:8080`.
- `blender_addon/H-CAD_V_1_0_0/`: target package folder for the Rust/PyO3 add-on build; currently empty.

## Replacement Boundary

Seamless_CAD has two separable layers:

1. Blender UI and workflow layer:
   - Python operators, panels, properties, snapping, sketch UI, feature list.
   - Mostly reusable.

2. Geometry kernel service layer:
   - Current path: `core_bridge.py` -> TCP request -> `cad_server.exe` -> OCCT/OpenCascade DLLs.
   - Replacement path: `core_bridge.py` -> same or adapted protocol -> Rust service/PyO3 module -> `zenith_*` crates.

The lowest-risk route is to first emulate the existing `cad_server.exe` protocol in Rust, then replace internals feature by feature.

## Seamless to Rust Feature Matrix

| Seamless feature / primitive | Current Seamless side | Rust kernel status | Migration action | Priority | Verification target |
| --- | --- | --- | --- | --- | --- |
| Stack lifecycle | `create_stack`, `delete_stack`, `update` requests in `core_bridge.py` | No compatible server protocol yet | Implement Rust `cad_server` compatibility layer and stack store | P0 | Add-on can create a part, update stack, and receive mesh data |
| Mesh preview | `generate_mesh`, `update(include_mesh)` | `zenith_tess` can tessellate `Solid`/`Face` to `TriangleMesh` | Match Seamless response binary/JSON format or adapt bridge | P0 | Box preview appears in Blender with edges/face ids |
| Picking | binary op codes for edge/face/vertex/midpoint picking | No equivalent exposed | Add ray picking over tessellated B-Rep with lineage ids | P0 | Edge/face selection returns stable lineage tokens |
| Box | `BOX` | `PrimitiveBuilder::make_box` exists | Wire into server stack evaluator | P0 | Geometry, volume, STEP roundtrip for box |
| Cylinder | `CYLINDER` | `make_cylinder` exists | Wire into server stack evaluator | P0 | Smooth cylinder, correct caps, selectable edges/faces |
| Sphere | `SPHERE` | `make_sphere` exists, single NURBS face | Wire into server stack evaluator | P1 | Preview normals, STL/STEP export sanity |
| Cone/frustum | `CONE` | `make_cone` exists | Wire into server stack evaluator | P1 | Top/bottom radius behavior and volume |
| Torus | `TORUS` | `make_torus` exists | Wire into server stack evaluator | P1 | Closed surface, feature edge extraction |
| Boolean operations | `operation`: `BASE`, `ADD`, `SUB`, `INT`; OCCT fuse/cut/common | Rust has mesh-classification boolean, not robust B-Rep boolean | Keep mesh boolean for preview only; implement real B-Rep boolean or staged fallback | P0/P1 | Overlapping boxes produce watertight result; STEP export remains solid |
| Fillet | `FILLET`, selected edges, variable per-edge radii | Limited box-specific fillet APIs | Replace first for box edges; then implement general edge blend/topology rebuild | P0/P1 | Selected edge fillet works after transforms and chained booleans |
| Chamfer | `CHAMFER`, selected edges | Limited box z-edge chamfer | Same staged approach as fillet | P0/P1 | Selected edge chamfer works on non-box results |
| Face offset / push-pull | `FACE_OFFSET` | `DirectModeling::push_pull_face`, `offset_multiple_faces` limited | Implement lineage-based face modification for general planar faces | P1 | Offset selected face without losing adjacent topology |
| Face inset | `FACE_INSET` | No direct equivalent found | New feature: inset wire creation + local face extrusion/cut | P1 | Inset rectangle on planar face with optional depth |
| Draft | `DRAFT`, target faces + reference lineage | `taper_face` limited | Generalize draft around neutral plane/edge reference | P1 | Draft selected faces with stable target tracking |
| Shell | `SHELL` | `make_hollow_box` and `ThickenBuilder` exist, limited | General shell/offset solver needed | P1 | Open-top box shell first, then arbitrary solid shell |
| Cleanup | `CLEANUP`, unify faces/edges | No equivalent found | Add topology cleanup: coplanar face merge, collinear/cocircular edge merge | P2 | Imported/boolean models simplify without shape drift |
| Curve | `CURVE`, point list, optional pipe/fill | NURBS curve exists; pipe via sweep exists | Map points/segments to NURBS/polycurve; pipe to sweep | P1 | Curve preview, filled closed curve, pipe curve |
| Polyline | `POLYLINE`, point list with per-point fillet flag | Basic wires/edges exist | Add polyline builder with optional corner fillets | P1 | Closed/open polyline, corner fillets |
| Arc/circle | `ARC` | `Circle3`, NURBS circle/arc representation exists | Expose arc segment builder | P1 | True circle/arc selection and export |
| Surface from curves | `SURFACE`, closed curves | Coons/Gordon/Triangular/trimmed surfaces exist | Build from Seamless segments and closed boundaries | P2 | Filled surface has valid normals and trim boundary |
| Slot | `SLOT` | No explicit primitive; can be built from arcs + lines | Add slot builder | P1 | Stadium shape, extrude/subtract behavior |
| Polygon | `POLYGON` | No explicit primitive; wire/extrude available | Add regular polygon builder | P1 | N-gon face/solid with correct radius/side count |
| Gear | `GEAR`, teeth/module/pressure angle | No equivalent found | Add involute gear profile generator + extrude | P2 | Tooth count, module dimensions, export |
| Helix | `HELIX`, turns | No explicit curve; sweep has RMF | Add helix curve builder; support pipe/sweep path | P2 | Spring/helix preview and sweep as path |
| Revolve | `REVOLVE`, profile target | `RevolveBuilder::revolve_curve` exists | Map selected profile/face to revolve input | P1 | 360 and partial revolve, cap behavior |
| Sweep | `SWEEP`, profile/path UUIDs, frame mode | `SweepBuilder::sweep_circle_along_curve` only | General profile sweep needed; keep circle-pipe first | P1 | Profile along curve, helix-axis frame later |
| Loft | `LOFT`, multiple profile UUIDs | `LoftBuilder::loft_curves` exists | Map profile UUIDs and ordering | P1 | Multiple sections produce stable surface/solid |
| Face loft | `FACE_LOFT` | No direct equivalent | Add face-to-face loft and fuse behavior | P2 | Two selected faces bridge/fuse cleanly |
| Face revolve | `FACE_REVOLVE` | Revolve curve exists, no face-specific operation | Add face boundary extraction + revolve/fuse | P2 | Revolved face generates solid/surface as expected |
| Dynamic loft / variable box | `VARIABLE_BOX` | No exact equivalent; loft support exists | Implement top/bottom profile generator and loft solid | P1 | Box-circle and circle-box lofts with editable height |
| Mirror | `MIRROR` | Transform exists; boolean integration missing | Add transform-copy feature and stack operation | P2 | Mirrored target participates in booleans |
| Linear array | `ARRAY_LINEAR` | Assembly/transform exists; no stack feature | Add repeated transformed solids | P2 | Count/axis/distance editable |
| Circular array | `ARRAY_CIRCULAR` | Transform exists; no stack feature | Add radial repeated solids | P2 | Count/angle/axis editable |
| Instance / body link | `INSTANCE` and target collection stack ptr | Assembly exists, no protocol integration | Add stack-to-stack instance resolution | P1 | Linked part updates propagate |
| Groups | `GROUP_START`, `GROUP_END` nested boolean groups | No equivalent found | Add stack evaluator with nested boolean grouping semantics | P1 | Grouped subtract/add matches Seamless |
| STEP import | `import_step`, creates `STEP_PART` entries | Importer is limited to simple STEP entities | Expand importer or integrate a temporary fallback during migration | P0/P1 | Import OCCT-exported box/cylinder/fillet test suite |
| STEP export | `export_stack_to_step`, `export_parts_to_step` with XCAF assembly names | Exporter exists, assembly/name support limited | Implement AP214/AP242 naming/assembly path after core export is stable | P1 | STEP opens in FreeCAD/OCCT with correct solids and names |
| IGES export | `export_stack_to_iges` | Current IGES exporter is skeletal | Decide whether to support fully or defer in favor of STEP | P3 | If kept: load in external CAD |
| STL export | `export_stack_to_stl` | STL binary export exists | Wire server action and scale/quality args | P0 | Non-empty STL, expected triangle count |
| SVG import | `import_svg`, Python flattens SVG then server builds shape | No Rust-side SVG part builder found | Reuse Python flattening, add Rust builder from flat segments | P2 | Simple SVG path imports as curves/faces |
| Measurement | `measure_stack`, `measure_entity` | Mesh mass properties; face/edge inspection exists | Match response format and entity kind codes | P0 | Volume/area/bbox and selected edge/face measurements |
| Shader/wire payload | Wireframe engine expects edge points/counts/lineages | `ShaderBRepPayload` exists but not Seamless format | Add stable lineage payload generator | P0 | Highlight/selection overlays match mesh |
| Sketch UI | Python sketch mode handles drawing/history | Reusable | Keep Python UI; only replace solver/finalization backend as needed | P1 | Sketch edit history survives recompute |
| Sketch constraints | Python GCS plus external solve path | Rust solver handles core constraints, not all Seamless constraints | Add missing angle/concentric/symmetric/midpoint/arc constraints | P1 | Constraint panel examples converge |

## Required Compatibility API

The Rust replacement server should initially support these existing `core_bridge.py` actions:

- `create_stack`
- `delete_stack`
- `update`
- `generate_mesh`
- `import_step`
- `import_svg`
- `export_step`
- `export_stack_to_step`
- `export_parts_to_step`
- `export_stack_to_stl`
- `export_stack_to_iges`
- `measure_stack`
- `measure_entity`
- `render_viewport_sdf`
- `csg_preview_begin`
- `csg_preview_update`
- `csg_preview_end`

And these binary picking operations:

- op `1`: `pick_edge`
- op `2`: `pick_face`
- op `3`: `pick_vertex_from_stack`
- op `4`: `pick_midpoint_from_stack`
- op `5`: `pick_face_from_stack`
- op `6`: `pick_edge_from_stack`

## Migration Phases

### Phase 0: Protocol and Test Harness

- Document exact request/response schemas from `core_bridge.py`.
- Implement a Rust server crate that listens on `127.0.0.1:8080`.
- Return compatible error responses and log format.
- Build standalone tests that send real Seamless requests to both servers and compare results.

Exit criteria:

- Blender add-on starts without changing Python UI code.
- Empty stack, box stack, and measurement requests work.

### Phase 1: Preview-Parity Kernel

- Implement stack evaluation for primitive solids: box, cylinder, sphere, cone, torus, polygon, slot.
- Implement tessellated mesh response, edge wire response, lineage ids, and picking.
- Wire STL export and stack measurement.

Exit criteria:

- Basic modeling in Blender works without OCCT for preview, selection, and STL.

### Phase 2: Feature-Parity Core

- Implement Boolean `ADD/SUB/INT` with a clear split:
  - preview: mesh boolean is acceptable;
  - final/export: must become watertight B-Rep or explicitly marked unsupported.
- Generalize fillet/chamfer/face offset/draft/shell beyond box-only cases.
- Implement stack groups and instances.

Exit criteria:

- Common Seamless workflows produce stable editable stacks and valid exported solids.

### Phase 3: Exchange and Assembly

- Expand STEP import/export coverage.
- Implement assembly/naming export equivalent to Seamless `export_parts_to_step`.
- Decide IGES support level; either full implementation or documented deprecation.

Exit criteria:

- STEP files roundtrip through OCCT/FreeCAD-like tools with expected topology and names.

### Phase 4: Advanced Modeling

- Profile sweep, face loft, face revolve, variable loft.
- SVG path import to curve/surface/solid.
- Gear and helix primitives.
- Sketch constraint coverage parity.

Exit criteria:

- The remaining Seamless feature list is either implemented or intentionally scoped out.

## High-Risk Gaps

1. Real B-Rep boolean:
   - Current Rust boolean is mesh triangle filtering. This cannot replace OCCT final modeling/export by itself.

2. General fillet/chamfer:
   - Current Rust APIs are mostly box-specific. Seamless expects arbitrary selected edge targets.

3. Stable topology naming / lineage:
   - Seamless selection, modifiers, snapping, and edit history depend on lineage strings that survive recompute.

4. STEP import/export:
   - Current Rust STEP importer is useful for simple roundtrips but does not yet parse general CAD STEP geometry.

5. Server protocol parity:
   - The Python add-on is mature and expects specific binary layouts, async behavior, and response shapes.

## First Implementation Target

Recommended first vertical slice:

1. Add a Rust `cad_server`/`zenith_server` crate.
2. Implement `create_stack`, `delete_stack`, `update`, `measure_stack`.
3. Support only `BOX`, `CYLINDER`, `SPHERE`, `CONE`, `TORUS` with `BASE/ADD` initially.
4. Return mesh, wireframe, face ids, edge ids in the shape expected by `core_bridge.py`.
5. Add a small request replay test fixture built from captured Seamless requests.

This gives the fastest proof that OCCT can be unplugged without rewriting the Blender UI first.

Current implementation note:

- `crates/zenith_server` has been scaffolded as the first protocol-compatible Rust server.
- It currently handles stack lifecycle, empty `update`/`generate_mesh`, import/export success stubs, and measurement zero responses.
- It passed `cargo check -p zenith_server`.
- Running a full binary build was blocked in this Windows session by file locks in Cargo target artifacts, not by Rust type errors.
