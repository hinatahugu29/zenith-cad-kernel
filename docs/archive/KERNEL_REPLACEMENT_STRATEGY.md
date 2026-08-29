# Kernel Replacement Strategy

Objective: remove OCCT from Seamless_CAD and replace it with the Rust `zenith_*` kernel, while growing beyond OCCT-style modeling into a more direct, expressive Plasticity/Fusion-like workflow.

## North Star

The project should not become a smaller clone of OCCT. The replacement kernel should preserve the productive parts of Seamless_CAD, then add strengths that are easier when the kernel, feature graph, selection model, and Blender UI are designed together.

Target identity:

- Direct modeling first: select faces/edges, push, offset, taper, fillet, bridge, rebuild.
- Precise B-Rep underneath: not only mesh preview.
- Rich curve/surface patching: Coons, Gordon, triangular patches, blends, trimmed regions.
- Editable feature history: Fusion-like parameter edits without losing selected targets.
- Fast preview paths: mesh/SDF/GPU-style approximations are allowed during interaction, but final geometry must be explicit and exportable.

## Two-Lane Architecture

### Lane A: Compatibility Lane

This lane exists to unplug OCCT without breaking the existing add-on.

- Emulate the current `cad_server.exe` protocol.
- Keep `reference/CAD_8_1_5_1` Python UI mostly intact.
- Implement stack primitives and responses in Rust.
- Match lineages, mesh arrays, face ids, pick results, measurements, import/export responses.

This is the lane that makes progress visible in Blender.

### Lane B: Native Zenith Lane

This lane exists to make the new kernel worth having.

- Build native Rust feature graph and topology naming.
- Add general curve/surface patch builders.
- Build real B-Rep boolean, split, trim, sew, heal, and classify operations.
- Add direct modeling operations that are not trapped by OCCT's assumptions.
- Expose richer operations to Blender only after the geometry model is trustworthy.

This is the lane that gives the project its own character.

## Design Principle: Preview Can Approximate, Commit Cannot

Interactive modeling should feel fluid:

- mesh boolean preview is acceptable
- coarse tessellation is acceptable
- SDF or shader preview is acceptable
- simplified edge overlays are acceptable

Committed geometry must be honest:

- final stack result must be a coherent B-Rep when the operation claims to produce a solid
- STEP export must not silently export preview meshes as exact solids
- topology ids must be stable enough for downstream operations
- failures should be explicit and recoverable in the UI

## Plasticity-Like Strengths to Prioritize

### 1. Curve Patch Workbench

This is a natural differentiator for Zenith because the Rust geometry crate already has Coons, Gordon, triangular, blend, trimmed, and NURBS surface modules.

Priority operations:

- patch from 3 boundary curves
- patch from 4 boundary curves
- network surface from curve grid
- fill selected edge loop
- rebuild surface with adjustable degree/control density
- G1/G2 bridge between faces
- zebra/curvature analysis output for surface quality

Required kernel capabilities:

- robust curve-loop validation
- curve orientation and endpoint snapping
- UV trim loop generation
- surface-to-surface intersection for trimming/fusing
- boundary continuity evaluation

### 2. Direct Face/Edge Modeling

Priority operations:

- push-pull planar face
- offset selected faces
- inset face with optional depth
- draft/taper around reference plane or edge
- extend edge/surface
- bridge/loft between selected faces
- replace face with patch

Required kernel capabilities:

- face adjacency graph
- edge/face classification
- local topology rebuild
- trimming and sewing
- persistent target matching

### 3. General Fillet and Chamfer

OCCT replacement is not credible until selected-edge fillet/chamfer is general enough for real work.

Milestones:

- box planar edges
- planar-polyhedral edges
- cylinder-plane edges
- variable-radius fillets
- multi-edge chain fillets
- rolling-ball style blends
- G2 blends for product-surface workflows

### 4. Feature Graph and Topology Naming

Seamless already behaves like a stack/history system. Zenith should formalize this instead of treating each update as a one-shot mesh build.

Needed concepts:

- stable primitive UUID
- stable operation node id
- generated face/edge ids derived from semantic role and geometry signature
- dependency links between target operations and source geometry
- rollback and recompute as first-class operations
- failure nodes that preserve editable parameters

### 5. Sketch-to-Solid Loop

Fusion-like strength depends on sketches surviving edits.

Priority:

- preserve Seamless sketch UI
- move more constraint solving into Rust
- add missing constraints: angle, concentric, symmetric, midpoint, arc tangent, equal radius
- finalize sketches into explicit profile wires
- make extrude/revolve/sweep/loft reference those wires by UUID

## Replacement Priorities

### P0: Must Work Before OCCT Can Be Removed

- Rust server process compatible with `core_bridge.py`
- stack create/delete/update
- basic primitives: box, cylinder
- mesh and wireframe response
- stable face/edge lineage strings
- face/edge/vertex picking
- measurement
- STL export
- clear error protocol

### P1: Daily Modeling Credibility

- sphere, cone, torus, polygon, slot
- add/sub/intersect with usable preview
- committed B-Rep boolean for common primitives
- selected-edge fillet/chamfer for common cases
- face offset, inset, draft, shell for planar solids
- STEP export for solids
- STEP import for common AP203/AP214 files
- sketch solver parity for common constraints

### P2: Differentiators

- curve patch workbench
- face bridge/loft/revolve
- surface replacement
- Gordon network surfaces
- G2 blends
- helix, gear, SVG-derived profiles
- topology cleanup and healing

### P3: Optional or Deferred

- full IGES support
- full XCAF-equivalent assembly metadata
- GPU compute acceleration
- advanced Class-A analysis beyond G2

## Immediate Build Plan

1. Build a `zenith_server` Rust crate that speaks the Seamless protocol.
2. Implement a strict parser for the existing primitive binary payload.
3. Evaluate only `BOX` and `CYLINDER` at first.
4. Return mesh and edge arrays in the expected format.
5. Add lineage generation to every face and edge.
6. Add a request replay harness so captured Python requests can be tested without Blender.
7. Add curve-patch features only after lineages, local topology rebuild, and final B-Rep export are not fragile.

## Non-Negotiables

- Do not hide missing exact geometry behind pretty mesh output.
- Do not rewrite the Blender UI until protocol compatibility is proven.
- Do not treat current docs' "100%" labels as truth; verify feature by feature.
- Do not let OCCT concepts limit Zenith's curve/surface design.
- Keep failure modes explicit so repeated redesign remains cheap.
