#!/usr/bin/env bash
# **門を、1 コマンドで通しで回す**（4-411）。
#
# なぜ要るのか
# ------------
# 2026/09/08 に、**記録されている門のうち 5 つが `origin/main` の時点で
# 赤**だと分かりました。**①は 10 日間、②は 5 日間、誰も気づいていません。**
# **そのあいだ、通しテストはずっと緑**でした。
#
# `.github/workflows/gates.yml` は、これらの門を回します。**ただし
# `push` では走りません**——2026/08/30 に、md だけの push でも毎回
# フルビルドになるのを避けて外してあります（理由はそのファイルに）。
# **このリポジトリは PR を使わない運用**なので、**実質ずっと回って
# いませんでした。**
#
# **回すのを飛ばしやすいのは、手間だからです。** ここが 1 コマンドです。
#
# 使い方
# ------
#     bash tools/run_gates.sh          # 全部
#     bash tools/run_gates.sh --quick  # 4 分かかる contact_placement を外す
#
# 読み方
# ------
# **終了コードが 0 でなければ、`main` に持っていくものではありません。**
#
# 見るのは 3 つです——
#
#   * 終了コードが 0 でない
#   * `WRONG n` / `PANIC n`（n≠0）を書いた
#   * `n miss(es)` / `n over the allowance`（n≠0）を書いた
#
# **3 つ目が、今回の 5 つを見逃した所**です。CI は 2 つ目しか見て
# いませんでした（門は自分で非ゼロを返すので CI でも落ちますが、
# **CI 自体が走っていません**）。
#
# **ここが緑でも、マージしてよいことにはなりません。** 本当の物差しは
# OpenCASCADE との突き合わせ（`tools/freecad_cross_validate.py`）で、
# それには FreeCAD 1.1 が要ります。

set -uo pipefail

cd "$(dirname "$0")/.."

QUICK=0
if [ "${1:-}" = "--quick" ]; then
  QUICK=1
fi

OUT="${TMPDIR:-/tmp}/zenith-gates"
mkdir -p "$OUT"

echo "門を通しで回します（4-411）"
echo

# **外部ファイルが揃っているかを、最初に見ます。**
# **「無い」は「緑」ではありません**——読めないファイルは飛ばす作りなので、
# 無いまま回すと、回っていない門を緑と読み違えます。
echo "== 要る外部ファイル =="
if ! cargo run --quiet --release -p zenith_algo --example external_data_probe; then
  echo
  echo "**外部ファイルが足りません。** 上に挙げた門は回りません。"
  echo "**それでも下の門は回します**——回った分だけが結果です。"
fi
echo

echo "== 通しテスト =="
if ! cargo test --release --workspace --exclude zenith_py > "$OUT/tests.txt" 2>&1; then
  echo "**通しテストが落ちました。** $OUT/tests.txt"
  tail -30 "$OUT/tests.txt"
  exit 1
fi
awk -F'[ ;]+' '/^test result:/{p+=$4; f+=$6; n++} END{printf "  %d 本 / %d 件通過 / %d 件失敗\n", n, p, f}' "$OUT/tests.txt"
echo

echo "== 門をつくります =="
if ! cargo build --release -p zenith_algo --examples > "$OUT/build.txt" 2>&1; then
  echo "**建ちません。** $OUT/build.txt"
  tail -30 "$OUT/build.txt"
  exit 1
fi

# **`target/validation` を読む門があります。** 他カーネルが書いた検体は
# 追跡下（`tests/fixtures`）にあるので、そこから置いてから書き出します。
mkdir -p target/validation
cp crates/zenith_algo/tests/fixtures/*.step target/validation/ 2>/dev/null || true
./target/release/examples/export_validation_suite > "$OUT/export.txt" 2>&1 || true
echo

GATES="builder_audit planar_face_audit boolean_topology_probe
mesh_watertight_probe slice_probe slice_robustness_probe
sketch_solver_probe pcurve_fidelity_probe inertia_probe
distance_probe interference_depth_probe regularize_probe
countersink_range_probe face_split_probe ssi_probe
boolean_gate_probe tess_density_probe step_import_audit
robustness_probe closure_probe foreign_boolean_probe
helix_volume_probe cutter_placement_probe cone_slab_probe
intersection_edge_probe shape_variety_probe march_stop_probe
gate_membership_probe foreign_distance_probe foreign_slice_probe
grid_fallback_probe foreign_edit_probe foreign_inertia_probe
curved_placement_probe oblique_section_probe foreign_cross_pair_probe
face_merge_probe step_representation_probe step_unit_probe
thicken_sheet_probe sketch_boolean_probe unused_builder_probe"

if [ "$QUICK" = "0" ]; then
  GATES="$GATES contact_placement_probe"
else
  echo '**--quick なので contact_placement_probe を外しました。**'
  echo "**外したものは「緑」ではありません**——回していないだけです。"
  echo
fi

echo "== 門 =="
fail=0
red=""
for probe in $GATES; do
  exe="./target/release/examples/$probe"
  if [ ! -x "$exe" ] && [ ! -f "$exe.exe" ]; then
    printf "  %-30s **ありません**\n" "$probe"
    continue
  fi
  started=$(date +%s)
  "$exe" > "$OUT/$probe.txt" 2>&1
  code=$?
  elapsed=$(( $(date +%s) - started ))

  # **判定の行**を見ます。**0 は健全**なので、1〜9 で始まる数だけを拾います。
  bad=$(grep -Eo "WRONG [1-9][0-9]*|PANIC [1-9][0-9]*|[1-9][0-9]* miss\(es\)|[1-9][0-9]* over the allowance" "$OUT/$probe.txt" | sort -u | tr '\n' ' ')

  if [ "$code" != "0" ] || [ -n "$bad" ]; then
    fail=1
    red="$red $probe"
    printf "  %-30s **赤**  exit=%s  %s  (%ss)\n" "$probe" "$code" "$bad" "$elapsed"
  else
    printf "  %-30s 緑    (%ss)\n" "$probe" "$elapsed"
  fi
done

# **Python の口も回します**（4-419）。
#
# **`cargo test` は、Python から使えることの証明になりません**（4-402）
# ——`volume` は Python では getter でした。**そして 2 つの道具は、
# 置いてあるだけで門に入っていませんでした**——**4-410 の「回って
# いない門」と同じ形**です。
#
# **Python か拡張モジュールが無ければ、赤にはせず、名前を出して飛ばします**
# ——**「無い」を「緑」と読み違えないため**に、行は必ず出します。
echo
echo "== Python の口 =="
PYTHON="${ZENITH_PYTHON:-}"
if [ -z "$PYTHON" ]; then
  for candidate in py python3 python; do
    if command -v "$candidate" > /dev/null 2>&1; then
      PYTHON="$candidate"
      break
    fi
  done
fi

if [ -z "$PYTHON" ]; then
  echo "  **Python がありません。** 下の 2 つは回していません（緑ではありません）。"
  echo "    tools/check_python_surface.py / tools/check_python_arguments.py"
elif ! PYO3_PYTHON="$("$PYTHON" -c 'import sys; print(sys.executable)')"         cargo build --release -p zenith_py > "$OUT/pybuild.txt" 2>&1; then
  echo "  **拡張モジュールが建ちません。** $OUT/pybuild.txt"
  fail=1
  red="$red zenith_py"
else
  # **配る包みが、いまのソースより新しいか**（4-426、4-431）。
  #
  # **2026/09/08 に「18 日古いまま」と分かりました**（4-405）。
  # **記録しただけで門を置かず**、**2026/09/12 に見たらまた 4 日古く**、
  # **4-409 から 4-425 までが 1 つも入っていません**でした。
  #
  # **包みは `.gitignore` の外**なので `git status` に出ず、
  # **どの門も `target/release` のほうを見ています**。**ここだけが、
  # 配る形を見ます。**
  #
  # **⚠ バイトで比べるのはやめました**（4-431）。**建て方が違うと
  # 中身が違います**——`tools/build_pyd.py` は `PYO3_PYTHON` を立てて
  # 建て、門は立てずに建てるので、**同じソースからでも大きさが違い**
  # （実測: 5842432 と 5747712）、**どちらが最後に建てたかで赤と緑が
  # 入れ替わりました。**
  #
  # **見たいのは「配る形が、いまのソースから作られたか」**です。
  # **ソースより新しければ緑**にします。
  PACKAGE="blender_addon/H-CAD_V_1_0_0/zenith_cad.pyd"
  if [ ! -f "$PACKAGE" ]; then
    printf "  %-30s **赤**  配る包みがありません（py tools/build_pyd.py）
" "配る包み"
    fail=1
    red="$red addon_package"
  else
    newest=$(find crates -name '*.rs' -newer "$PACKAGE" -print -quit 2>/dev/null)
    if [ -z "$newest" ]; then
      newest=$(find crates -name 'Cargo.toml' -newer "$PACKAGE" -print -quit 2>/dev/null)
    fi
    if [ -n "$newest" ]; then
      printf "  %-30s **赤**  %s より古い（py tools/build_pyd.py）
" "配る包み" "$newest"
      fail=1
      red="$red addon_package"
    else
      printf "  %-30s 緑    (ソースより新しい)
" "配る包み"
    fi
  fi

  # **書き出した glTF を、Blender に読ませます**（4-432）。
  #
  # **glTF だけ、他人に読ませていませんでした**（4-389 の積み残し）
  # ——FreeCAD の `Mesh` は断り、`trimesh` も `pygltflib` もこの環境に
  # 入っていないからです。**Blender の取り込みは Khronos が保守している
  # 実装**（`io_scene_gltf2`）で、**自前のパーサではありません。**
  #
  # **入れた初回に、向きの誤りが出ました**——**8 検体中 7 件で境界箱が
  # `(x, y, z)` → `(x, -z, y)` にずれ**、**球だけは対称なので合って**
  # いました。**glTF は +Y が上**（仕様 3.5）で、**z-up のまま書いて
  # いた**のです。**節点に回転を置いて直しました。**
  #
  # **Blender が無ければ、赤にはせず飛ばします**——**「無い」を
  # 「緑」と読み違えないため**に、**行は必ず出します。**
  BLENDER="${ZENITH_BLENDER:-}"
  if [ -z "$BLENDER" ]; then
    for candidate in "/c/Program Files/Blender Foundation/Blender 4.4/blender.exe"                      "/c/Program Files/Blender Foundation/Blender 4.3/blender.exe"                      "/c/Program Files/Blender Foundation/Blender 4.2/blender.exe"; do
      if [ -x "$candidate" ]; then
        BLENDER="$candidate"
        break
      fi
    done
  fi
  if [ -z "$BLENDER" ]; then
    printf "  %-30s 飛ばす（Blender がありません。緑ではありません）
" "glTF を Blender に"
  else
    # **検体は、ここで書き出します**——**門が自己完結していないと、
    # 「前に回したときの検体」を読むことになります**（5 章の
    # 「古い実行ファイル」と同じ一族）。
    ./target/release/examples/export_mesh_suite > "$OUT/mesh_exports.txt" 2>&1 || true
  fi
  # **配る包みを、Blender の中で読ませます**（4-433）。
  #
  # **4-400 が 3 つの段を分けました**——**カーネルにある → Python から
  # 呼べる → 包みに入っている**。**4 段目があります**——
  # **Blender の中で読める**。**そこは一度も測っていませんでした。**
  #
  # **`check_python_surface` はこちらの Python で読みます。**
  # **Blender は自前の Python を積んでいます**（版により 3.10 〜 3.13）。
  # **`abi3-py310` なので読めるはず**ですが、**「はず」で通してきました。**
  # **8 版で確かめました**（3.5 〜 5.1。Python 3.10.9 〜 3.13.9）。
  if [ -n "$BLENDER" ]; then
    started=$(date +%s)
    ZENITH_ROOT="$PWD" "$BLENDER" --background --factory-startup       --python tools/blender_load_addon.py > "$OUT/blender_addon.txt" 2>&1
    code=$?
    elapsed=$(( $(date +%s) - started ))
    if [ "$code" != "0" ]; then
      fail=1
      red="$red blender_addon_load"
      printf "  %-30s **赤**  exit=%s  (%ss)  %s
" "包みを Blender で読む" "$code" "$elapsed" "$OUT/blender_addon.txt"
    else
      printf "  %-30s 緑    (%ss)
" "包みを Blender で読む" "$elapsed"
    fi
  fi

  if [ -n "$BLENDER" ] && [ -f target/mesh_exports/manifest.json ]; then
    started=$(date +%s)
    "$BLENDER" --background --factory-startup       --python tools/blender_read_gltf_exports.py > "$OUT/blender_gltf.txt" 2>&1
    code=$?
    elapsed=$(( $(date +%s) - started ))
    if [ "$code" != "0" ]; then
      fail=1
      red="$red blender_gltf"
      printf "  %-30s **赤**  exit=%s  (%ss)  %s
" "glTF を Blender に" "$code" "$elapsed" "$OUT/blender_gltf.txt"
    else
      printf "  %-30s 緑    (%ss)
" "glTF を Blender に" "$elapsed"
    fi
  fi

  # **読ませる STEP は、追跡下の検体から選びます**（`reference/` は
  # 2026/09/08 に消えました）。
  export ZENITH_ARGS_STEP="${ZENITH_ARGS_STEP:-crates/zenith_algo/tests/fixtures/occ_reference_cylinder.step}"
  for script in check_python_surface check_python_arguments; do
    started=$(date +%s)
    "$PYTHON" "tools/$script.py" > "$OUT/$script.txt" 2>&1
    code=$?
    elapsed=$(( $(date +%s) - started ))
    if [ "$code" != "0" ]; then
      fail=1
      red="$red $script"
      printf "  %-30s **赤**  exit=%s  (%ss)
" "$script" "$code" "$elapsed"
    else
      printf "  %-30s 緑    (%ss)
" "$script" "$elapsed"
    fi
  done
fi

echo
if [ "$fail" = "0" ]; then
  echo "**門は全部緑です。**"
  echo "出力: $OUT"
  echo
  echo "**ただし、これだけでは足りません。** 本当の物差しは OpenCASCADE との"
  echo "突き合わせです——\`tools/freecad_cross_validate.py\`（FreeCAD 1.1 が要ります）。"
else
  echo "**赤:$red**"
  echo "出力: $OUT"
  echo
  echo "**どれか 1 つでも赤なら、\`main\` に持っていくものではありません。**"
fi
exit $fail
