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
elif ! cargo build --release -p zenith_py > "$OUT/pybuild.txt" 2>&1; then
  echo "  **拡張モジュールが建ちません。** $OUT/pybuild.txt"
  fail=1
  red="$red zenith_py"
else
  # **配る包みが、いま建てたものと同じか**（4-426）。
  #
  # **2026/09/08 に「18 日古いまま」と分かりました**（4-405）。
  # **記録しただけで、門を置きませんでした**——**2026/09/12 に見たら、
  # また 4 日古く**、**4-409 から 4-425 までが 1 つも入っていません**
  # でした（**プロセスごと落ちる欠陥の直しも、法線の直しも**）。
  #
  # **包みは `.gitignore` の外**なので `git status` に出ず、
  # **どの門も `target/release` のほうを見ています**。**ここだけが、
  # 配る形を見ます。**
  PACKAGE="blender_addon/H-CAD_V_1_0_0/zenith_cad.pyd"
  BUILT="target/release/zenith_cad.dll"
  if [ ! -f "$BUILT" ]; then
    BUILT="target/release/libzenith_cad.so"
  fi
  if [ ! -f "$PACKAGE" ]; then
    printf "  %-30s **赤**  配る包みがありません（py tools/build_pyd.py）
" "配る包み"
    fail=1
    red="$red addon_package"
  elif ! cmp -s "$PACKAGE" "$BUILT"; then
    printf "  %-30s **赤**  いま建てたものと中身が違います（py tools/build_pyd.py）
" "配る包み"
    fail=1
    red="$red addon_package"
  else
    printf "  %-30s 緑    (同じ中身)
" "配る包み"
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
