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
# **通しテストを飛ばす口**（4-633）。**1 回に回せる時間に上限がある機械**
# （背景の走行が 30 分で止められ、端末タブも使えない）で、**門を 2 回に
# 分けて回すため**です——**通しテストだけ別に回し、ここでは門だけ回します。**
#
# **これは「緑」を名乗れません。** **判定の行にも「通しテストを回して
# いません」と書きます**——**飛ばしたことが判定に残らないと、読み違えます**
# （`check_doc_claims` が Python 無しのとき「緑ではありません」と書くのと
# 同じ理屈）。
NO_TESTS=0
for arg in "$@"; do
  case "$arg" in
    --quick) QUICK=1 ;;
    --no-tests) NO_TESTS=1 ;;
    *) echo "知らない引数です: $arg（--quick と --no-tests だけ受けます）"; exit 2 ;;
  esac
done

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

# **飛ばしたものを、画面の外にも残します**（4-660）——**画面には
# 「緑ではありません」と出るのに、`verdict.txt` には「緑（全部）」と
# 書いていました**。**判定を読むだけの人には、飛ばしたことが見えません。**
skipped=""

echo "== 通しテスト =="
if [ "$NO_TESTS" = "1" ]; then
  echo "  **--no-tests なので回していません。** **これは緑ではありません。**"
  skipped="$skipped 通しテスト(--no-tests)"
  echo "  別に回してください: cargo test --release --workspace --exclude zenith_py"
  echo
elif ! cargo test --release --workspace --exclude zenith_py > "$OUT/tests.txt" 2>&1; then
  echo "**通しテストが落ちました。** $OUT/tests.txt"
  tail -30 "$OUT/tests.txt"
  exit 1
fi
if [ "$NO_TESTS" != "1" ]; then
  awk -F'[ ;]+' '/^test result:/{p+=$4; f+=$6; n++} END{printf "  %d 本 / %d 件通過 / %d 件失敗\n", n, p, f}' "$OUT/tests.txt"
  echo
fi

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
# **終了コードを捨てません**（4-659）——ここが黙って失敗すると、
# **下の門は古い `target/validation` を読んで緑で通ります**。
# **「0 件書けた」も失敗です**（4-656 と同じ穴）。
if ! ./target/release/examples/export_validation_suite > "$OUT/export.txt" 2>&1; then
  echo "WRONG 1 export_validation_suite が落ちました（rc 非 0）"
  tail -20 "$OUT/export.txt"
  exit 1
fi
wrote=$(grep -oE "^wrote [0-9]+ subject\(s\)" "$OUT/export.txt" | tail -1 | awk "{print \$2}")
if [ -z "$wrote" ] || [ "$wrote" -eq 0 ]; then
  echo "WRONG 1 export_validation_suite が検体を 1 件も書きませんでした（\"$wrote\"）"
  tail -20 "$OUT/export.txt"
  exit 1
fi
echo "export_validation_suite: $wrote 件書きました"
echo

GATES="builder_audit planar_face_audit boolean_topology_probe
mesh_watertight_probe import_closure_probe slice_probe slice_robustness_probe
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
thicken_sheet_probe sketch_boolean_probe unused_builder_probe
tangent_sweep_probe cylinder_tangency_sweep_probe
sphere_tangency_sweep_probe sphere_cap_sweep_probe
cone_apex_sweep_probe torus_plane_sweep_probe
cross_cylinder_sweep_probe curved_pair_identity_probe
aspect_sweep_probe curved_pair_placement_probe"

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

# **口を立てて回す門**（4-610）。
#
# **上の門は、呼ばれたときの環境のまま回します。** **既定では巻き方は
# 揃っていない**（4-582、4-602）ので、**巻き方の門をあの一覧に入れると、
# 既定の回が赤になります**——**それは退行ではなく、既定の姿**です。
#
# **なので、ここで口を立てて回します。** **掃き出しの側も、口が立って
# いなければ測らずに赤**にします（**立て忘れて緑が、いちばん悪い**）。
echo
echo "== 口を立てて回す門 =="
for probe in winding_contract_probe; do
  exe="./target/release/examples/$probe"
  if [ ! -x "$exe" ] && [ ! -f "$exe.exe" ]; then
    printf "  %-30s **ありません**\n" "$probe"
    continue
  fi
  started=$(date +%s)
  ZENITH_TRIM_WINDING_AFTER_WELD=1 ZENITH_NO_SPLIT_FLAT=1 \
    "$exe" > "$OUT/$probe.txt" 2>&1
  code=$?
  elapsed=$(( $(date +%s) - started ))
  if [ "$code" != "0" ]; then
    fail=1
    red="$red $probe"
    printf "  %-30s **赤**  exit=%s  (%ss)\n" "$probe" "$code" "$elapsed"
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
# **文書の「完了」が、実体を指しているか**（4-440）。
#
# **2026/09/12 に、実体の無い「完了」が見つかりました**（4-439）——
# **`blender_addon::zenith_patch_addon.py` は、どこにもありません**
# でした。**同じ形を、ここで数えます。**
#
# **Python は要りません**（crates の中を読むだけ）ので、
# **Python の口より前**に置きます。
echo
echo "== 門に床があるか =="
# **門を足したのに床を置き忘れたら、ここで赤**（4-661）。
# **今夜 13 本ためたのは、これが無かったから**です——
# **「数えるだけ」の監査は、次の 1 本について何も言いません。**
started=$(date +%s)
if bash tools/audit_gate_redness.sh --gate > "$OUT/gate_floors.txt" 2>&1; then
  printf "  %-30s 緑    (%ss)
" "audit_gate_redness --gate" "$(( $(date +%s) - started ))"
else
  fail=1
  red="$red gate_floors"
  printf "  %-30s **赤**  (%ss)  %s
" "audit_gate_redness --gate" "$(( $(date +%s) - started ))" "$OUT/gate_floors.txt"
  tail -4 "$OUT/gate_floors.txt"
fi

echo
echo "== 文書の指し先 =="
DOC_PYTHON="${ZENITH_PYTHON:-}"
if [ -z "$DOC_PYTHON" ]; then
  for candidate in py python3 python; do
    if command -v "$candidate" > /dev/null 2>&1; then
      DOC_PYTHON="$candidate"
      break
    fi
  done
fi
if [ -z "$DOC_PYTHON" ]; then
  echo "  **Python がありません。** check_doc_claims は回していません（緑ではありません）。"
  skipped="$skipped check_doc_claims(Python なし)"
else
  started=$(date +%s)
  "$DOC_PYTHON" tools/check_doc_claims.py > "$OUT/doc_claims.txt" 2>&1
  code=$?
  elapsed=$(( $(date +%s) - started ))
  if [ "$code" != "0" ]; then
    fail=1
    red="$red doc_claims"
    printf "  %-30s **赤**  exit=%s  (%ss)  %s
" "check_doc_claims" "$code" "$elapsed" "$OUT/doc_claims.txt"
  else
    printf "  %-30s 緑    (%ss)  %s
" "check_doc_claims" "$elapsed"       "$(grep -o '当てた名前 [0-9]* 件' "$OUT/doc_claims.txt" | head -1)"
  fi
fi

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
  skipped="$skipped check_python_surface/check_python_arguments(Python なし)"
  echo "    tools/check_python_surface.py / tools/check_python_arguments.py"
elif ! PYO3_PYTHON="$("$PYTHON" -c 'import sys; print(sys.executable)')"         cargo build --release -p zenith_py > "$OUT/pybuild.txt" 2>&1; then
  echo "  **拡張モジュールが建ちません。** $OUT/pybuild.txt"
  fail=1
  red="$red zenith_py"
else
  # **配る包みが、いまのソースから作られたか**（4-426、4-431、4-620）。
  # **4-620 から、更新時刻ではなく中身の指紋で見ます。**
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
    # **見るのは、拡張に入るソースだけ**です（4-437）。
    #
    # **`examples/` と `tests/` は、拡張に入りません**——**そこを
    # 触っただけで赤になるのは、嘘**です。**実際に 1 度なりました**
    # （`seam_torus_wall_probe.rs` を直したら「包みが古い」と言われた）。
    # **まず中身の指紋で見ます**（4-620）。
    #
    # **更新時刻だけで見ていたので、`git checkout` が中身の同じファイルを
    # 書き直すだけで赤**になりました（**1 日に 2 度**。4-615、4-619）。
    # **嘘の赤は「赤を無視する癖」をつけるので、本当の赤より害がある**
    # ことがあります。
    #
    # **指紋が無い（まだ作り直していない）ときだけ、更新時刻に戻ります。**
    STAMP="blender_addon/H-CAD_V_1_0_0/zenith_cad.sources.sha256"
    newest=""
    if [ -n "$DOC_PYTHON" ] && [ -f "$STAMP" ]; then
      want=$(cat "$STAMP" 2>/dev/null | tr -d '[:space:]')
      have=$("$DOC_PYTHON" tools/source_fingerprint.py 2>/dev/null | tr -d '[:space:]')
      if [ -n "$have" ] && [ "$want" != "$have" ]; then
        newest="指紋ちがい"
      fi
    else
      newest=$(find crates -name '*.rs' -newer "$PACKAGE"                  -not -path '*/examples/*' -not -path '*/tests/*'                  -print -quit 2>/dev/null)
      if [ -z "$newest" ]; then
        newest=$(find crates -name 'Cargo.toml' -newer "$PACKAGE" -print -quit 2>/dev/null)
      fi
    fi
    if [ "$newest" = "指紋ちがい" ]; then
      # **指紋がちがうときは「古い」とは言いません**（4-625）。
      # **時刻ではなく中身を見ている**ので、**どちらが新しいかは分かりません。**
      printf "  %-30s **赤**  ソースの指紋がちがいます（包みを作り直してください: py tools/build_pyd.py）
" "配る包み"
      fail=1
      red="$red addon_package"
    elif [ -n "$newest" ]; then
      printf "  %-30s **赤**  %s より古い（py tools/build_pyd.py）
" "配る包み" "$newest"
      fail=1
      red="$red addon_package"
    else
      printf "  %-30s 緑    (ソースの指紋と一致)
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
    skipped="$skipped glTF を Blender に(Blender なし)"
  else
    # **検体は、ここで書き出します**——**門が自己完結していないと、
    # 「前に回したときの検体」を読むことになります**（5 章の
    # 「古い実行ファイル」と同じ一族）。
    # **ここも終了コードを捨てません**（4-659）。**0 ファイルも失敗です。**
    if ! ./target/release/examples/export_mesh_suite > "$OUT/mesh_exports.txt" 2>&1; then
      echo "WRONG 1 export_mesh_suite が落ちました（rc 非 0）"
      tail -20 "$OUT/mesh_exports.txt"
      exit 1
    fi
    nfiles=$(find target/mesh_exports -type f 2>/dev/null | wc -l)
    if [ "$nfiles" -eq 0 ]; then
      echo "WRONG 1 export_mesh_suite が target/mesh_exports に 1 つも書きませんでした"
      exit 1
    fi
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
# **判定そのものも、出力の隣に残します**（4-579）。**1 本ずつの詳しい出力は
# `$OUT` に残るのに、「緑だったのか赤だったのか」は画面にしか出ていません**
# でした——**画面を失うと、80 分回した結果が分からなくなります**（実際に
# 失いました。`/tmp/*.log` が消える機械でした）。**`verdict.txt` を読めば
# 分かるように**しておきます。
verdict="$OUT/verdict.txt"
{
  echo "回した時刻: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "rc: $fail"
  if [ "$fail" != "0" ]; then
    echo "判定: 赤"
    echo "赤いもの:$red"
  elif [ -n "$skipped" ]; then
    echo "判定: 門は緑／**飛ばした項目があります**——**「全部緑」ではありません**"
  else
    echo "判定: 緑（全部）"
  fi
  # **飛ばしたものは、赤でも緑でも必ず 1 行出します**（4-660）。
  echo "飛ばしたもの:${skipped:- なし}"
  # **どの口を立てて回したか**——**これが分からないと、数字が再現できません**
  # （4-577）。
  echo "立っていた ZENITH_ の口:"
  env | grep -E '^ZENITH_' | sort | sed 's/^/  /' || echo "  （なし)"
} > "$verdict" 2>&1

if [ "$fail" = "0" ] && [ -n "$skipped" ]; then
  echo "**門は緑です——ただし飛ばした項目があります:$skipped**"
  echo "**「全部緑」ではありません。** 飛ばしたものを別に回して、両方で見てください。"
  echo "出力: $OUT（判定は $verdict）"
elif [ "$fail" = "0" ]; then
  echo "**門は全部緑です。**"
  echo "出力: $OUT（判定は $verdict）"
  echo
  echo "**ただし、これだけでは足りません。** 本当の物差しは OpenCASCADE との"
  echo "突き合わせです——\`tools/freecad_cross_validate.py\`（FreeCAD 1.1 が要ります）。"
else
  echo "**赤:$red**"
  echo "出力: $OUT（判定は $verdict）"
  echo
  echo "**どれか 1 つでも赤なら、\`main\` に持っていくものではありません。**"
fi
exit $fail
