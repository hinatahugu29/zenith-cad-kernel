#!/usr/bin/env bash
# **H8（`linkrods.step` の 3 演算）を、返る状態で回します**（4-577）。
#
# # なぜ要るのか
#
# **`linkrods` の 3 演算が返るのは、口 20 本を立てたときだけ**です
# （4-571）。**既定では 3 つとも断られます**——
#
#     $ cargo run --release -p zenith_algo --example h8_difference_probe -- difference
#     difference: 断られました: Exact B-Rep boolean is not implemented yet ...
#
# **その 20 本は、引継書の散らばった所（4-563 の 18 本 ＋ 4-571 の 2 本）に
# 文章として書かれているだけ**でした。**この計画の一番大きな結果が、
# 51,000 行の散文から env 名を 20 個拾い直さないと再現できない**という
# ことです。**それをここに固めます。**
#
# # 使い方
#
#     bash tools/h8_ports.sh                     # 3 演算ぜんぶ
#     bash tools/h8_ports.sh difference          # 1 つだけ
#     bash tools/h8_ports.sh union intersection  # 選んで
#
# **呼ぶ側の環境変数は残ります**ので、口を足して測れます——
#
#     ZENITH_TRIM_WINDING_BY_NEIGHBOR=1 bash tools/h8_ports.sh difference
#     ZENITH_H8_FACE_VOLUMES=1 bash tools/h8_ports.sh union
#
# # 出るはずの数（4-571。2026/09/28 の実測）
#
# | | 体積 | OCC（4-511） | 相対 |
# | :--- | :--- | :--- | :--- |
# | `A − B` | 1.551149 | 1.551124 | 1.59e-5 |
# | `A ∩ B` | 2.295747 | 2.295916 | 7.35e-5 |
# | `A ∪ B` | 7.805801 | 7.805669 | 1.69e-5 |
#
# **1 演算 3.5〜6 分**かかります。
#
# **既定を動かすものではありません。** ここで立てるのは、**この 1 回の
# 実行の中だけ**です。**既定化はリポジトリの持ち主の判断**です（4-563）。

set -u

# **4-563 の 18 本**（`SUBDIV_THIRD` は入れません——**断りを誤答に
# 変えるから**です。4-558）。
export ZENITH_SSI_EMPTY_RETRY=1
export ZENITH_SSI_EXTRA_BRANCHES=1
export ZENITH_WELD_ENDS=2e-4              # **値を取ります**（距離。4-524）
export ZENITH_LEFTOVER_LOOPS=1
export ZENITH_HOLE_CUT=1
export ZENITH_SPLIT_ONE_PIECE=1
export ZENITH_DEDUP_SELECTED=1
export ZENITH_ID_WELD=1
export ZENITH_HOLE_BOX_GUARD=1
export ZENITH_FACE_SNAP=1
export ZENITH_CHAIN_AREA_TOL=1
export ZENITH_HOLECUT_SIGN=1
export ZENITH_WRAPPED_AREA=1
export ZENITH_KEEP_MIXED_HOLES=1
export ZENITH_EDGE_TOL_FROM_FACES=1
export ZENITH_EDGE_TOL_BOUNDARY=1
export ZENITH_IMPORT_ROUGHNESS_SAMPLES=64 # **値を取ります**（本数）
export ZENITH_VERTEX_IMPRINT_TOL=1

# **4-571 で足した 2 本**——これで和が検証を通ります。
export ZENITH_CAP_ORIENT_BY_GEOMETRY=1
export ZENITH_ID_WELD_PLANES=1

operations=("$@")
if [ ${#operations[@]} -eq 0 ]; then
  operations=(difference union intersection)
fi

status=0
for operation in "${operations[@]}"; do
  echo "=== $operation ==="
  # **積だけ `SUBDIV_THIRD` が要ります**（4-571）。**ほかの演算では、
  # 断りを誤答に変えるので立てません**（4-558）。
  if [ "$operation" = "intersection" ]; then
    ZENITH_SUBDIV_THIRD=1 cargo run --release -q -p zenith_algo \
      --example h8_difference_probe -- "$operation" || status=1
  else
    cargo run --release -q -p zenith_algo \
      --example h8_difference_probe -- "$operation" || status=1
  fi
  echo
done

exit "$status"
