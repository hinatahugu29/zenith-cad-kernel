"""**配る包みの元になったソースの指紋を出します**（4-620）。

**なぜ要るのか**——**門は更新時刻で鮮度を見ていました**（`find -newer`）。
**`git checkout` / `git merge` は、中身が同じでもファイルを書き直す**ので、
**枝を行き来するだけで「包みが古い」と赤になります**。
**1 日に 2 度出ました**（4-615、4-619）。**嘘の赤は、読む人に
「赤を無視する癖」をつけるので、本当の赤より害がある**ことがあります。

**中身で見れば、書き直されても動きません。**

**数えるのは、拡張に入るソースだけ**です——**`examples/` と `tests/` は
入らない**ので外します（4-437 で、掃き出しを直しただけで赤になった）。
"""

import hashlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def shipped_sources():
    """**拡張に入るソースを、道順で並べて返します。**"""
    out = []
    for folder, _dirs, files in os.walk(os.path.join(ROOT, "crates")):
        parts = folder.replace("\\", "/").split("/")
        if "examples" in parts or "tests" in parts:
            continue
        for name in files:
            if name.endswith(".rs") or name == "Cargo.toml":
                out.append(os.path.join(folder, name))
    out.sort()
    return out


def fingerprint():
    """**道と中身の両方**を混ぜます（**名前を変えただけでも動くように**）。"""
    digest = hashlib.sha256()
    for path in shipped_sources():
        relative = os.path.relpath(path, ROOT).replace("\\", "/")
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        with open(path, "rb") as handle:
            # **改行は揃えます**——**`git` の設定で CRLF と LF が入れ替わる**
            # ことがあり、**中身は同じなのに指紋が動く**のを防ぎます。
            digest.update(handle.read().replace(b"\r\n", b"\n"))
        digest.update(b"\0")
    return digest.hexdigest()


if __name__ == "__main__":
    print(fingerprint())
    sys.exit(0)
