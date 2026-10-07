import { readFileSync } from "node:fs";
const path = process.argv[2];
const bytes = readFileSync(path);
const mod = await WebAssembly.compile(bytes);
const needs = WebAssembly.Module.imports(mod);
console.log("外から要るもの:", needs.length === 0 ? "なし" : JSON.stringify(needs));
const imports = {};
for (const { module, name } of needs) {
  imports[module] ??= {};
  imports[module][name] = (...a) => { throw new Error(`呼ばれました: ${module}.${name}`); };
}
const inst = await WebAssembly.instantiate(mod, imports);
const ex = inst.exports;
console.log("export:", Object.keys(ex).filter(k => k.startsWith("zenith")).join(", "));
{
  const t0 = process.hrtime.bigint();
  const v = ex.zenith_wasm_embedded_step();
  const ms = Number(process.hrtime.bigint() - t0) / 1e6;
  console.log(`  埋め込んだ STEP: 立体 ${Math.floor(v / 1000)} 個 / 面 ${v % 1000} 枚  ${ms.toFixed(1)} ms`);
}
{
  const t0 = process.hrtime.bigint();
  const faces = ex.zenith_wasm_big_subject_faces();
  const ms = Number(process.hrtime.bigint() - t0) / 1e6;
  console.log(`  実寸の検体 linkrods: 面 ${faces} 枚  ${ms.toFixed(0)} ms`);
}
for (const n of [8, 16]) {
  const t0 = process.hrtime.bigint();
  const v = ex.zenith_wasm_big_subject_volume(n);
  const ms = Number(process.hrtime.bigint() - t0) / 1e6;
  const mb = ex.memory ? (ex.memory.buffer.byteLength / 1048576).toFixed(1) : "?";
  console.log(`  linkrods 刻み ${n}: 体積*1000 = ${v}  ${ms.toFixed(0)} ms  メモリ ${mb} MB`);
}
for (const n of [8, 16, 24]) {
  const t0 = process.hrtime.bigint();
  const tris = ex.zenith_wasm_box_difference(n);
  const ms = Number(process.hrtime.bigint() - t0) / 1e6;
  console.log(`  刻み ${n}: 三角形 ${tris} 枚  ${ms.toFixed(1)} ms`);
}
