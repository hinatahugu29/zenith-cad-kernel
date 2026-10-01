use serde::{Deserialize, Serialize};
use zenith_math::{Point3, Vec2, Vec3};

/// 三角形ポリゴンメッシュ（CADテッセレーション出力・Blender連携用）
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    pub positions: Vec<Point3>,
    pub normals: Vec<Vec3>,
    pub uvs: Vec<Vec2>,
    pub indices: Vec<[u32; 3]>,
}

impl TriangleMesh {
    pub fn new() -> Self {
        Self::default()
    }

    /// 頂点数の取得
    pub fn num_vertices(&self) -> usize {
        self.positions.len()
    }

    /// 三角形（ポリゴン）数の取得
    pub fn num_triangles(&self) -> usize {
        self.indices.len()
    }

    /// 別のメッシュをマージ
    pub fn merge(&mut self, other: &TriangleMesh) {
        let base_idx = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.normals.extend_from_slice(&other.normals);
        self.uvs.extend_from_slice(&other.uvs);
        for tri in &other.indices {
            self.indices
                .push([tri[0] + base_idx, tri[1] + base_idx, tri[2] + base_idx]);
        }
    }

    /// Wavefront OBJ 形式の文字列に変換（Blenderでの即時インポート・確認用）
    pub fn to_obj_string(&self, object_name: &str) -> String {
        let mut out = String::new();
        out.push_str(&format!("o {}\n", object_name));

        // 頂点位置
        for p in &self.positions {
            out.push_str(&format!("v {:.6} {:.6} {:.6}\n", p.x, p.y, p.z));
        }

        // テクスチャ座標 (UV)
        for uv in &self.uvs {
            out.push_str(&format!("vt {:.6} {:.6}\n", uv.x, uv.y));
        }

        // 法線
        for n in &self.normals {
            out.push_str(&format!("vn {:.6} {:.6} {:.6}\n", n.x, n.y, n.z));
        }

        // 三角形面
        for tri in &self.indices {
            let i0 = tri[0] + 1;
            let i1 = tri[1] + 1;
            let i2 = tri[2] + 1;
            if !self.uvs.is_empty() && !self.normals.is_empty() {
                out.push_str(&format!(
                    "f {}/{}/{} {}/{}/{} {}/{}/{}\n",
                    i0, i0, i0, i1, i1, i1, i2, i2, i2
                ));
            } else if !self.normals.is_empty() {
                out.push_str(&format!("f {}//{} {}//{} {}//{}\n", i0, i0, i1, i1, i2, i2));
            } else {
                out.push_str(&format!("f {} {} {}\n", i0, i1, i2));
            }
        }

        out
    }

    /// 二面角しきい値 (deg) に基づいて CAD 的な特徴エッジ（稜線）を高速抽出
    pub fn extract_feature_edges(&self, angle_deg: f64) -> Vec<[u32; 2]> {
        if self.positions.is_empty() || self.indices.is_empty() {
            return Vec::new();
        }

        use std::collections::HashMap;

        #[derive(Hash, PartialEq, Eq)]
        struct WeldKey(i64, i64, i64);
        let quant = 100000.0; // 0.01mm 精度で溶接
        let mut weld_map: HashMap<WeldKey, usize> = HashMap::with_capacity(self.positions.len());
        let mut remap: Vec<usize> = Vec::with_capacity(self.positions.len());

        for p in &self.positions {
            let key = WeldKey(
                (p.x * quant).round() as i64,
                (p.y * quant).round() as i64,
                (p.z * quant).round() as i64,
            );
            let next_idx = weld_map.len();
            let idx = *weld_map.entry(key).or_insert(next_idx);
            remap.push(idx);
        }

        // 各三角形の法線とエッジマップを構築
        let mut edge_faces: HashMap<(usize, usize), (Vec3, Option<Vec3>, [u32; 2])> =
            HashMap::with_capacity(self.indices.len() * 3);

        for tri in &self.indices {
            let a = self.positions[tri[0] as usize];
            let b = self.positions[tri[1] as usize];
            let c = self.positions[tri[2] as usize];
            let u = b - a;
            let v = c - a;
            let n = u.cross(&v);
            let len = n.norm();
            if len < 1e-12 {
                continue;
            }
            let norm = n / len;

            let w = [
                remap[tri[0] as usize],
                remap[tri[1] as usize],
                remap[tri[2] as usize],
            ];
            for i in 0..3 {
                let v0 = w[i];
                let v1 = w[(i + 1) % 3];
                if v0 == v1 {
                    continue;
                }
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                let orig_pair = [tri[i], tri[(i + 1) % 3]];

                match edge_faces.get_mut(&key) {
                    Some(slot) => {
                        if slot.1.is_none() {
                            slot.1 = Some(norm);
                        }
                    }
                    None => {
                        edge_faces.insert(key, (norm, None, orig_pair));
                    }
                }
            }
        }

        let cos_thr = angle_deg.to_radians().cos();
        let mut out = Vec::new();

        for (n0, n1_opt, orig_pair) in edge_faces.values() {
            match n1_opt {
                Some(n1) => {
                    let dot = n0.dot(n1);
                    if dot < cos_thr {
                        out.push(*orig_pair);
                    }
                }
                None => {
                    // 境界エッジ（1枚しか面が隣接していない）
                    out.push(*orig_pair);
                }
            }
        }

        out
    }

    /// 疑似 3 灯ランバートライティング（キーライト、フィルライト、リムライト＋アンビエント）を計算
    pub fn compute_shaded_colors(&self, base_rgb: [f32; 3], selected: bool) -> Vec<[f32; 4]> {
        let (mut r0, mut g0, mut b0) = (base_rgb[0], base_rgb[1], base_rgb[2]);
        if selected {
            r0 = (r0 * 1.25).min(1.0);
            g0 = (g0 * 1.05).min(1.0);
            b0 = (b0 * 0.75).min(1.0);
        }

        let lights: [([f32; 3], f32); 3] = [
            ([0.40, -0.55, 0.73], 0.85),  // キーライト
            ([-0.60, -0.35, 0.72], 0.35), // フィルライト
            ([0.10, 0.80, -0.59], 0.20),  // リムライト
        ];
        let ambient = 0.28f32;

        let has_normals = self.normals.len() == self.positions.len();
        let mut colors = Vec::with_capacity(self.positions.len());

        for i in 0..self.positions.len() {
            let lum = if has_normals {
                let n = self.normals[i];
                let (nx, ny, nz) = (n.x as f32, n.y as f32, n.z as f32);
                let mut l = ambient;
                for (d, w) in &lights {
                    let dot = nx * d[0] + ny * d[1] + nz * d[2];
                    if dot > 0.0 {
                        l += dot * w;
                    }
                }
                l.min(1.15)
            } else {
                1.0
            };
            colors.push([r0 * lum, g0 * lum, b0 * lum, 1.0]);
        }

        colors
    }
}

/// **その 3 点が、uv で一直線に潰れているか**（4-605）。
///
/// # なぜ 1 か所に集めるのか
///
/// **同じ式が 5 か所に散っていました**（細分の歯止め・2 つの診断・
/// 掃き出し 2 か所）。**この判定は、折り返しの正体そのもの**
/// （4-597）なので、**1 つの名前にして、試験を書いておきます。**
///
/// # 測り方
///
/// **符号つき面積を、三角形自身の広がり（いちばん長い辺）の 2 乗で
/// 正規化**して比べます。**絶対値で比べてはいけません**——
/// **面の大きさに引っ張られます**（`screw.step` の面 442 は差し渡し
/// 28.6、`linkrods` の小さい面は 0.1 の桁。同じ閾値は使えません）。
///
/// `limit` は比率です。**4-604 で 1e-11 〜 1e-4 の 4 桁を振って、
/// 答えが変わらないことを確かめてあります**（既定は 1e-9）。
///
/// **3 点が重なっている（広がりが 0）ときは `false`**を返します——
/// **それは「一直線」ではなく「点」**で、別の段
/// （`3d-zero`、`collapsed`）が落とします。
pub fn uv_triangle_is_flat(a: Vec2, b: Vec2, c: Vec2, limit: f64) -> bool {
    let signed = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    let scale = (b - a)
        .norm()
        .max((c - a).norm())
        .max((c - b).norm());
    scale > 0.0 && signed.abs() <= scale * scale * limit
}

#[cfg(test)]
mod flat_tests {
    use super::uv_triangle_is_flat;
    use zenith_math::Vec2;

    /// **一直線の 3 点は、潰れていると言えます。**
    #[test]
    fn three_points_on_one_line_are_flat() {
        let (a, b, c) = (
            Vec2::new(0.428035, 0.5),
            Vec2::new(0.432239, 0.5),
            Vec2::new(0.485418, 0.5),
        );
        assert!(uv_triangle_is_flat(a, b, c, 1e-9));
        // **4-604 で振った 4 桁すべてで同じ答え**になること。
        for limit in [1e-11, 1e-9, 1e-7, 1e-5] {
            assert!(
                uv_triangle_is_flat(a, b, c, limit),
                "limit {limit} で取りこぼしました"
            );
        }
    }

    /// **まともな三角形は、潰れていません。**
    #[test]
    fn a_healthy_triangle_is_not_flat() {
        let (a, b, c) = (
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
        );
        assert!(!uv_triangle_is_flat(a, b, c, 1e-9));
    }

    /// **細いが本物の三角形は、潰れていません。**
    ///
    /// **ここが効きます**——**絶対値で見ると「小さいから潰れている」に
    /// 見える**三角形が、**比率で見ると健全**だと分かります。
    #[test]
    fn a_thin_but_real_triangle_is_not_flat() {
        let (a, b, c) = (
            Vec2::new(0.0, 0.0),
            Vec2::new(1e-3, 0.0),
            Vec2::new(0.0, 1e-9),
        );
        // 面積は 5e-13 しかありませんが、広がり 1e-3 に対しては健全です。
        assert!(!uv_triangle_is_flat(a, b, c, 1e-9));
    }

    /// **3 点が重なっているときは「一直線」とは言いません。**
    #[test]
    fn a_single_point_is_not_called_flat() {
        let p = Vec2::new(0.25, 0.5);
        assert!(!uv_triangle_is_flat(p, p, p, 1e-9));
    }

    /// **大きい面でも小さい面でも、同じ閾値で使えます**（正規化の効き）。
    #[test]
    fn the_same_limit_works_on_large_and_small_faces() {
        for span in [1e-2, 1.0, 28.6] {
            let flat = (
                Vec2::new(0.0, 0.0),
                Vec2::new(span * 0.5, 0.0),
                Vec2::new(span, 0.0),
            );
            assert!(
                uv_triangle_is_flat(flat.0, flat.1, flat.2, 1e-9),
                "差し渡し {span} の一直線を取りこぼしました"
            );
            let healthy = (
                Vec2::new(0.0, 0.0),
                Vec2::new(span, 0.0),
                Vec2::new(0.0, span),
            );
            assert!(
                !uv_triangle_is_flat(healthy.0, healthy.1, healthy.2, 1e-9),
                "差し渡し {span} の健全な三角形を潰れていると言いました"
            );
        }
    }
}
