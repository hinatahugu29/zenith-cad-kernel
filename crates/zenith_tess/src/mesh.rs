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
