use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use zenith_tess::TriangleMesh;

/// 3Dプリント用 STL (Stereolithography) フォーマットエクスポーター
pub struct StlExporter;

impl StlExporter {
    /// メッシュをバイナリSTLファイルとして保存（3Dプリントスライサー標準）
    pub fn export_binary<P: AsRef<Path>>(mesh: &TriangleMesh, path: P) -> Result<(), String> {
        let file = File::create(path).map_err(|e| format!("Failed to create file: {}", e))?;
        let mut writer = BufWriter::new(file);

        // 1. 80バイトヘッダー
        let mut header = [0u8; 80];
        let desc = b"Zenith CAD Kernel Generated Binary STL";
        let len = desc.len().min(80);
        header[..len].copy_from_slice(&desc[..len]);
        writer.write_all(&header).map_err(|e| e.to_string())?;

        // 2. 三角形数 (u32, リトルエンディアン)
        let num_triangles = mesh.indices.len() as u32;
        writer
            .write_all(&num_triangles.to_le_bytes())
            .map_err(|e| e.to_string())?;

        // 3. 各ファセット (50バイト: 法線12B + 頂点1 12B + 頂点2 12B + 頂点3 12B + 属性2B)
        for tri in &mesh.indices {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0 = mesh.positions[i0];
            let p1 = mesh.positions[i1];
            let p2 = mesh.positions[i2];

            // 面法線の算出
            let v1 = p1 - p0;
            let v2 = p2 - p0;
            let n = v1.cross(&v2).normalize();
            let normal = if n.norm() > 1e-9 {
                n
            } else {
                zenith_math::Vec3::new(0.0, 0.0, 1.0)
            };

            // 法線 (f32 x 3)
            writer
                .write_all(&(normal.x as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(normal.y as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(normal.z as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;

            // 頂点0 (f32 x 3)
            writer
                .write_all(&(p0.x as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p0.y as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p0.z as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;

            // 頂点1 (f32 x 3)
            writer
                .write_all(&(p1.x as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p1.y as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p1.z as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;

            // 頂点2 (f32 x 3)
            writer
                .write_all(&(p2.x as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p2.y as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;
            writer
                .write_all(&(p2.z as f32).to_le_bytes())
                .map_err(|e| e.to_string())?;

            // 属性バイトカウント (u16, 0)
            writer
                .write_all(&0u16.to_le_bytes())
                .map_err(|e| e.to_string())?;
        }

        writer.flush().map_err(|e| e.to_string())?;
        Ok(())
    }

    /// メッシュをASCII STL文字列として生成
    pub fn export_ascii_string(mesh: &TriangleMesh, solid_name: &str) -> String {
        let mut out = format!("solid {}\n", solid_name);
        for tri in &mesh.indices {
            let p0 = mesh.positions[tri[0] as usize];
            let p1 = mesh.positions[tri[1] as usize];
            let p2 = mesh.positions[tri[2] as usize];
            let n = (p1 - p0).cross(&(p2 - p0)).normalize();

            out.push_str(&format!(
                "  facet normal {:.6e} {:.6e} {:.6e}\n    outer loop\n      vertex {:.6e} {:.6e} {:.6e}\n      vertex {:.6e} {:.6e} {:.6e}\n      vertex {:.6e} {:.6e} {:.6e}\n    endloop\n  endfacet\n",
                n.x, n.y, n.z, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
            ));
        }
        out.push_str(&format!("endsolid {}\n", solid_name));
        out
    }
}
