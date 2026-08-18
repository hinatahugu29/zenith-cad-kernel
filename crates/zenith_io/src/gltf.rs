use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_tess::TriangleMesh;

/// glTF 2.0 Web 3D エクスポーター
pub struct GltfExporter;

impl GltfExporter {
    /// TriangleMesh を単一の自己完結型 glTF 2.0 (.gltf) ファイルとしてエクスポート
    pub fn export_to_file<P: AsRef<Path>>(mesh: &TriangleMesh, path: P) -> Result<(), String> {
        let json_str = Self::export_to_json(mesh)?;
        let mut file =
            File::create(path).map_err(|e| format!("Failed to create glTF file: {}", e))?;
        file.write_all(json_str.as_bytes())
            .map_err(|e| format!("Failed to write glTF file: {}", e))?;
        Ok(())
    }

    /// TriangleMesh から glTF 2.0 JSON 文字列を生成
    pub fn export_to_json(mesh: &TriangleMesh) -> Result<String, String> {
        let num_verts = mesh.positions.len();
        let num_tris = mesh.indices.len();

        if num_verts == 0 || num_tris == 0 {
            return Err("Mesh is empty".to_string());
        }

        // バイトバッファの構築
        // 1. 位置座標 (Vec3: float32 x 3 = 12 bytes / vert)
        // 2. 法線ベクトル (Vec3: float32 x 3 = 12 bytes / vert)
        // 3. インデックス (uint32 x 3 = 12 bytes / tri)
        let mut bin_data = Vec::new();

        // Min / Max 計算
        let mut min_pos = [f32::INFINITY; 3];
        let mut max_pos = [f32::NEG_INFINITY; 3];

        let pos_offset = bin_data.len();
        for p in &mesh.positions {
            let x = p.x as f32;
            let y = p.y as f32;
            let z = p.z as f32;

            min_pos[0] = min_pos[0].min(x);
            min_pos[1] = min_pos[1].min(y);
            min_pos[2] = min_pos[2].min(z);
            max_pos[0] = max_pos[0].max(x);
            max_pos[1] = max_pos[1].max(y);
            max_pos[2] = max_pos[2].max(z);

            bin_data.extend_from_slice(&x.to_le_bytes());
            bin_data.extend_from_slice(&y.to_le_bytes());
            bin_data.extend_from_slice(&z.to_le_bytes());
        }
        let pos_length = bin_data.len() - pos_offset;

        let norm_offset = bin_data.len();
        for n in &mesh.normals {
            let x = n.x as f32;
            let y = n.y as f32;
            let z = n.z as f32;
            bin_data.extend_from_slice(&x.to_le_bytes());
            bin_data.extend_from_slice(&y.to_le_bytes());
            bin_data.extend_from_slice(&z.to_le_bytes());
        }
        let norm_length = bin_data.len() - norm_offset;

        let idx_offset = bin_data.len();
        for tri in &mesh.indices {
            bin_data.extend_from_slice(&tri[0].to_le_bytes());
            bin_data.extend_from_slice(&tri[1].to_le_bytes());
            bin_data.extend_from_slice(&tri[2].to_le_bytes());
        }
        let idx_length = bin_data.len() - idx_offset;

        // Base64 エンコード
        let base64_uri = format!(
            "data:application/octet-stream;base64,{}",
            base64_encode(&bin_data)
        );

        let json = format!(
            r#"{{
  "asset": {{
    "version": "2.0",
    "generator": "Zenith CAD Kernel glTF Exporter"
  }},
  "scene": 0,
  "scenes": [
    {{
      "nodes": [0]
    }}
  ],
  "nodes": [
    {{
      "mesh": 0,
      "name": "Zenith_Solid_Mesh"
    }}
  ],
  "meshes": [
    {{
      "primitives": [
        {{
          "attributes": {{
            "POSITION": 0,
            "NORMAL": 1
          }},
          "indices": 2,
          "mode": 4
        }}
      ]
    }}
  ],
  "accessors": [
    {{
      "bufferView": 0,
      "componentType": 5126,
      "count": {},
      "type": "VEC3",
      "min": [{:.6}, {:.6}, {:.6}],
      "max": [{:.6}, {:.6}, {:.6}]
    }},
    {{
      "bufferView": 1,
      "componentType": 5126,
      "count": {},
      "type": "VEC3"
    }},
    {{
      "bufferView": 2,
      "componentType": 5125,
      "count": {},
      "type": "SCALAR"
    }}
  ],
  "bufferViews": [
    {{
      "buffer": 0,
      "byteOffset": {},
      "byteLength": {},
      "target": 34962
    }},
    {{
      "buffer": 0,
      "byteOffset": {},
      "byteLength": {},
      "target": 34962
    }},
    {{
      "buffer": 0,
      "byteOffset": {},
      "byteLength": {},
      "target": 34963
    }}
  ],
  "buffers": [
    {{
      "byteLength": {},
      "uri": "{}"
    }}
  ]
}}"#,
            num_verts,
            min_pos[0],
            min_pos[1],
            min_pos[2],
            max_pos[0],
            max_pos[1],
            max_pos[2],
            num_verts,
            num_tris * 3,
            pos_offset,
            pos_length,
            norm_offset,
            norm_length,
            idx_offset,
            idx_length,
            bin_data.len(),
            base64_uri
        );

        Ok(json)
    }
}

fn base64_encode(data: &[u8]) -> String {
    const CHARSET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(data.len().div_ceil(3) * 4);

    for chunk in data.chunks(3) {
        let b0 = chunk[0];
        let b1 = if chunk.len() > 1 { chunk[1] } else { 0 };
        let b2 = if chunk.len() > 2 { chunk[2] } else { 0 };

        let triple = ((b0 as u32) << 16) | ((b1 as u32) << 8) | (b2 as u32);

        result.push(CHARSET[((triple >> 18) & 0x3F) as usize] as char);
        result.push(CHARSET[((triple >> 12) & 0x3F) as usize] as char);

        if chunk.len() > 1 {
            result.push(CHARSET[((triple >> 6) & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }

        if chunk.len() > 2 {
            result.push(CHARSET[(triple & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }
    }

    result
}
