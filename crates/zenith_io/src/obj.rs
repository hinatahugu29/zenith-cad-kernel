use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_tess::TriangleMesh;

/// Wavefront OBJ エクスポーター
pub struct ObjExporter;

impl ObjExporter {
    /// メッシュをOBJファイルとして書き出し
    pub fn export_to_file<P: AsRef<Path>>(
        mesh: &TriangleMesh,
        path: P,
        object_name: &str,
    ) -> std::io::Result<()> {
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let content = mesh.to_obj_string(object_name);
        let mut file = File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }
}
