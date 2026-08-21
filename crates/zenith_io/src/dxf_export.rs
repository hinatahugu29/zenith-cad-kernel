//! Zenith IO: 2D DXF 図面エクスポート (DXF Exporter)
//!
//! 3D断面スライス結果の閉ポリライン群を標準AutoCAD DXF形式（LWPOLYLINE）でファイル出力します。

use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_math::Point3;

pub struct DxfExporter;

impl DxfExporter {
    /// 3D断面ループ群（X-Y平面射影）を DXF 形式の文字列として生成
    pub fn generate_dxf_string(loops: &[Vec<Point3>]) -> String {
        let mut out = String::new();
        out.push_str("0\nSECTION\n2\nHEADER\n0\nENDSEC\n");
        out.push_str("0\nSECTION\n2\nENTITIES\n");

        for loop_pts in loops {
            if loop_pts.len() < 2 {
                continue;
            }

            out.push_str("0\nLWPOLYLINE\n");
            out.push_str("100\nAcDbEntity\n");
            out.push_str("8\n0\n"); // レイヤー 0
            out.push_str("100\nAcDbPolyline\n");
            out.push_str(&format!("90\n{}\n", loop_pts.len()));
            out.push_str("70\n1\n"); // 閉じたポリライン

            for pt in loop_pts {
                out.push_str(&format!("10\n{:.6}\n", pt.x));
                out.push_str(&format!("20\n{:.6}\n", pt.y));
            }
        }

        out.push_str("0\nENDSEC\n0\nEOF\n");
        out
    }

    /// 断面ループ群を DXF ファイルに書き出し
    pub fn export_loops_to_file<P: AsRef<Path>>(loops: &[Vec<Point3>], path: P) -> Result<(), String> {
        let content = Self::generate_dxf_string(loops);
        let mut file = File::create(path).map_err(|e| format!("Failed to create DXF file: {e}"))?;
        file.write_all(content.as_bytes())
            .map_err(|e| format!("Failed to write DXF content: {e}"))?;
        Ok(())
    }
}
