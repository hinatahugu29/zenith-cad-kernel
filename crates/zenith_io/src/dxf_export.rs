//! Zenith IO: 2D DXF 図面エクスポート (DXF Exporter)
//!
//! 3D断面スライス結果の閉ポリライン群を標準AutoCAD DXF形式（LWPOLYLINE / CIRCLE / LINE）でファイル出力します。

use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_math::Point3;

/// DXF図面エンティティのレイヤー種別
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DxfLayer {
    /// 外形輪郭線 (VISIBLE_OUTLINE: 白色/黒色, 実線, 太線)
    Outline,
    /// 穴・内周境界 (HOLES: シアン色, 実線)
    Hole,
    /// 中心線・ピッチ円 (CENTER_LINES: 赤色, 一点鎖線)
    Centerline,
    /// ハッチング境界 (HATCH: 緑色, 細線)
    Hatch,
}

impl DxfLayer {
    pub fn name(&self) -> &'static str {
        match self {
            DxfLayer::Outline => "OUTLINE",
            DxfLayer::Hole => "HOLE",
            DxfLayer::Centerline => "CENTERLINE",
            DxfLayer::Hatch => "HATCH",
        }
    }

    pub fn color_number(&self) -> i32 {
        match self {
            DxfLayer::Outline => 7,    // 白/黒
            DxfLayer::Hole => 4,       // シアン
            DxfLayer::Centerline => 1, // 赤
            DxfLayer::Hatch => 3,      // 緑
        }
    }
}

pub struct DxfExporter;
impl DxfExporter {
    /// 3D断面ループ群（X-Y平面射影）を標準DXF文字列として生成
    ///
    /// **層は向きで決めます。索引ではありません。** `SectionSlicer` は外周を
    /// 反時計回り、穴を時計回りで返すので、X-Y へ落としたときの符号付き面積が
    /// 正なら外形、負なら穴です。
    ///
    /// 以前はここを「最初のループが外形、残りは全部穴」と決めていました。
    /// 断面に**外形が2つ以上**出る形（上から溝を掘った棒を溝の底より上で
    /// 切る、離れたボスを2本切る、など）では、2つ目以降の外形がすべて
    /// `HOLE` 層に落ちます。実測（`export_mesh_suite` の
    /// `slotted_bar_two_outlines`）では、外形2・穴0の断面が
    /// `OUTLINE` 1本・`HOLE` 1本として書かれていました。
    pub fn generate_dxf_string(loops: &[Vec<Point3>]) -> String {
        let layered: Vec<(DxfLayer, &[Point3])> = loops
            .iter()
            .map(|l| {
                let layer = if signed_area_xy(l) < 0.0 {
                    DxfLayer::Hole
                } else {
                    DxfLayer::Outline
                };
                (layer, l.as_slice())
            })
            .collect();
        Self::generate_dxf_string_layered(&layered)
    }

    /// レイヤー指定付きのポリライン群からDXF文字列を生成
    pub fn generate_dxf_string_layered(layers: &[(DxfLayer, &[Point3])]) -> String {
        let mut out = String::new();
        // 1. HEADER セクション
        out.push_str("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1015\n0\nENDSEC\n");

        // 2. TABLES セクション (レイヤー定義)
        out.push_str("0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n4\n");
        for layer in [
            DxfLayer::Outline,
            DxfLayer::Hole,
            DxfLayer::Centerline,
            DxfLayer::Hatch,
        ] {
            out.push_str("0\nLAYER\n100\nAcDbSymbolTableRecord\n100\nAcDbLayerTableRecord\n");
            out.push_str(&format!(
                "2\n{}\n70\n0\n62\n{}\n6\nCONTINUOUS\n",
                layer.name(),
                layer.color_number()
            ));
        }
        out.push_str("0\nENDTAB\n0\nENDSEC\n");

        // 3. BLOCKS セクション
        out.push_str("0\nSECTION\n2\nBLOCKS\n0\nENDSEC\n");

        // 4. ENTITIES セクション
        out.push_str("0\nSECTION\n2\nENTITIES\n");

        for (layer, loop_pts) in layers {
            if loop_pts.len() < 2 {
                continue;
            }

            out.push_str("0\nLWPOLYLINE\n");
            out.push_str("100\nAcDbEntity\n");
            out.push_str(&format!("8\n{}\n", layer.name()));
            out.push_str("100\nAcDbPolyline\n");
            out.push_str(&format!("90\n{}\n", loop_pts.len()));
            out.push_str("70\n1\n"); // 閉じたポリライン

            for pt in *loop_pts {
                out.push_str(&format!("10\n{:.6}\n", pt.x));
                out.push_str(&format!("20\n{:.6}\n", pt.y));
            }
        }

        out.push_str("0\nENDSEC\n0\nEOF\n");
        out
    }

    /// 断面ループ群を DXF ファイルに書き出し
    pub fn export_loops_to_file<P: AsRef<Path>>(
        loops: &[Vec<Point3>],
        path: P,
    ) -> Result<(), String> {
        let content = Self::generate_dxf_string(loops);
        let mut file = File::create(path).map_err(|e| format!("Failed to create DXF file: {e}"))?;
        file.write_all(content.as_bytes())
            .map_err(|e| format!("Failed to write DXF content: {e}"))?;
        Ok(())
    }
}

/// X-Y へ落としたときの符号付き面積（靴紐公式）。
///
/// 反時計回りが正。`SectionSlicer` はこの向きで外周を、逆向きで穴を返す。
fn signed_area_xy(points: &[Point3]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut twice = 0.0;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        twice += a.x * b.y - b.x * a.y;
    }
    twice * 0.5
}
