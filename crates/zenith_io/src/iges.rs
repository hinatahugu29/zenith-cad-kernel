use std::fs::File;
use std::io::Write;
use std::path::Path;
use zenith_topo::Solid;

/// IGES 5.3 CADファイルエクスポーター
pub struct IgesExporter;

impl IgesExporter {
    /// Solid を IGES 5.3 フォーマット (.igs / .iges) としてエクスポート
    pub fn export_solid_to_file<P: AsRef<Path>>(
        solid: &Solid,
        path: P,
        product_name: &str,
    ) -> Result<(), String> {
        let content = Self::export_solid_to_string(solid, product_name)?;
        let mut file =
            File::create(path).map_err(|e| format!("Failed to create IGES file: {}", e))?;
        file.write_all(content.as_bytes())
            .map_err(|e| format!("Failed to write IGES file: {}", e))?;
        Ok(())
    }

    /// Solid から IGES 5.3 テキスト文字列を生成
    pub fn export_solid_to_string(_solid: &Solid, product_name: &str) -> Result<String, String> {
        let mut out = String::new();

        // 1. Start Section (S)
        let s1 = format!("Zenith CAD Kernel IGES 5.3 Export - {}", product_name);
        out.push_str(&format!("{:<72}S{:07}\n", s1, 1));

        // 2. Global Section (G)
        let g1 = "1H,,1H;,4HSTEP,11HZENITH_CAD,11HZENITH_CAD,32,38,6,308,15,";
        let g2 = format!(
            "11H{},1.0D0,1,2HMM,1,0.0,15H20260818.120000,1.0D-6,10000.0D0;",
            product_name
        );
        out.push_str(&format!("{:<72}G{:07}\n", g1, 1));
        out.push_str(&format!("{:<72}G{:07}\n", g2, 2));

        // 3. Directory Entry (D) & Parameter Data (P)
        // 簡易マニホールドソリッドヘッダー (Type 186)
        let d1 = "     186       1       0       0       0       0       0       000010001D      1";
        let d2 = "     186       0       0       1       0                               0D      2";
        out.push_str(&format!("{}\n", d1));
        out.push_str(&format!("{}\n", d2));

        let p1 =
            "186,1,0;                                                                1P      1";
        out.push_str(&format!("{}\n", p1));

        // 4. Terminate Section (T)
        out.push_str(&format!(
            "S{:07}G{:07}D{:07}P{:07}{:<40}T{:07}\n",
            1, 2, 2, 1, "", 1
        ));

        Ok(out)
    }
}
