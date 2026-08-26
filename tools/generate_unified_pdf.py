"""
Zenith CAD Kernel Treatise & Architecture Comprehensive Guide
Professional Engineering & Theoretical Textbook + Project Engineering Log PDF Generator
"""

import os
import sys
import re
import json
import time
import base64
import subprocess
import urllib.request
import markdown

# 置き場はこのファイルの位置から決める。**以前は `r"e:\CAD-Kernel"` と
# 直書きしてあったので、別の場所へ clone した人の手元では動きませんでした。**
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TREATISE_DIR = os.path.join(WORKSPACE_DIR, "docs", "treatise")
OUTPUT_HTML_PATH = os.path.join(WORKSPACE_DIR, "tools", "unified_docs.html")
OUTPUT_PDF_PATH = os.path.join(WORKSPACE_DIR, "Zenith_CAD_Kernel_Documentation.pdf")

# ==============================================================================
# 解説・理論の本文は docs/treatise/ にある
#
# **以前はこのファイルの中に直書きしてありました。** 生成スクリプトの中に
# 読み物が埋まっていると、他の文書と食い違っても気づけません。本文は
# Markdown として `docs/treatise/` に置き、ここは組み立てだけをします。
# ==============================================================================

def load_treatise(name):
    """`docs/treatise/<name>.md` を読む。無ければ黙って空にせず止める。"""
    path = os.path.join(TREATISE_DIR, name + ".md")
    if not os.path.exists(path):
        raise SystemExit(
            "treatise section not found: {}\n"
            "本文は docs/treatise/ にあります。移動・改名したなら "
            "tools/generate_unified_pdf.py の対応も直してください。".format(path)
        )
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


PROLOGUE_MD = load_treatise("00-prologue")
CH1_THEORY_MD = load_treatise("01-brep-and-nurbs")
CH2_THEORY_MD = load_treatise("02-freeform-and-constraints")
CH3_THEORY_MD = load_treatise("03-boolean-and-ssi")
CH4_THEORY_MD = load_treatise("04-tessellation-and-validation")
CH5_THEORY_MD = load_treatise("05-occt-replacement")
CH6_THEORY_MD = load_treatise("06-step-interchange")
APPENDIX_A_MD = load_treatise("a1-glossary")
APPENDIX_B_MD = load_treatise("a2-references")

# ==============================================================================
# 章立て構成定義
# ==============================================================================

TREATISE_SECTIONS = [
    {
        "part_title": "第1部：CAD 幾何・トポロジー理論と Zenith の基礎構造",
        "part_id": "part-1",
        "chapters": [
            {
                "chapter_num": 1,
                "chapter_id": "chapter-1",
                "title": "B-Rep 多面体トポロジーと NURBS 幾何学の基礎理論 ＆ Zenith 実装仕様",
                "theory_md": CH1_THEORY_MD,
                "project_files": [
                    ("KERNEL_SPECS.md", "Zenith CAD Kernel スペック総覧・幾何トポロジー層", None, "### 3. 形状生成"),
                    ("KERNEL_INVENTORY_SPECS.md", "Zenith クレート別アーキテクチャ詳細仕様書", None, "## 2. クレート別")
                ]
            },
            {
                "chapter_num": 2,
                "chapter_id": "chapter-2",
                "title": "自由曲面パッチ理論・ダイレクトモデリング ＆ 幾何拘束ソルバー",
                "theory_md": CH2_THEORY_MD,
                "project_files": [
                    ("KERNEL_SPECS.md", "Zenith 自由曲面・ダイレクトモデリング層", "### 3. 形状生成", None),
                    ("KERNEL_INVENTORY_SPECS.md", "Zenith モデリング・フィーチャー詳細棚卸し", "## 2. クレート別", "## 3. テストスイート")
                ]
            }
        ]
    },
    {
        "part_title": "第2部：難関幾何アルゴリズムと実測ブレークスルー",
        "part_id": "part-2",
        "chapters": [
            {
                "chapter_num": 3,
                "chapter_id": "chapter-3",
                "title": "厳密 B-Rep ブーリアン・SSI 自由曲面交差 ＆ 接触配置規約",
                "theory_md": CH3_THEORY_MD,
                "project_files": [
                    ("KERNEL_AUDIT.md", "Zenith Kernel 実装リスク監査とB-Repブーリアン進化史", None, None)
                ]
            },
            {
                "chapter_num": 4,
                "chapter_id": "chapter-4",
                "title": "完全閉多様体テッセレーション・質量特性解析 ＆ 外部相互検証",
                "theory_md": CH4_THEORY_MD,
                "project_files": [
                    ("FREECAD_VALIDATION_REPORT.md", "FreeCAD 1.1 / OpenCASCADE ヘッドレス相互検証報告書", None, None)
                ]
            }
        ]
    },
    {
        "part_title": "第3部：システム統合・脱 OCCT 移行戦略とエコシステム連携",
        "part_id": "part-3",
        "chapters": [
            {
                "chapter_num": 5,
                "chapter_id": "chapter-5",
                "title": "脱 OCCT 置換戦略・Two-Lane アーキテクチャ ＆ 機能移行マトリクス",
                "theory_md": CH5_THEORY_MD,
                "project_files": [
                    ("KERNEL_REPLACEMENT_STRATEGY.md", "Seamless_CAD OCCT 置換戦略書", None, None),
                    ("MIGRATION_MAP.md", "Seamless_CAD 移行マップ＆互換 API マップ", None, None)
                ]
            },
            {
                "chapter_num": 6,
                "chapter_id": "chapter-6",
                "title": "STEP ISO 10303 規格適合・トポロジー共有 ＆ 外部 CAD データ交換",
                "theory_md": CH6_THEORY_MD,
                "project_files": []
            }
        ]
    },
    {
        "part_title": "第4部：エンジニアリング・実測検証・開発引継全履歴",
        "part_id": "part-4",
        "chapters": [
            {
                "chapter_num": 7,
                "chapter_id": "chapter-7",
                "title": "Zenith CAD Kernel 開発引継書（現在地点・作業思想・最優先課題 3-N）",
                "theory_md": "",
                "project_files": [
                    ("HANDOVER.md", "Zenith CAD Kernel 開発引継書 本文", None, None)
                ]
            }
        ]
    }
]

# ==============================================================================
# Markdown 前処理 & HTML 生成関数群
# ==============================================================================

def find_heading(lines, anchor, filename):
    """`anchor` で始まる行の位置を返す。無ければ止める。"""
    for index, line in enumerate(lines):
        if line.startswith(anchor):
            return index
    raise SystemExit(
        "{} に見出し {!r} が見つかりません。\n"
        "章の切れ目に使っているので、見出しを変えたなら "
        "tools/generate_unified_pdf.py の TREATISE_SECTIONS も直してください。"
        .format(filename, anchor)
    )


def slice_by_heading(lines, start_anchor, end_anchor, filename):
    """文書を**見出しで**切る。行番号では切らない。

    **以前はここが (開始行, 終了行) の直値でした。** 文書が伸びると末尾が黙って
    落ちます。2026年8月27日に測ったところ、

      | 文書 | 入っていた範囲 | 実際の行数 | 落ちていた行 |
      | :--- | ---: | ---: | ---: |
      | `HANDOVER.md` | 1〜8349 | 9012 | **663** |
      | `FREECAD_VALIDATION_REPORT.md` | 1〜234 | 265 | 31 |
      | `KERNEL_INVENTORY_SPECS.md` | 63〜205 | 228 | 23 |
      | `KERNEL_SPECS.md` | 77〜149 | 152 | 3 |

    でした。**PDF は普通に出来上がるので、読んだ人には分かりません。**
    見出しで切れば、本文が伸びても切れ目は同じ場所に付いていきます。
    """
    start = 0 if start_anchor is None else find_heading(lines, start_anchor, filename)
    end = len(lines) if end_anchor is None else find_heading(lines, end_anchor, filename)
    if end <= start:
        raise SystemExit(
            "{}: 見出し {!r} が {!r} より前にあります。切れ目の順序が逆です。"
            .format(filename, end_anchor, start_anchor)
        )
    return ''.join(lines[start:end])


def process_alerts(md_text):
    alert_types = {
        'NOTE': ('alert-note', 'ℹ️ NOTE'),
        'TIP': ('alert-tip', '💡 TIP'),
        'IMPORTANT': ('alert-important', '📌 IMPORTANT'),
        'WARNING': ('alert-warning', '⚠️ WARNING'),
        'CAUTION': ('alert-caution', '🛑 CAUTION'),
    }
    
    lines = md_text.split('\n')
    out_lines = []
    in_alert = False
    alert_class = ''
    alert_title = ''
    alert_body = []
    
    i = 0
    while i < len(lines):
        line = lines[i]
        alert_match = re.match(r'^>\s*\[!(NOTE|TIP|IMPORTANT|WARNING|CAUTION)\]\s*$', line.strip())
        if alert_match:
            atype = alert_match.group(1)
            alert_class, alert_title = alert_types[atype]
            in_alert = True
            alert_body = []
            i += 1
            continue
            
        if in_alert:
            if line.startswith('>'):
                content = re.sub(r'^>\s?', '', line)
                alert_body.append(content)
                i += 1
                continue
            else:
                body_md = '\n'.join(alert_body)
                out_lines.append(f'<div class="alert {alert_class}"><div class="alert-title">{alert_title}</div><div class="alert-content">\n\n{body_md}\n\n</div></div>')
                in_alert = False
                out_lines.append(line)
                i += 1
                continue
        else:
            out_lines.append(line)
            i += 1
            
    if in_alert:
        body_md = '\n'.join(alert_body)
        out_lines.append(f'<div class="alert {alert_class}"><div class="alert-title">{alert_title}</div><div class="alert-content">\n\n{body_md}\n\n</div></div>')
        
    return '\n'.join(out_lines)

def process_mermaid(md_text):
    pattern = re.compile(r'```mermaid\s*\n(.*?)```', re.DOTALL)
    def repl(m):
        code = m.group(1).strip()
        return f'<div class="mermaid">\n{code}\n</div>'
    return pattern.sub(repl, md_text)

def process_links(md_text):
    link_map = {
        'KERNEL_SPECS.md': '#chapter-1',
        'KERNEL_INVENTORY_SPECS.md': '#chapter-1',
        'KERNEL_REPLACEMENT_STRATEGY.md': '#chapter-5',
        'MIGRATION_MAP.md': '#chapter-5',
        'KERNEL_AUDIT.md': '#chapter-3',
        'FREECAD_VALIDATION_REPORT.md': '#chapter-4',
        'HANDOVER.md': '#chapter-7',
        'VERIFICATION_PLAYBOOK.md': '#chapter-7',
        'ROADMAP.md': '#chapter-5'
    }
    
    for filename, target in link_map.items():
        md_text = re.sub(rf'\[([^\]]+)\]\((?:file:///[^\)]+/)?{re.escape(filename)}(?:#[^\)]*)?\)', rf'[\1]({target})', md_text)
        md_text = re.sub(rf'`{re.escape(filename)}`', rf'[`{filename}`]({target})', md_text)
        
    return md_text

def build_treatise_toc():
    toc_html = ['<div class="toc-container" id="table-of-contents">', '<h2 class="toc-main-title">📑 総合目次 (Table of Contents)</h2>']
    
    toc_html.append('<div class="toc-prologue"><a href="#prologue" class="toc-part-link" style="background:#e8f0fe;border-left-color:#1a73e8;">巻頭言：現代 CAD カーネル工学と Zenith の挑戦</a></div>')
    
    for sec in TREATISE_SECTIONS:
        toc_html.append(f'<div class="toc-part"><a href="#{sec["part_id"]}" class="toc-part-link">{sec["part_title"]}</a>')
        toc_html.append('<ul class="toc-chapter-list">')
        for ch in sec["chapters"]:
            toc_html.append(f'<li class="toc-chapter-item"><a href="#{ch["chapter_id"]}" class="toc-chapter-link"><span class="toc-ch-num">第 {ch["chapter_num"]} 章</span> {ch["title"]}</a></li>')
        toc_html.append('</ul></div>')
        
    toc_html.append('<div class="toc-appendix"><a href="#appendix-a" class="toc-part-link" style="background:#f1f3f4;border-left-color:#5f6368;">巻末付録 (Appendix & Glossary)</a><ul class="toc-chapter-list"><li class="toc-chapter-item"><a href="#appendix-a" class="toc-chapter-link"><span class="toc-ch-num">付録 A</span> CAD 幾何・B-Rep・NURBS 専門用語集 (Glossary)</a></li><li class="toc-chapter-item"><a href="#appendix-b" class="toc-chapter-link"><span class="toc-ch-num">付録 B</span> 常設検証プローブ＆ベンチマーク便覧</a></li></ul></div>')
    
    toc_html.append('</div>')
    return '\n'.join(toc_html)

def generate_html():
    md_parser = markdown.Markdown(extensions=[
        'tables',
        'fenced_code',
        'def_list',
        'attr_list',
        'toc'
    ])
    
    all_content_html = []
    
    # 総合目次
    toc_html = build_treatise_toc()
    
    # 巻頭言
    md_parser.reset()
    prologue_processed = process_mermaid(process_alerts(PROLOGUE_MD))
    prologue_html = f'<section class="prologue-section" id="prologue">\n{md_parser.convert(prologue_processed)}\n</section>'
    all_content_html.append(prologue_html)
    
    # 各部・各章
    for sec in TREATISE_SECTIONS:
        part_banner = f'<div class="part-divider" id="{sec["part_id"]}"><h1>{sec["part_title"]}</h1></div>'
        all_content_html.append(part_banner)
        
        for ch in sec["chapters"]:
            chapter_html_parts = []
            
            # 章ヘッダー
            chapter_html_parts.append(f'<h2 id="{ch["chapter_id"]}" class="chapter-heading"><span class="chapter-number">第 {ch["chapter_num"]} 章</span> {ch["title"]}</h2>')
            
            # 1. 基礎理論セクション (General CAD Theory)
            if ch["theory_md"].strip():
                md_parser.reset()
                theory_proc = process_mermaid(process_alerts(ch["theory_md"]))
                theory_html = f'<div class="theory-box">\n<div class="theory-badge">📐 基礎理論・業界標準 (General CAD Theory)</div>\n{md_parser.convert(theory_proc)}\n</div>'
                chapter_html_parts.append(theory_html)
                
            # 2. プロジェクト特有の実装・実測セクション (Project-Specific Implementation)
            if ch["project_files"]:
                if ch["theory_md"].strip():
                    chapter_html_parts.append('<div class="impl-box-header"><div class="impl-badge">🦀 Zenith CAD Kernel における設計・実装と実測 (Project Implementation)</div></div>')
                
                for filename, label, start_anchor, end_anchor in ch["project_files"]:
                    file_path = os.path.join(WORKSPACE_DIR, filename)
                    if not os.path.exists(file_path):
                        raise SystemExit(
                            "project document not found: {}\n"
                            "以前はここで黙って読み飛ばしていました。章が丸ごと消えても "
                            "PDF は普通に出来上がるので、止めます。".format(file_path)
                        )

                    with open(file_path, 'r', encoding='utf-8') as f:
                        lines = f.readlines()

                    content_slice = slice_by_heading(lines, start_anchor, end_anchor, filename)
                    
                    # 前処理
                    processed = process_alerts(content_slice)
                    processed = process_mermaid(processed)
                    processed = process_links(processed)
                    
                    # 見出しシフト
                    proc_lines = []
                    for pline in processed.split('\n'):
                        if pline.startswith('###### '): proc_lines.append('###### ' + pline[7:])
                        elif pline.startswith('##### '): proc_lines.append('###### ' + pline[6:])
                        elif pline.startswith('#### '): proc_lines.append('##### ' + pline[5:])
                        elif pline.startswith('### '): proc_lines.append('#### ' + pline[4:])
                        elif pline.startswith('## '): proc_lines.append('### ' + pline[3:])
                        elif pline.startswith('# '): proc_lines.append('### ' + pline[2:])
                        else: proc_lines.append(pline)
                    
                    md_parser.reset()
                    file_html = md_parser.convert('\n'.join(proc_lines))
                    chapter_html_parts.append(f'<div class="project-doc-section">\n{file_html}\n</div>')
                    
            chapter_wrapper = f'<section class="chapter-section" id="section-{ch["chapter_id"]}">\n' + '\n'.join(chapter_html_parts) + '\n</section>'
            all_content_html.append(chapter_wrapper)
            
    # 巻末付録
    md_parser.reset()
    app_a_proc = process_alerts(APPENDIX_A_MD)
    app_a_html = f'<section class="appendix-section" id="appendix-a">\n{md_parser.convert(app_a_proc)}\n</section>'
    
    md_parser.reset()
    app_b_proc = process_alerts(APPENDIX_B_MD)
    app_b_html = f'<section class="appendix-section" id="appendix-b">\n{md_parser.convert(app_b_proc)}\n</section>'
    
    all_content_html.append(app_a_html)
    all_content_html.append(app_b_html)
    
    body_content = '\n'.join(all_content_html)
    
    full_html = f"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<title>Zenith CAD Kernel 総合技術書＆アーキテクチャ解説書</title>
<!-- Mermaid.js -->
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<!-- MathJax for TeX formulas -->
<script>
window.MathJax = {{
  tex: {{
    inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
    displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
    processEscapes: true
  }},
  options: {{
    skipHtmlTags: ['script', 'noscript', 'style', 'textarea', 'pre', 'code']
  }},
  startup: {{
    pageReady: () => {{
      return MathJax.startup.defaultPageReady().then(() => {{
        window.mathJaxReady = true;
        checkAllReady();
      }});
    }}
  }}
}};
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

<style>
/* ==========================================================================
   Base & Print Layout (A4 Optimized)
   ========================================================================== */
@page {{
  size: A4 portrait;
  margin: 20mm 15mm 20mm 15mm;
  @bottom-right {{
    content: counter(page);
    font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
    font-size: 9pt;
    color: #666;
  }}
  @bottom-left {{
    content: "Zenith CAD Kernel 総合技術書＆アーキテクチャ解説書";
    font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
    font-size: 8pt;
    color: #888;
  }}
}}

@page :first {{
  margin: 0;
  @bottom-right {{ content: normal; }}
  @bottom-left {{ content: normal; }}
}}

*, *::before, *::after {{
  box-sizing: border-box;
}}

body {{
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "Yu Gothic", "Meiryo", "Hiragino Kaku Gothic ProN", sans-serif;
  font-size: 10.5pt;
  line-height: 1.65;
  color: #24292f;
  background-color: #ffffff;
  margin: 0;
  padding: 0;
  -webkit-font-smoothing: antialiased;
  text-rendering: optimizeLegibility;
}}

/* ==========================================================================
   Cover Page (表紙)
   ========================================================================== */
.cover-page {{
  page-break-after: always;
  height: 100vh;
  min-height: 297mm;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  padding: 40mm 25mm 25mm 25mm;
  background: linear-gradient(145deg, #0d1117 0%, #161b22 50%, #0f2744 100%);
  color: #ffffff;
  box-sizing: border-box;
}}

.cover-header {{
  border-bottom: 2px solid #30363d;
  padding-bottom: 15px;
}}

.cover-badge {{
  display: inline-block;
  padding: 5px 12px;
  background: #1f6feb;
  color: #ffffff;
  font-size: 9pt;
  font-weight: 700;
  border-radius: 20px;
  letter-spacing: 1px;
  text-transform: uppercase;
  margin-bottom: 15px;
}}

.cover-title {{
  font-size: 26pt;
  font-weight: 800;
  line-height: 1.25;
  margin: 10px 0 15px 0;
  letter-spacing: -0.5px;
  background: linear-gradient(90deg, #ffffff, #58a6ff);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}}

.cover-subtitle {{
  font-size: 13pt;
  font-weight: 400;
  color: #8b949e;
  line-height: 1.5;
  margin: 0;
}}

.cover-body {{
  margin: 30px 0;
}}

.cover-highlights {{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 15px;
  margin-top: 20px;
}}

.cover-highlight-card {{
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.12);
  border-radius: 8px;
  padding: 15px;
}}

.cover-highlight-title {{
  font-size: 10.5pt;
  font-weight: 700;
  color: #58a6ff;
  margin-bottom: 5px;
}}

.cover-highlight-desc {{
  font-size: 9pt;
  color: #c9d1d9;
  line-height: 1.4;
}}

.cover-footer {{
  border-top: 1px solid #30363d;
  padding-top: 15px;
  display: flex;
  justify-content: space-between;
  align-items: flex-end;
  font-size: 8.5pt;
  color: #8b949e;
}}

.cover-meta-item {{
  margin-bottom: 4px;
}}

.cover-meta-item strong {{
  color: #c9d1d9;
}}

/* ==========================================================================
   Table of Contents (目次)
   ========================================================================== */
.toc-container {{
  page-break-before: always;
  page-break-after: always;
  padding: 10mm 0;
}}

.toc-main-title {{
  font-size: 18pt;
  font-weight: 800;
  border-bottom: 3px solid #1f6feb;
  padding-bottom: 8px;
  margin-bottom: 25px;
  color: #0d1117;
}}

.toc-part, .toc-prologue, .toc-appendix {{
  margin-bottom: 20px;
  break-inside: avoid;
}}

.toc-part-link {{
  font-size: 12pt;
  font-weight: 700;
  color: #0969da;
  text-decoration: none;
  display: block;
  background: #f6f8fa;
  padding: 8px 12px;
  border-left: 4px solid #0969da;
  border-radius: 0 4px 4px 0;
  margin-bottom: 8px;
}}

.toc-chapter-list {{
  list-style: none;
  padding-left: 10px;
  margin: 0;
}}

.toc-chapter-item {{
  margin: 8px 0;
  padding-left: 10px;
  border-left: 2px solid #e1e4e8;
}}

.toc-chapter-link {{
  font-size: 10pt;
  font-weight: 600;
  color: #24292f;
  text-decoration: none;
  display: inline-block;
}}

.toc-ch-num {{
  display: inline-block;
  padding: 2px 6px;
  background: #e1e4e8;
  color: #24292f;
  font-size: 8pt;
  font-weight: 700;
  border-radius: 4px;
  margin-right: 6px;
}}

/* ==========================================================================
   Theory Box & Implementation Box Styling
   ========================================================================== */
.theory-box {{
  background: #f8fafd;
  border: 1px solid #c8d8ea;
  border-radius: 8px;
  padding: 18px 20px;
  margin: 20px 0 25px 0;
  position: relative;
}}

.theory-badge {{
  display: inline-block;
  background: #0969da;
  color: #ffffff;
  font-size: 9pt;
  font-weight: 700;
  padding: 4px 12px;
  border-radius: 4px;
  margin-bottom: 12px;
  letter-spacing: 0.5px;
}}

.impl-box-header {{
  margin: 30px 0 15px 0;
}}

.impl-badge {{
  display: inline-block;
  background: #cf222e;
  color: #ffffff;
  font-size: 9.5pt;
  font-weight: 700;
  padding: 5px 14px;
  border-radius: 4px;
  letter-spacing: 0.5px;
}}

/* ==========================================================================
   Part & Chapter Headings
   ========================================================================== */
.part-divider {{
  page-break-before: always;
  padding: 60px 0 20px 0;
  text-align: center;
  border-bottom: 4px double #1f6feb;
  margin-bottom: 30px;
}}

.part-divider h1 {{
  font-size: 20pt;
  font-weight: 800;
  color: #0969da;
  letter-spacing: -0.5px;
  margin: 0;
}}

.prologue-section, .chapter-section, .appendix-section {{
  page-break-before: always;
  padding-top: 10px;
}}

.chapter-heading {{
  font-size: 16pt;
  font-weight: 800;
  color: #1f2328;
  border-bottom: 2px solid #d0d7de;
  padding-bottom: 8px;
  margin-top: 0;
  margin-bottom: 20px;
  display: flex;
  align-items: center;
  gap: 10px;
}}

.chapter-number {{
  display: inline-block;
  background: #0969da;
  color: #ffffff;
  font-size: 9.5pt;
  padding: 4px 10px;
  border-radius: 4px;
  font-weight: 700;
}}

h3 {{
  font-size: 13pt;
  font-weight: 700;
  color: #1f2328;
  border-left: 4px solid #1f6feb;
  padding-left: 10px;
  margin-top: 25px;
  margin-bottom: 12px;
  page-break-after: avoid;
}}

h4 {{
  font-size: 11.5pt;
  font-weight: 700;
  color: #24292f;
  margin-top: 20px;
  margin-bottom: 10px;
  page-break-after: avoid;
}}

h5, h6 {{
  font-size: 10.5pt;
  font-weight: 600;
  color: #333;
  margin-top: 15px;
  margin-bottom: 8px;
  page-break-after: avoid;
}}

/* ==========================================================================
   Typography & Flow Elements
   ========================================================================== */
p {{
  margin: 0 0 10px 0;
  text-align: justify;
}}

ul, ol {{
  margin: 0 0 12px 0;
  padding-left: 24px;
}}

li {{
  margin-bottom: 4px;
}}

li > p {{
  margin-bottom: 4px;
}}

hr {{
  border: 0;
  height: 1px;
  background-color: #d0d7de;
  margin: 24px 0;
}}

a {{
  color: #0969da;
  text-decoration: none;
}}

a:hover {{
  text-decoration: underline;
}}

/* ==========================================================================
   Tables
   ========================================================================== */
table {{
  width: 100%;
  border-collapse: collapse;
  margin: 15px 0 20px 0;
  font-size: 9pt;
  line-height: 1.45;
  page-break-inside: auto;
}}

tr {{
  page-break-inside: avoid;
  page-break-after: auto;
}}

thead {{
  display: table-header-group;
}}

th, td {{
  border: 1px solid #d0d7de;
  padding: 6px 10px;
  text-align: left;
  vertical-align: top;
  word-break: break-word;
}}

th {{
  background-color: #f6f8fa;
  font-weight: 700;
  color: #24292f;
}}

tbody tr:nth-child(even) {{
  background-color: #fafbfc;
}}

/* ==========================================================================
   Code & Preformatted
   ========================================================================== */
code {{
  font-family: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, "Liberation Mono", monospace;
  font-size: 8.5pt;
  background-color: #eff1f3;
  padding: 0.15em 0.35em;
  border-radius: 4px;
  color: #bf3989;
}}

pre {{
  font-family: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, "Liberation Mono", monospace;
  font-size: 8.5pt;
  line-height: 1.45;
  background-color: #f6f8fa;
  border: 1px solid #d0d7de;
  border-radius: 6px;
  padding: 12px;
  overflow-x: auto;
  margin: 12px 0 16px 0;
  page-break-inside: avoid;
}}

pre code {{
  background-color: transparent;
  padding: 0;
  border-radius: 0;
  color: #24292f;
  font-size: inherit;
}}

/* ==========================================================================
   Alerts & Callouts
   ========================================================================== */
.alert {{
  border-left: 4px solid;
  border-radius: 4px;
  padding: 12px 16px;
  margin: 16px 0;
  page-break-inside: avoid;
}}

.alert-title {{
  font-weight: 700;
  font-size: 9.5pt;
  margin-bottom: 6px;
  display: flex;
  align-items: center;
  gap: 6px;
}}

.alert-content > p:last-child {{
  margin-bottom: 0;
}}

.alert-note {{
  border-color: #0969da;
  background-color: #ddf4ff;
  color: #0969da;
}}
.alert-note .alert-content {{ color: #1f2328; }}

.alert-tip {{
  border-color: #1a7f37;
  background-color: #dafbe1;
  color: #1a7f37;
}}
.alert-tip .alert-content {{ color: #1f2328; }}

.alert-important {{
  border-color: #8250df;
  background-color: #fbefff;
  color: #8250df;
}}
.alert-important .alert-content {{ color: #1f2328; }}

.alert-warning {{
  border-color: #9a6700;
  background-color: #fff8c5;
  color: #9a6700;
}}
.alert-warning .alert-content {{ color: #1f2328; }}

.alert-caution {{
  border-color: #cf222e;
  background-color: #ffebe9;
  color: #cf222e;
}}
.alert-caution .alert-content {{ color: #1f2328; }}

blockquote {{
  margin: 12px 0;
  padding: 8px 16px;
  color: #57606a;
  border-left: 4px solid #d0d7de;
  background: #f6f8fa;
  border-radius: 0 4px 4px 0;
}}

blockquote > p:last-child {{
  margin-bottom: 0;
}}

/* ==========================================================================
   Mermaid Diagrams & Math
   ========================================================================== */
.mermaid {{
  display: flex;
  justify-content: center;
  margin: 20px 0;
  background: #f6f8fa;
  border: 1px solid #d0d7de;
  border-radius: 6px;
  padding: 15px;
  page-break-inside: avoid;
}}

.mermaid svg {{
  max-width: 100% !important;
  height: auto !important;
}}
</style>
</head>
<body>

<!-- 表紙 -->
<div class="cover-page">
  <div class="cover-header">
    <div class="cover-badge">Official Theoretical & Engineering Treatise</div>
    <h1 class="cover-title">Zenith CAD Kernel<br>総合技術書 ＆ アーキテクチャ解説書</h1>
    <p class="cover-subtitle">現代 3次元 CAD 幾何学・B-Rep 理論の体系的解説と<br>Rust 製独自カーネルの具現化・実測検証全履歴</p>
  </div>
  
  <div class="cover-body">
    <div class="cover-highlights">
      <div class="cover-highlight-card">
        <div class="cover-highlight-title">📐 CAD 幾何工学の数理体系</div>
        <div class="cover-highlight-desc">境界表現（B-Rep）、Euler-Poincaré不変量、有理NURBS基底、Gregoryパッチツイスト解消、SSIニュートン追跡の理論。</div>
      </div>
      <div class="cover-highlight-card">
        <div class="cover-highlight-title">🦀 完全自前 Rust B-Rep コア</div>
        <div class="cover-highlight-desc">OpenCASCADE等の外部巨大C++ライブラリを完全排除。単一軽量C拡張（zenith_cad.pyd 3.85MB）による脱OCCT達成。</div>
      </div>
      <div class="cover-highlight-card">
        <div class="cover-highlight-title">🔬 外部標準CADによる厳密実測検証</div>
        <div class="cover-highlight-desc">FreeCAD 1.1 / OpenCASCADE 7.x ヘッドレス連携による全STEP相互検証、体積・表面積・慣性・最短距離の解析解突き合わせ。</div>
      </div>
      <div class="cover-highlight-card">
        <div class="cover-highlight-title">📘 開発・検証・落とし穴の全記録</div>
        <div class="cover-highlight-desc">4-1から4-97に至る全改修ログ、接触配置での退化解消、テッセレーション非多様体病理の克服史を網羅。</div>
      </div>
    </div>
  </div>
  
  <div class="cover-footer">
    <div>
      <div class="cover-meta-item"><strong>発行プロジェクト:</strong> Zenith CAD Kernel Project (hinatahugu29/zenith-cad-kernel)</div>
      <div class="cover-meta-item"><strong>書誌構成:</strong> 4部・7章 ＋ 巻頭言 ＋ 巻末付録（CAD幾何用語集・検証プローブ便覧）</div>
      <div class="cover-meta-item"><strong>対象バージョン:</strong> Kernel v3.4.0 / Documentation v2026.08.26</div>
    </div>
    <div style="text-align: right;">
      <div class="cover-meta-item"><strong>生成日時:</strong> 2026年8月26日</div>
      <div class="cover-meta-item"><strong>ステータス:</strong> 公式技術書 (Official Treatise / Production Verified)</div>
    </div>
  </div>
</div>

<!-- 総合目次 -->
{toc_html}

<!-- 本文コンテンツ -->
{body_content}

<script>
window.mermaidReady = false;
window.mathJaxReady = false;

function checkAllReady() {{
  if (window.mermaidReady && (window.mathJaxReady || !window.MathJax)) {{
    console.log("ALL_READY");
    document.body.setAttribute("data-ready", "true");
  }}
}}

document.addEventListener("DOMContentLoaded", function() {{
  mermaid.initialize({{
    startOnLoad: true,
    theme: 'neutral',
    securityLevel: 'loose',
    fontFamily: 'Segoe UI, Meiryo, sans-serif'
  }});
  
  mermaid.run().then(() => {{
    window.mermaidReady = true;
    checkAllReady();
  }}).catch((e) => {{
    console.error("Mermaid error:", e);
    window.mermaidReady = true;
    checkAllReady();
  }});
}});

setTimeout(function() {{
  document.body.setAttribute("data-ready", "true");
}}, 6000);
</script>
</body>
</html>
"""
    with open(OUTPUT_HTML_PATH, 'w', encoding='utf-8') as f:
        f.write(full_html)
    print(f"Generated HTML: {OUTPUT_HTML_PATH} ({len(full_html):,} bytes)")
    return OUTPUT_HTML_PATH

def render_pdf_via_pyside(html_path, pdf_path):
    print("Launching PySide6 (Qt WebEngine / Chromium) PDF Renderer...")
    
    from PySide6.QtCore import QUrl, QTimer, QMarginsF
    from PySide6.QtWidgets import QApplication
    from PySide6.QtGui import QPageLayout, QPageSize
    from PySide6.QtWebEngineWidgets import QWebEngineView

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
        
    view = QWebEngineView()
    page = view.page()

    success_flag = False

    def on_pdf_finished(file_path, success):
        nonlocal success_flag
        success_flag = success
        if success:
            file_size = os.path.getsize(pdf_path)
            print(f"Successfully generated PDF: {pdf_path}")
            print(f"PDF File Size: {file_size:,} bytes ({file_size / (1024 * 1024):.2f} MB)")
        else:
            print(f"Failed to generate PDF: {file_path}")
        app.quit()

    page.pdfPrintingFinished.connect(on_pdf_finished)

    def on_loaded(ok):
        if not ok:
            print("Failed to load HTML into WebEnginePage")
            app.quit()
            return
            
        print("HTML loaded successfully into WebEngine. Waiting for Mermaid & MathJax rendering...")
        
        layout = QPageLayout(
            QPageSize(QPageSize.PageSizeId.A4),
            QPageLayout.Orientation.Portrait,
            QMarginsF(12, 16, 12, 16),
            QPageLayout.Unit.Millimeter
        )
        
        # 5秒待機してMermaidとMathJaxの非同期描画を確実に完了させる
        QTimer.singleShot(5000, lambda: page.printToPdf(pdf_path, layout))

    page.loadFinished.connect(on_loaded)
    page.load(QUrl.fromLocalFile(os.path.abspath(html_path)))
    app.exec()
    return success_flag

def main():
    print("=== Zenith CAD Kernel Professional Treatise PDF Pipeline ===")
    html_path = generate_html()
    success = render_pdf_via_pyside(html_path, OUTPUT_PDF_PATH)
    if success:
        print("=== Professional Treatise Pipeline Completed Successfully! ===")
    else:
        print("=== Pipeline Finished with Errors ===")

if __name__ == "__main__":
    main()
