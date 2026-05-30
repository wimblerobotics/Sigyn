#!/usr/bin/env bash
# generate_pdf.sh
#
# Convert topic_analysis.md → topic_analysis.pdf using
#   pandoc  (Markdown → standalone HTML, preserving the embedded <style> block)
#   wkhtmltopdf  (HTML → PDF, WebKit correctly repeats <thead> on every page)
#
# Usage:
#   ./generate_pdf.sh                          # uses defaults
#   ./generate_pdf.sh /path/to/my_report.md    # override input
#   ./generate_pdf.sh "" /tmp/out.pdf          # override output
#
# Dependencies (both already installed on Sigyn dev machine):
#   pandoc 3.x    https://pandoc.org
#   wkhtmltopdf   https://wkhtmltopdf.org

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARTIFACTS_DIR="$SCRIPT_DIR/../../docs/artifacts"

INPUT="${1:-$ARTIFACTS_DIR/topic_analysis.md}"
OUTPUT="${2:-$ARTIFACTS_DIR/topic_analysis.pdf}"
TMP_HTML="$(mktemp /tmp/topic_analysis_XXXXXX.html)"
TMP_CSS="$(mktemp /tmp/topic_analysis_XXXXXX.css)"

trap 'rm -f "$TMP_HTML" "$TMP_CSS"' EXIT

# ── Write CSS into a real <head> stylesheet ───────────────────────────────────
cat > "$TMP_CSS" << 'EOF'
<style>
@page {
  size: A4 landscape;
  margin: 10mm 3mm 10mm 3mm;
}
body {
  font-size: 9pt;
  max-width: none !important;
  padding: 0 !important;
  margin: 0 !important;
}
table {
  font-size: 7pt;
  border-collapse: collapse;
  width: 100%;
  table-layout: fixed;
}
th {
  white-space: nowrap;
  padding: 2px 4px;
  vertical-align: bottom;
  overflow: hidden;
  background-color: #d8d8d8;
  border-bottom: 1px solid #999;
  font-weight: bold;
}
td {
  padding: 2px 4px;
  vertical-align: top;
  overflow-wrap: break-word;
  word-wrap: break-word;
  word-break: break-all;
}
/* 8-column summary table — fixed widths summing to 100% */
table th:nth-child(1), table td:nth-child(1) { width: 22%; }
table th:nth-child(2), table td:nth-child(2) { width:  8%; }
table th:nth-child(3), table td:nth-child(3) { width:  7%; }
table th:nth-child(4), table td:nth-child(4) { width:  6%; }
table th:nth-child(5), table td:nth-child(5) { width: 15%; }
table th:nth-child(6), table td:nth-child(6) { width: 19%; }
table th:nth-child(7), table td:nth-child(7) { width: 17%; }
table th:nth-child(8), table td:nth-child(8) { width:  6%; }
code { font-size: 6.5pt; }
</style>
EOF

echo "Converting Markdown → HTML …"
pandoc "$INPUT" \
    --from gfm+raw_html \
    --to html5 \
    --standalone \
    --include-in-header "$TMP_CSS" \
    --metadata title="Sigyn Topic Analysis" \
    -o "$TMP_HTML"

# ── Physically insert repeated header rows every N data rows ──────────────────
# wkhtmltopdf 0.12.6 ignores thead { display: table-header-group } regardless
# of where the CSS lives — the only reliable workaround is to duplicate the
# header row in the HTML itself.
echo "Inserting repeated table headers …"
python3 - "$TMP_HTML" << 'PYEOF'
import re, sys

REPEAT_EVERY = 28   # data rows between repeated header rows

html = open(sys.argv[1]).read()

def process_table(m):
    table_html = m.group(0)
    thead_m = re.search(r'<thead>(.*?)</thead>', table_html, re.DOTALL)
    tbody_m = re.search(r'<tbody>(.*?)</tbody>', table_html, re.DOTALL)
    if not thead_m or not tbody_m:
        return table_html

    header_row_m = re.search(r'<tr[^>]*>.*?</tr>', thead_m.group(1), re.DOTALL)
    if not header_row_m:
        return table_html

    # Clone the header row with a distinct repeat style
    repeat_row = re.sub(
        r'<tr([^>]*)>',
        r'<tr\1 style="background:#d8d8d8;font-weight:bold;font-size:6.5pt;border-top:1px solid #999;">',
        header_row_m.group(0),
        count=1
    )

    rows = re.findall(r'<tr[^>]*>.*?</tr>', tbody_m.group(1), re.DOTALL)
    result = []
    for i, row in enumerate(rows):
        if i > 0 and i % REPEAT_EVERY == 0:
            result.append(repeat_row)
        result.append(row)

    new_tbody = '<tbody>' + ''.join(result) + '</tbody>'
    return table_html.replace(tbody_m.group(0), new_tbody)

html = re.sub(r'<table>.*?</table>', process_table, html, flags=re.DOTALL)
open(sys.argv[1], 'w').write(html)
PYEOF

echo "Converting HTML → PDF …"
wkhtmltopdf \
    --page-size A4 \
    --orientation Landscape \
    --margin-top    10mm \
    --margin-bottom 10mm \
    --margin-left    3mm \
    --margin-right   3mm \
    --enable-local-file-access \
    --quiet \
    "$TMP_HTML" "$OUTPUT"

echo "Written: $OUTPUT"
