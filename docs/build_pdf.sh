#!/usr/bin/env bash
# Rebuild docs/CONTROLS_GUIDE.pdf from docs/CONTROLS_GUIDE.md (needs pandoc + chromium/playwright).
set -euo pipefail
cd "$(dirname "$0")"
pandoc CONTROLS_GUIDE.md -s --no-highlight --toc --toc-depth=2 -c guide.css --embed-resources \
  --metadata lang=en -o /tmp/CONTROLS_GUIDE.html
node -e '
const { chromium } = require("playwright");
(async () => {
  const b = await chromium.launch(); const p = await b.newPage();
  await p.goto("file:///tmp/CONTROLS_GUIDE.html", { waitUntil: "load" });
  await p.pdf({ path: "CONTROLS_GUIDE.pdf", format: "A4", printBackground: true,
    margin: { top: "16mm", bottom: "16mm", left: "14mm", right: "14mm" },
    displayHeaderFooter: true, headerTemplate: "<span></span>",
    footerTemplate: "<div style=\"font-size:8px;width:100%;text-align:center;color:#888\">NIDAR Controls guide - page <span class=pageNumber></span> / <span class=totalPages></span></div>" });
  await b.close();
})();'
echo "wrote $(pwd)/CONTROLS_GUIDE.pdf"
