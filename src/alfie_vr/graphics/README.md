# VR controls graphics

Reference graphics for the Quest controller mapping in `alfie_teleop_vr.py` and `web-ui/controller_tracking.js`. These files are the source of truth; update them in the same commit as any mapping change.

| File | What it is |
|------|------------|
| `controls_sheet.html` | 16:9 sheet with labels on the controllers. Source of the in-headset key. |
| `controls_sheet_print.html` | Light, ink-friendly copy of the sheet for printing. |
| `controls_reference.html` | Numbered left/right page with fuller descriptions. |
| `render_controls_key.py` | Renders `controls_sheet.html` in dark mode to `../alfie_vr/web-ui/controls_key.png` (1920x1080). |

The headset shows `controls_key.png` while left X is held (`web-ui/controls_key_panel.js`).

## After changing the mapping

1. Edit the labels in all three HTML files. The sheets use fixed SVG coordinates, so keep labels short and check that nothing collides.
2. Re-render the headset key, then rebuild the package:
   ```bash
   python3 src/alfie_vr/graphics/render_controls_key.py
   colcon build --packages-select alfie_vr
   ```
   Rendering needs Firefox and network access for Google Fonts. Open the PNG and check it before committing.
3. Reload the page in the headset.

The pages are also published as private claude.ai artifacts: [sheet](https://claude.ai/artifact/BrkPRJeLj9Xt5h38MVMdYT), [print](https://claude.ai/artifact/JVCUP8tjS58nUVBqoyzDCq), [reference](https://claude.ai/artifact/GyKBBKm33r3A8GmMcB8Z4f). They are copies; republish them from these files.

Each HTML file is a page fragment (no `<html>`/`<head>` wrapper) as the artifact host expects. It still opens directly in a browser.
