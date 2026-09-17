"""Regenerate the page embedded in ESP32 firmware after editing index.html."""
from pathlib import Path
root = Path(__file__).resolve().parents[1] / 'main'
data = (root / 'index.html').read_bytes()
lines = ['unsigned char s_index_html[] = {']
for offset in range(0, len(data), 12):
    lines.append('  ' + ', '.join(f'0x{byte:02x}' for byte in data[offset:offset+12]) + ',')
lines += ['};', f'unsigned int s_index_html_len = {len(data)};', '']
(root / 'index_html.h').write_text('\n'.join(lines))
