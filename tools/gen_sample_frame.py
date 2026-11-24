"""Extract the TOFSense-M sample frame from the Nooploop protocol page.

Writes tools/tof_sample_frame.h with the 400 byte reference frame (8x8 mode)
documented at https://support.nooploop.com/tofsense/protocol/
"""

import io
import os
import re
import sys
import urllib.request

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

URL = "https://support.nooploop.com/tofsense/protocol/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tof_sample_frame.h")

req = urllib.request.Request(URL, headers={"User-Agent": "Mozilla/5.0"})
html = urllib.request.urlopen(req, timeout=60).read().decode("utf-8", "replace")
text = re.sub(r"<[^>]+>", " ", html)
text = (text.replace("&quot;", '"').replace("&#x27;", "'").replace("&amp;", "&")
        .replace("&lt;", "<").replace("&gt;", ">"))
text = re.sub(r"\s+", " ", text)

frames = []
for match in re.finditer(r"(57 01 ff 00 03 a0 00 00 40)", text, flags=re.IGNORECASE):
    tail = text[match.start():match.start() + 400 * 3 + 80]
    tokens = re.findall(r"\b[0-9a-fA-F]{2}\b", tail)
    if len(tokens) < 400:
        continue
    nums = [int(t, 16) for t in tokens[:400]]
    if sum(nums[:-1]) & 0xFF != nums[-1]:
        print("checksum mismatch for candidate (len %d), trying next" % len(nums))
        continue
    frames.append(nums)

if not frames:
    # fall back: some pages show the frame as 0xNN,NN
    for run in re.findall(r"(?:0x[0-9a-fA-F]{2}\s*,\s*){390,}0x[0-9a-fA-F]{2}", text):
        nums = [int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", run)]
        if len(nums) >= 390 and nums[0] == 0x57 and nums[1] == 0x01:
            frames.append(nums[:400] if len(nums) >= 400 else nums)

if not frames:
    raise SystemExit("sample frame not found")

best = frames[0]
print("bytes:", len(best), "checksum ok:", (sum(best[:-1]) & 0xFF) == best[-1])

lines = []
for i in range(0, len(best), 12):
    lines.append("\t" + ", ".join("0x%02X" % b for b in best[i:i + 12]) + ",")

with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
    fh.write("// generated from %s\n" % URL)
    fh.write("#pragma once\n\n")
    fh.write("static const unsigned char kSampleFrame[] = {\n")
    fh.write("\n".join(lines) + "\n")
    fh.write("};\n\nstatic const unsigned kSampleFrameSize = sizeof(kSampleFrame);\n")

print("wrote", OUT)
