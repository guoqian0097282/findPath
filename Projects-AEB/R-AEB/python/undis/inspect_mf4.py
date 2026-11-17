# inspect_mf4.py
import sys, numpy as np
from asammdf import MDF

MF4 = sys.argv[1]
mdf = MDF(MF4)

def hx(b, n=16):
    return " ".join(f"{x:02X}" for x in b[:n])

# ---------- 1. 附件 ----------
print("=" * 70)
print("Attachments:", len(mdf.attachments))
for i, att in enumerate(mdf.attachments):
    print(f"\n--- attachment[{i}] ---")
    for attr in ("file_name", "mime", "comment", "compression", "flags"):
        print(f"  {attr:12s}: {getattr(att, attr, '<no attr>')}")
    try:
        raw = att.extract()
    except Exception as e:
        print("  extract() error:", e)
        raw = att.embedded_data
    print(f"  size        : {len(raw)} bytes")
    print(f"  head(16B)   : {hx(raw)}")
    print(f"  head(ASCII) : {raw[:16]!r}")

# ---------- 2. channels ----------
print("\n" + "=" * 70)
print("Groups:", len(mdf.groups))
for gi, g in enumerate(mdf.groups):
    print(f"\n=== group[{gi}]  ({len(g.channels)} channels) ===")
    for ci, ch in enumerate(g.channels):
        dtype = getattr(ch, "data_type", "?")
        ctype = getattr(ch, "channel_type", "?")
        unit  = getattr(ch, "unit", "")
        print(f"  [{ci:2d}] {ch.name!r:40s} dtype={dtype} ctype={ctype} unit={unit!r}")
        # 只对可能的视频 channel 深挖样本
        nm = (ch.name or "").lower()
        if any(k in nm for k in ("video","jpeg","jpg","image","frame","pic","cam","h264","mp4","yuv","rgb")):
            try:
                sig = mdf.get(group=gi, index=ci, raw=True)
            except Exception as e:
                print("       get() error:", e); continue
            arr = sig.samples
            print(f"       samples.shape={getattr(arr,'shape',None)} dtype={arr.dtype}")
            if arr.dtype == object and len(arr):
                s0 = arr[0]
                b0 = s0.tobytes() if isinstance(s0, np.ndarray) else bytes(s0)
                print(f"       samples[0] len={len(b0)}  head={hx(b0)}  ascii={b0[:16]!r}")
            elif arr.dtype == np.uint8 and arr.ndim >= 2:
                print(f"       samples[0] head={hx(arr[0].tobytes())}")