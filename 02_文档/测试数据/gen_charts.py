import csv, os
from PIL import Image, ImageDraw

D = r'C:\Users\lenovo\Desktop\FOC\02_文档\测试数据'
W, H = 900, 480
PAD = 70

def load(name, col):
    rows = list(csv.DictReader(open(os.path.join(D, name))))
    return [float(r["t_ms"]) for r in rows], [float(r[col]) for r in rows]

def chart(ts, ys, fname, title, ylabel, target=None, yunit=""):
    img = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img)
    iw, ih = W - 2*PAD, H - 2*PAD
    dr.rectangle([PAD, PAD, PAD+iw, PAD+ih], outline="black")
    ymin, ymax = min(ys), max(ys)
    if target is not None:
        ymin = min(ymin, target); ymax = max(ymax, target)
    span = ymax - ymin
    if span < 1e-9: span = 1
    ymin -= span*0.1; ymax += span*0.1; span = ymax - ymin
    tmax = max(ts) if max(ts) > 0 else 1

    def X(t): return PAD + (t/tmax)*iw
    def Y(v): return PAD + ih - ((v-ymin)/span)*ih

    # 网格
    for i in range(6):
        gy = PAD + ih*i/5
        dr.line([PAD, gy, PAD+iw, gy], fill=(230,230,230))
        lab = f"{ymax - span*i/5:.0f}"
        dr.text((8, gy-6), lab, fill="black")
    for i in range(6):
        gx = PAD + iw*i/5
        dr.line([gx, PAD, gx, PAD+ih], fill=(230,230,230))
        dr.text((gx-12, PAD+ih+8), f"{tmax*i/5:.0f}", fill="black")

    # 目标线
    if target is not None:
        dr.line([PAD, Y(target), PAD+iw, Y(target)], fill=(220,60,60), width=2)
        dr.text((PAD+iw-110, Y(target)-16), f"target {target:.0f}{yunit}", fill=(220,60,60))

    # 数据
    pts = [(X(t), Y(v)) for t, v in zip(ts, ys)]
    for i in range(len(pts)-1):
        dr.line([pts[i], pts[i+1]], fill=(30,90,200), width=2)

    dr.text((W//2 - len(title)*4, 18), title, fill="black")
    dr.text((W//2 - 60, H-28), "time [ms]", fill="black")
    dr.text((8, 30), ylabel, fill="black")
    img.save(os.path.join(D, fname))
    print(f"  ✓ {fname}  ({len(ts)} 点)")

ts, ys = load("step_current.csv", "iq_ma")
chart(ts, ys, "fig_current.png", "Current loop step (T0.2)", "Iq [mA]", 200, "mA")

ts, ys = load("step_velocity.csv", "pos_mrad")
spd = [(ys[i]-ys[i-1])/10.0 for i in range(1,len(ys))]
chart(ts[1:], spd, "fig_velocity.png", "Velocity loop step (V10)", "speed [rad/s]", 10, " rad/s")

ts, ys = load("step_position.csv", "pos_mrad")
chart(ts, ys, "fig_position.png", "Position loop step (+1 turn)", "pos [mrad]", 42278, " mrad")

print("\n三张图已生成")
