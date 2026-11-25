# -*- coding: utf-8 -*-
"""
FOC 三环阶跃响应分析 + 绘图
读取 step_current / step_velocity / step_position 三个 CSV，
计算上升时间、超调量、调节时间、稳态误差，并用 Pillow 画曲线图。
"""
import csv
import os
import math
from PIL import Image, ImageDraw, ImageFont

DATA = r"C:\Users\lenovo\Desktop\FOC\02_文档\测试数据"
OUT = DATA
DT = 0.020  # 遥测周期 20 ms

# ---------- 字体 ----------
def get_font(size):
    for p in (r"C:\Windows\Fonts\msyh.ttc",
              r"C:\Windows\Fonts\msyhl.ttc",
              r"C:\Windows\Fonts\simhei.ttf",
              r"C:\Windows\Fonts\simsun.ttc"):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()

F_TITLE = get_font(22)
F_LABEL = get_font(15)
F_SMALL = get_font(13)

# ---------- 读 CSV ----------
def load(name):
    path = os.path.join(DATA, name)
    rows = []
    with open(path, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows

# ---------- 指标 ----------
def metrics(t, y, target, tol=0.05):
    """返回 (上升时间10-90%, 超调%, 调节时间, 稳态误差, 峰值)"""
    n = len(y)
    y0 = y[0]
    span = target - y0
    if abs(span) < 1e-9:
        return None
    # 归一化到 0..1
    yn = [(v - y0) / span for v in y]

    # 上升时间 10% -> 90%
    t10 = t90 = None
    for i, v in enumerate(yn):
        if t10 is None and v >= 0.10:
            t10 = t[i]
        if t90 is None and v >= 0.90:
            t90 = t[i]
            break
    tr = (t90 - t10) if (t10 is not None and t90 is not None) else float("nan")

    peak = max(yn)
    overshoot = (peak - 1.0) * 100.0

    # 调节时间：最后一次离开 ±tol 带
    band = tol
    ts = t[-1]
    for i in range(n - 1, -1, -1):
        if abs(yn[i] - 1.0) > band:
            ts = t[i + 1] if i + 1 < n else t[i]
            break

    ss_err = y[-1] - target
    return {
        "rise": tr,
        "overshoot": overshoot,
        "settle": ts,
        "ss_err": ss_err,
        "ss_err_pct": (ss_err / target * 100.0) if abs(target) > 1e-9 else 0.0,
        "peak_val": y[0] + peak * span,
    }

# ---------- 画图 ----------
def plot(points_list, target, title, ylabel, fname, extra_lines=None):
    """points_list = [(label, [(t,y), ...], color), ...]"""
    W, H = 1100, 620
    ML, MR, MT, MB = 95, 40, 70, 75
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    all_t = [p[0] for _, pts, _ in points_list for p in pts]
    all_y = [p[1] for _, pts, _ in points_list for p in pts]
    all_y.append(target)
    tmax = max(all_t) if all_t else 1.0
    ymin = min(all_y)
    ymax = max(all_y)
    pad = (ymax - ymin) * 0.12 or 1.0
    ymin -= pad
    ymax += pad

    def X(t):
        return ML + (t / tmax) * (W - ML - MR)

    def Y(v):
        return H - MB - (v - ymin) / (ymax - ymin) * (H - MT - MB)

    # 网格
    for i in range(6):
        v = ymin + (ymax - ymin) * i / 5.0
        y = Y(v)
        d.line([(ML, y), (W - MR, y)], fill=(230, 230, 230), width=1)
        d.text((ML - 80, y - 8), f"{v:.2f}", font=F_SMALL, fill=(90, 90, 90))
    for i in range(6):
        tt = tmax * i / 5.0
        x = X(tt)
        d.line([(x, MT), (x, H - MB)], fill=(240, 240, 240), width=1)
        d.text((x - 18, H - MB + 8), f"{tt:.1f}", font=F_SMALL, fill=(90, 90, 90))

    # 坐标轴
    d.line([(ML, MT), (ML, H - MB)], fill=(0, 0, 0), width=2)
    d.line([(ML, H - MB), (W - MR, H - MB)], fill=(0, 0, 0), width=2)

    # 目标线
    ty = Y(target)
    for x in range(ML, W - MR, 12):
        d.line([(x, ty), (x + 6, ty)], fill=(220, 60, 60), width=2)

    # 曲线
    for label, pts, color in points_list:
        if len(pts) < 2:
            continue
        xy = [(X(t), Y(v)) for t, v in pts]
        d.line(xy, fill=color, width=2)

    # 标题/轴标签
    d.text((ML, 20), title, font=F_TITLE, fill=(0, 0, 0))
    d.text((W // 2 - 40, H - 45), "时间 (s)", font=F_LABEL, fill=(0, 0, 0))
    d.text((10, MT + 60), ylabel, font=F_LABEL, fill=(0, 0, 0))

    # 图例
    lx, ly = W - MR - 250, MT + 8
    for i, (label, pts, color) in enumerate(points_list):
        yy = ly + i * 24
        d.line([(lx, yy + 8), (lx + 28, yy + 8)], fill=color, width=3)
        d.text((lx + 36, yy), label, font=F_SMALL, fill=(0, 0, 0))
    d.line([(lx, ly + len(points_list) * 24 + 8), (lx + 28, ly + len(points_list) * 24 + 8)],
           fill=(220, 60, 60), width=2)
    d.text((lx + 36, ly + len(points_list) * 24), "目标值", font=F_SMALL, fill=(0, 0, 0))

    # 指标文字
    if extra_lines:
        y0 = H - MB - 20 * len(extra_lines) - 10
        for i, s in enumerate(extra_lines):
            d.text((W - MR - 330, ly + 90 + i * 20), s, font=F_SMALL, fill=(30, 30, 130))

    img.save(os.path.join(OUT, fname))
    return fname

# ================= 主流程 =================
report = []
report.append("=" * 64)
report.append("            FOC 三环阶跃响应 实测报告")
report.append("=" * 64)

# ---- 1. 电流环 ----
cur = load("step_current.csv")
t = [i * DT for i in range(len(cur))]
y = [r["iq"] for r in cur]
target = 300.0
m = metrics(t, y, target)
report.append("")
report.append("【1】电流环阶跃  0 → 300 mA")
report.append(f"    采样点数        : {len(cur)}   采样周期 {DT*1000:.0f} ms")
report.append(f"    上升时间(10-90%): {m['rise']*1000:.0f} ms")
report.append(f"    超调量          : {m['overshoot']:.1f} %")
report.append(f"    调节时间(±5%)   : {m['settle']*1000:.0f} ms")
report.append(f"    稳态误差        : {m['ss_err']:.0f} mA  ({m['ss_err_pct']:.1f} %)")
report.append(f"    稳态值          : {y[-1]:.0f} mA")
plot([("Iq 实测", list(zip(t, y)), (0, 100, 200))], target,
     "电流环阶跃响应  0 → 300 mA", "Iq (mA)", "fig_current.png",
     [f"上升时间 {m['rise']*1000:.0f} ms",
      f"超调 {m['overshoot']:.1f} %",
      f"调节时间 {m['settle']*1000:.0f} ms",
      f"稳态误差 {m['ss_err']:.0f} mA"])

# ---- 2. 速度环 ----
vel = load("step_velocity.csv")
t2 = [i * DT for i in range(len(vel))]
pos = [r["pos"] for r in vel]
# 由位置差分求速度 (mrad/s -> rad/s)
w = [0.0]
for i in range(1, len(pos)):
    w.append((pos[i] - pos[i - 1]) / 1000.0 / DT)
# 简单滑动平均，抑制差分噪声
ws = []
for i in range(len(w)):
    a = max(0, i - 2)
    seg = w[a:i + 1]
    ws.append(sum(seg) / len(seg))
tw = t2[:len(ws)]
target2 = 10.0
m2 = metrics(tw, ws, target2)
report.append("")
report.append("【2】速度环阶跃  0 → 10 rad/s")
report.append(f"    采样点数        : {len(vel)}")
report.append(f"    上升时间(10-90%): {m2['rise']*1000:.0f} ms")
report.append(f"    超调量          : {m2['overshoot']:.1f} %")
report.append(f"    调节时间(±5%)   : {m2['settle']*1000:.0f} ms")
report.append(f"    稳态误差        : {m2['ss_err']:.2f} rad/s  ({m2['ss_err_pct']:.1f} %)")
report.append(f"    稳态速度        : {ws[-1]:.2f} rad/s  ({ws[-1]*60/(2*math.pi):.1f} rpm)")
plot([("速度实测", list(zip(tw, ws)), (0, 140, 90))], target2,
     "速度环阶跃响应  0 → 10 rad/s", "速度 (rad/s)", "fig_velocity.png",
     [f"上升时间 {m2['rise']*1000:.0f} ms",
      f"超调 {m2['overshoot']:.1f} %",
      f"调节时间 {m2['settle']*1000:.0f} ms",
      f"稳态误差 {m2['ss_err']:.2f} rad/s"])

# ---- 3. 位置环 ----
posf = load("step_position.csv")
t3 = [i * DT for i in range(len(posf))]
y3 = [r["pos"] / 1000.0 for r in posf]   # mrad -> rad
target3 = 1.0
m3 = metrics(t3, y3, target3)
report.append("")
report.append("【3】位置环阶跃  0 → 1.000 rad")
report.append(f"    采样点数        : {len(posf)}")
report.append(f"    上升时间(10-90%): {m3['rise']*1000:.0f} ms")
report.append(f"    超调量          : {m3['overshoot']:.1f} %")
report.append(f"    调节时间(±5%)   : {m3['settle']*1000:.0f} ms")
report.append(f"    稳态误差        : {m3['ss_err']*1000:.1f} mrad  ({m3['ss_err_pct']:.2f} %)")
report.append(f"    稳态位置        : {y3[-1]:.3f} rad  ({y3[-1]*180/math.pi:.2f} deg)")
plot([("位置实测", list(zip(t3, y3)), (150, 60, 180))], target3,
     "位置环阶跃响应  0 → 1.000 rad", "位置 (rad)", "fig_position.png",
     [f"上升时间 {m3['rise']*1000:.0f} ms",
      f"超调 {m3['overshoot']:.1f} %",
      f"调节时间 {m3['settle']*1000:.0f} ms",
      f"稳态误差 {m3['ss_err']*1000:.0f} mrad"])

# ---- 汇总 ----
report.append("")
report.append("=" * 64)
report.append("                        汇 总")
report.append("=" * 64)
report.append(f"  环     上升时间    超调量     调节时间   稳态误差")
report.append(f"  电流   {m['rise']*1000:6.0f} ms  {m['overshoot']:7.1f}%  {m['settle']*1000:7.0f} ms  {m['ss_err']:7.0f} mA")
report.append(f"  速度   {m2['rise']*1000:6.0f} ms  {m2['overshoot']:7.1f}%  {m2['settle']*1000:7.0f} ms  {m2['ss_err']:7.2f} rad/s")
report.append(f"  位置   {m3['rise']*1000:6.0f} ms  {m3['overshoot']:7.1f}%  {m3['settle']*1000:7.0f} ms  {m3['ss_err']*1000:7.0f} mrad")
report.append("=" * 64)

txt = "\n".join(report)
print(txt)
with open(os.path.join(OUT, "阶跃响应实测报告.txt"), "w", encoding="utf-8") as f:
    f.write(txt + "\n")
print("\n已生成:")
for f in ("fig_current.png", "fig_velocity.png", "fig_position.png", "阶跃响应实测报告.txt"):
    p = os.path.join(OUT, f)
    print("  ", f, os.path.getsize(p) if os.path.exists(p) else "(missing)")
