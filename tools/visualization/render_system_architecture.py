"""Render the documented system architecture locally without web services."""

from pathlib import Path
import math

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs" / "system_architecture.png"


def font(size, bold=False):
    for name in (("arialbd.ttf" if bold else "arial.ttf"), ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


TITLE, HEADER, BODY, SMALL = font(32, True), font(23, True), font(18), font(15)


def box(draw, xy, label, fill="#ffffff", outline="#31506f", text_font=BODY):
    draw.rounded_rectangle(xy, radius=14, fill=fill, outline=outline, width=3)
    left, top, right, bottom = xy
    bounds = draw.multiline_textbbox((0, 0), label, font=text_font, align="center", spacing=4)
    width, height = bounds[2] - bounds[0], bounds[3] - bounds[1]
    draw.multiline_text(((left + right - width) / 2, (top + bottom - height) / 2), label,
                        font=text_font, fill="#132238", align="center", spacing=4)


def arrow(draw, start, end, label=None, dashed=False):
    color, width = "#31506f", 4
    x1, y1 = start
    x2, y2 = end
    if dashed:
        for i in range(0, 12, 2):
            a, b = i / 12, min((i + 1) / 12, 1)
            draw.line((x1+(x2-x1)*a, y1+(y2-y1)*a, x1+(x2-x1)*b, y1+(y2-y1)*b), fill=color, width=width)
    else:
        draw.line((*start, *end), fill=color, width=width)
    angle = math.atan2(y2-y1, x2-x1)
    for offset in (2.6, -2.6):
        draw.line((x2, y2, x2+15*math.cos(angle+offset), y2+15*math.sin(angle+offset)), fill=color, width=width)
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        bounds = draw.textbbox((0, 0), label, font=SMALL)
        text_width = bounds[2] - bounds[0]
        draw.rectangle((mx-text_width/2-5, my-13, mx+text_width/2+5, my+11), fill="#f7f9fc")
        draw.text((mx-text_width/2, my-10), label, font=SMALL, fill="#243b55")


def main():
    image = Image.new("RGB", (1800, 1050), "#f7f9fc")
    draw = ImageDraw.Draw(image)
    draw.text((900, 35), "Multi-QCar System Architecture", font=TITLE, fill="#132238", anchor="ma")
    panels = [
        ((45, 105, 555, 920), "Local Windows Computer — 192.168.0.153", "#e8f1fb"),
        ((645, 105, 1190, 920), "QCar .151 — preventive stop", "#eef8ee"),
        ((1280, 105, 1755, 920), "QCar .152 — baseline safety", "#fff5e8"),
    ]
    for area, label, color in panels:
        draw.rounded_rectangle(area, radius=20, fill=color, outline="#60758a", width=3)
        draw.text(((area[0]+area[2])/2, area[1]+22), label, font=HEADER, fill="#132238", anchor="ma")

    box(draw, (115, 190, 485, 285), "Launcher\nrun_all_cars.bat / start.py")
    box(draw, (115, 390, 485, 485), "Intersection Coordinator\nUDP 18750", fill="#d7e8fa")
    box(draw, (115, 645, 485, 740), "Observer and Logs")
    box(draw, (795, 185, 1040, 265), "Vehicle Control", fill="#dcefdc")
    box(draw, (690, 350, 895, 430), "GPS/LiDAR + EKF")
    box(draw, (945, 350, 1145, 430), "Route + Path\nTracking")
    box(draw, (680, 520, 885, 600), "YOLO Obstacle\nSafety")
    box(draw, (945, 500, 1155, 620), "Front CSI\nMeasurement +\nShadow Proposal", fill="#fff2ba")
    box(draw, (700, 695, 910, 800), "Preventive Stop\nSpeed only; latched;\nno auto recovery", fill="#ffd9d9", outline="#a83c3c", text_font=SMALL)
    box(draw, (970, 710, 1145, 785), "Steering +\nThrottle")
    box(draw, (1400, 185, 1635, 265), "Vehicle Control", fill="#ffe6c7")
    box(draw, (1315, 350, 1505, 430), "GPS/LiDAR + EKF")
    box(draw, (1530, 350, 1720, 430), "Route + Path\nTracking")
    box(draw, (1320, 540, 1510, 620), "YOLO Obstacle\nSafety")
    box(draw, (1535, 545, 1715, 615), "Steering +\nThrottle")

    arrow(draw, (485, 235), (795, 225), "SSH/SCP")
    arrow(draw, (485, 255), (1400, 225), "SSH/SCP")
    arrow(draw, (795, 245), (485, 420), "state / grant")
    arrow(draw, (1400, 245), (485, 455), "state / grant")
    arrow(draw, (300, 285), (300, 390))
    arrow(draw, (300, 285), (300, 645), dashed=True)
    arrow(draw, (790, 350), (850, 265))
    arrow(draw, (1045, 350), (990, 265))
    arrow(draw, (785, 520), (835, 265))
    arrow(draw, (1050, 500), (1000, 265), "measurement", True)
    arrow(draw, (1050, 620), (805, 695), "boundary state", True)
    arrow(draw, (910, 748), (970, 748), "speed limit", True)
    arrow(draw, (1040, 225), (1055, 710))
    arrow(draw, (945, 560), (485, 690), "diagnostics", True)
    arrow(draw, (795, 255), (485, 670), "logs", True)
    arrow(draw, (1410, 350), (1460, 265))
    arrow(draw, (1625, 350), (1580, 265))
    arrow(draw, (1415, 540), (1460, 265))
    arrow(draw, (1630, 545), (1555, 265))
    arrow(draw, (1400, 250), (485, 720), "logs", True)
    draw.text((900, 970), "Solid: active control/data path   •   Dashed: diagnostics or restrictive safety path", font=BODY, fill="#40566e", anchor="ma")
    image.save(OUTPUT)
    print(f"Wrote {OUTPUT}")


if __name__ == "__main__":
    main()
