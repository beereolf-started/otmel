# Сводит карты otmel_map (рельеф и зоны, вблизи и издалека) в одно изображение с подписями. Р19.
import sys
from PIL import Image, ImageDraw, ImageFont

src, out = sys.argv[1], sys.argv[2]
panels = [("near_relief", "Рельеф и покров, 4096×4096 блоков"), ("near_zones", "Зоны: равнины / холмы / горы, 4096 блоков"),
          ("far_relief", "Рельеф и покров, 40960 блоков (пояса климата)"), ("far_zones", "Зоны, 40960 блоков")]
S, PAD, CAP = 760, 12, 34
img = Image.new("RGB", (2 * S + 3 * PAD, 2 * (S + CAP) + 3 * PAD), (24, 26, 30))
try:
    font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 20)
except OSError:
    font = ImageFont.load_default()
d = ImageDraw.Draw(img)
for k, (name, cap) in enumerate(panels):
    x, y = PAD + (k % 2) * (S + PAD), PAD + (k // 2) * (S + CAP + PAD)
    img.paste(Image.open(f"{src}/{name}.bmp").resize((S, S), Image.LANCZOS), (x, y + CAP))
    d.text((x + 4, y + 4), cap, fill=(225, 225, 220), font=font)
img.save(out)
