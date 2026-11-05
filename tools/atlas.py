# Сборка атласа текстур блоков (Р80).
#
# Образцы берутся из сборки TerraFirmaGreg: двадцать наших пород совпадают с породами TFC, и их
# рисунки берутся как есть. Для шестнадцати промежуточных пород (Р78) своего рисунка в TFC нет -
# берём рисунок ближайшей родственной породы как МАСКУ (её светлота) и красим нашей палитрой.
#
# На выходе два файла рядом с игрой: assets/atlas.png (посмотреть глазами) и assets/atlas.bin
# (сырые RGBA, по ним игра и грузит - чтобы не тащить в движок разбор PNG).
#
# В atlas.bin клетки лежат не сеткой, а СЛОЯМИ: каждый материал - свой слой набора картинок. Сетка
# не годится, потому что вдали нужны уменьшенные копии (без них рисунок на земле рассыпается в
# полосы), а уменьшение сетки смешивает соседние клетки между собой. Порядок: заголовок
# <сторона, слоёв, уровней>, затем уровень за уровнем, внутри уровня слой за слоем, RGBA.
#
#   python tools/atlas.py
#
# Текстуры не наши: см. assets/ТЕКСТУРЫ.md.
import io
import os
import struct
import sys
import zipfile
import zlib

GAME = r"C:\Users\Lenovo LEGION\AppData\Roaming\.tlauncher\legacy\Minecraft\game"
TFC = os.path.join(GAME, "mods", "TerraFirmaCraft-Forge-1.20.1-3.2.21.jar")
VANILLA = os.path.join(GAME, "versions", "Forge 1.20.1", "Forge 1.20.1.jar")

TILE = 16          # сторона рисунка, точек
LAYERS = 173       # материалы 0..172 (world/chunk.h)
COLS, ROWS = 8, 22  # раскладка только для картинки-образца atlas.png

# ── чтение PNG без сторонних библиотек: нам нужны только 16×16 RGBA и палитровые ────────────────


def png_read(data):
    """Возвращает (ширина, высота, список RGBA-точек)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "не PNG"
    pos, w, h, bits, ctype, idat, plte, trns = 8, 0, 0, 8, 6, b"", None, None
    while pos < len(data):
        ln = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + ln]
        if tag == b"IHDR":
            w, h, bits, ctype = struct.unpack(">IIBB", body[:10])
        elif tag == b"PLTE":
            plte = body
        elif tag == b"tRNS":
            trns = body
        elif tag == b"IDAT":
            idat += body
        elif tag == b"IEND":
            break
        pos += 12 + ln
    assert bits in (1, 2, 4, 8), "ждём не больше 8 бит на канал"
    chan = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[ctype]
    raw = zlib.decompress(idat)
    # палитра с малой разрядностью: распаковываем в байт на точку, дальше всё как при 8 битах
    if bits < 8:
        per = 8 // bits
        stride = (w + per - 1) // per
        wide = bytearray()
        for y in range(h):
            off = y * (stride + 1)
            wide.append(raw[off])   # способ восстановления строки
            for x in range(w):
                b8 = raw[off + 1 + x // per]
                sh = 8 - bits * (x % per + 1)
                wide.append((b8 >> sh) & ((1 << bits) - 1))
        raw = bytes(wide)
    out, prev, p = [], bytearray(w * chan), 0
    for _ in range(h):
        f = raw[p]
        p += 1
        line = bytearray(raw[p:p + w * chan])
        p += w * chan
        for i in range(len(line)):
            a = line[i - chan] if i >= chan else 0
            b = prev[i]
            c = prev[i - chan] if i >= chan else 0
            if f == 1: line[i] = (line[i] + a) & 255
            elif f == 2: line[i] = (line[i] + b) & 255
            elif f == 3: line[i] = (line[i] + (a + b) // 2) & 255
            elif f == 4:
                pp = a + b - c
                pa, pb, pc = abs(pp - a), abs(pp - b), abs(pp - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 255
        prev = line
        for x in range(w):
            if ctype == 6: out.append(tuple(line[x * 4:x * 4 + 4]))
            elif ctype == 2: out.append((line[x * 3], line[x * 3 + 1], line[x * 3 + 2], 255))
            elif ctype == 0: out.append((line[x], line[x], line[x], 255))
            elif ctype == 4: out.append((line[x * 2], line[x * 2], line[x * 2], line[x * 2 + 1]))
            else:
                i = line[x]
                a = trns[i] if trns and i < len(trns) else 255
                out.append((plte[i * 3], plte[i * 3 + 1], plte[i * 3 + 2], a))
    return w, h, out


def png_write(path, w, h, px):
    rows = b"".join(b"\x00" + bytes(v for p in px[y * w:(y + 1) * w] for v in p) for y in range(h))
    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(rows, 9)))
        f.write(chunk(b"IEND", b""))


# ── что куда кладём ────────────────────────────────────────────────────────────────────────────
# породы Р77 по порядку world::Rock - ровно двадцать пород TFC
TFC_ROCKS = ["granite", "diorite", "gabbro", "rhyolite", "basalt", "andesite", "dacite", "shale", "claystone",
             "limestone", "conglomerate", "dolomite", "chert", "chalk", "quartzite", "slate", "phyllite", "schist",
             "gneiss", "marble"]
# промежуточные Р78: чей рисунок берём маской и каким цветом красим (цвета - из shaders/terrain.vert)
MIXED = [("granite", (0.67, 0.63, 0.60)), ("diorite", (0.70, 0.66, 0.60)), ("gabbro", (0.18, 0.22, 0.18)),
         ("rhyolite", (0.74, 0.70, 0.64)), ("andesite", (0.66, 0.62, 0.56)), ("basalt", (0.33, 0.33, 0.34)),
         ("basalt", (0.10, 0.10, 0.12)), ("conglomerate", (0.78, 0.68, 0.48)), ("shale", (0.58, 0.54, 0.46)),
         ("limestone", (0.74, 0.72, 0.62)), ("conglomerate", (0.50, 0.48, 0.42)), ("chert", (0.42, 0.38, 0.36)),
         ("schist", (0.26, 0.30, 0.26)), ("gneiss", (0.64, 0.58, 0.54)), ("phyllite", (0.40, 0.50, 0.40)),
         ("marble", (0.28, 0.40, 0.32))]
# материалы 0..11 (world/chunk.h): воздух, камень, земля, трава, песок, снег, вода, кора, древесина,
# листва, листва светлее, кора в трещине
GRASS_RGB = (0.36, 0.58, 0.25)   # тот же цвет, что в shaders/terrain.vert
GRASS_BY_SOIL = [(0.5, 0.6, 0.3), (0.44, 0.6, 0.27), (0.36, 0.58, 0.25), (0.4, 0.6, 0.28), (0.47, 0.62, 0.33), (0.31, 0.53, 0.23), (0.28, 0.49, 0.22), (0.3, 0.47, 0.24), (0.25, 0.4, 0.18)]
# Р87: почвы по номерам SOIL_FIRST.. - порядок ровно как в src/world/soil_table.cpp
# Р88: руды по номерам ORE_FIRST.. - порядок ровно как в src/world/ores.cpp
ORES = ["normal_malachite", "normal_native_copper", "pyrite", "normal_tetrahedrite", "normal_limonite",
        "normal_hematite", "normal_magnetite", "normal_cassiterite", "normal_sphalerite",
        "normal_bismuthinite", "normal_native_gold", "normal_native_silver", "rich_bismuthinite",
        "normal_garnierite", "bituminous_coal", "lignite", "gypsum", "halite", "sulfur", "graphite"]
ORES = ["tfc:block/ore/" + o for o in ORES]
SOILS = ["tfc:block/sand/yellow", "tfc:block/dirt/sandy_loam", "tfc:block/dirt/loam",
         "tfc:block/dirt/silty_loam", "tfc:block/dirt/silt", "tfc:block/clay/loam",
         "tfc:block/clay/silty_loam", "tfc:block/mud/silt", "tfc:block/peat"]
BASE = ["", "tfc:block/rock/raw/granite", "tfc:block/dirt/loam", "tfc:block/grass_top", "tfc:block/sand/yellow",
        "mc:block/snow", "", "tfc:block/wood/log/oak", "tfc:block/wood/stripped_log/oak", "tfc:block/wood/leaves/oak",
        "tfc:block/wood/leaves/oak", "tfc:block/wood/log/oak"]


def main():
    tfc = zipfile.ZipFile(TFC)
    van = zipfile.ZipFile(VANILLA)

    def load(ref):
        mod, path = ref.split(":")
        z, root = (tfc, "assets/tfc/textures/") if mod == "tfc" else (van, "assets/minecraft/textures/")
        w, h, px = png_read(z.read(root + path + ".png"))
        return [px[(y % h) * w + (x % w)] for y in range(TILE) for x in range(TILE)]   # первый кадр, если лента

    def recolor(src, rgb):
        """Рисунок соседней породы как маска светлоты, цвет - наш."""
        lum = [(0.299 * p[0] + 0.587 * p[1] + 0.114 * p[2]) / 255.0 for p in src]
        vis = [l for l, p in zip(lum, src) if p[3]] or lum   # прозрачные точки в среднее не идут
        mid = sum(vis) / len(vis) or 1.0
        out = []
        for l, p in zip(lum, src):
            k = l / mid
            out.append(tuple(min(255, int(255 * c * k + 0.5)) for c in rgb) + (p[3],))
        return out

    def over(top, bottom):
        """Накладка с прозрачностью: полоска травы поверх земли."""
        out = []
        for t, b in zip(top, bottom):
            a = t[3] / 255.0
            out.append(tuple(int(t[i] * a + b[i] * (1 - a) + 0.5) for i in range(3)) + (255,))
        return out

    tiles = [[(255, 0, 255, 255)] * (TILE * TILE) for _ in range(LAYERS)]
    for i, ref in enumerate(BASE):
        if ref: tiles[i] = load(ref)
    # Трава и листва в TFC - серые маски: цвет им даёт палитра, как и в самой игре. Красим их теми
    # же цветами, что стоят в shaders/terrain.vert, чтобы вблизи и вдали был один цвет.
    for i, rgb in [(3, GRASS_RGB)]:   # листва текстуру пока не берёт - у неё свой принцип
        tiles[i] = recolor(tiles[i], rgb)
    for i, name in enumerate(TFC_ROCKS):
        tiles[12 + i] = load("tfc:block/rock/raw/" + name)
    for i, (parent, rgb) in enumerate(MIXED):
        tiles[32 + i] = recolor(load("tfc:block/rock/raw/" + parent), rgb)
    # Р82: обрубленный дёрн - бок блока травы. В TFC это накладка с прозрачностью поверх земли:
    # сверху полоска дёрна, ниже видна сама земля. Накладка - такая же серая маска, красим её.
    tiles[48] = over(recolor(load("tfc:block/grass_side"), GRASS_RGB), tiles[2])
    # Р82: тот же бок, но под снежной шапкой - полоска снега; снег в TFC не красится палитрой
    tiles[49] = over(load("tfc:block/grass_snowy_side"), tiles[2])
    # Р87: именованные почвы - порядок ровно как в src/world/soil_table.cpp
    for i, ref in enumerate(SOILS):
        tiles[50 + i] = load(ref)
    # Р88: руды - накладка поверх камня. Порода-хозяин у вокселя своя, а рисунок пока один и тот же,
    # на граните: отдельная клетка на каждую пару «порода + руда» - это под тысячу клеток.
    for i, ref in enumerate(ORES):
        tiles[59 + i] = over(load(ref), tiles[12])
    # Р106: кора видов - рисунок брёвен TFC (чего в сборке нет - дубовый, перекрашенный)
    for i, (ref, rgb) in enumerate([("birch", None), ("birch", (0.14, 0.14, 0.14)), ("aspen", None), ("spruce", None), ("pine", None),
                                    ("willow", None), ("chestnut", None), ("ash", None), ("hickory", None)]):
        try:
            t = load("tfc:block/wood/log/" + ref)
        except KeyError:
            t = load("tfc:block/wood/log/oak")
        tiles[155 + i] = recolor(t, rgb) if rgb else t
    # Р101: щебень под почвой
    tiles[154] = load("tfc:block/rock/gravel/granite")
    # Р89: дёрн по почвам - верх, бок (полоска травы поверх своей почвы), бок под снегом. Цвет травы у
    # каждой почвы свой (те же числа, что в shaders/terrain.vert): на песке сухая, на торфе тёмная.
    for i, rgb in enumerate(GRASS_BY_SOIL):
        tiles[119 + i] = recolor(load("tfc:block/grass_top"), rgb)
        tiles[128 + i] = over(recolor(load("tfc:block/grass_side"), rgb), tiles[50 + i])
        tiles[137 + i] = over(load("tfc:block/grass_snowy_side"), tiles[50 + i])

    W, H = COLS * TILE, ROWS * TILE
    px = [(0, 0, 0, 0)] * (W * H)
    for n, t in enumerate(tiles):
        tx, ty = (n % COLS) * TILE, (n // COLS) * TILE
        for y in range(TILE):
            for x in range(TILE):
                px[(ty + y) * W + tx + x] = t[y * TILE + x]

    # уровни уменьшения: 16, 8, 4, 2, 1 - усреднением четвёрок, с учётом прозрачности
    def shrink(t, n):
        out = []
        for y in range(n // 2):
            for x in range(n // 2):
                q = [t[(2 * y + dy) * n + 2 * x + dx] for dy in (0, 1) for dx in (0, 1)]
                a = sum(c[3] for c in q)
                if a:
                    out.append(tuple(sum(c[i] * c[3] for c in q) // a for i in range(3)) + (a // 4,))
                else:
                    out.append((0, 0, 0, 0))
        return out

    levels, cur, n = [], tiles, TILE
    while True:
        levels.append(cur)
        if n == 1: break
        cur = [shrink(t, n) for t in cur]
        n //= 2

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(here, "assets")
    os.makedirs(out, exist_ok=True)
    png_write(os.path.join(out, "atlas.png"), W, H, px)
    with open(os.path.join(out, "atlas.bin"), "wb") as f:
        f.write(struct.pack("<HHH", TILE, len(tiles), len(levels)))
        for lv in levels:
            for t in lv:
                f.write(bytes(v for p in t for v in p))
    print("atlas: клеток %d по %d точек, уровней %d; картинка %d x %d" % (len(tiles), TILE, len(levels), W, H))


if __name__ == "__main__":
    sys.exit(main())
