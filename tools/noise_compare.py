"""Сравнение видов шума для рельефа: одно зерно, одно сложение октав, разные шумы.

python tools/noise_compare.py  ->  docs/шумы.png

Хеш целочисленный (как будет в движке): одинаковый ответ на любой машине.
Рисуем карту высот с отмывкой рельефа (свет с северо-запада), чтобы было видно форму, а не только яркость.
"""
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

N = 384          # пикселей на сторону картинки
SCALE = 1 / 96   # частота первой октавы: одна «ячейка» шума на 96 пикселей
OCT = 6          # октав
SEED = 1337
M = 0xFFFFFFFF


def h32(x, z, seed):
    """Целочисленный хеш (x, z, зерно) -> uint32. Умножения по модулю 2^32, сдвиги переносят старшие биты вниз."""
    x = x.astype(np.uint32)
    z = z.astype(np.uint32)
    h = np.full(x.shape, seed, dtype=np.uint32)
    with np.errstate(over="ignore"):
        h ^= x * np.uint32(0x27D4EB2D)
        h ^= z * np.uint32(0x165667B1)
        h ^= h >> np.uint32(15); h *= np.uint32(0x2C1B3C6D)
        h ^= h >> np.uint32(12); h *= np.uint32(0x297A2D39)
        h ^= h >> np.uint32(15)
    return h


def fade(t):  # квинтика Перлина: гладкая вторая производная, без «швов» на рёбрах ячеек
    return t * t * t * (t * (t * 6 - 15) + 10)


def value_noise(x, z, seed):
    """Шум значений: случайное ЧИСЛО в узлах сетки, между ними - сглаженная интерполяция."""
    xi, zi = np.floor(x).astype(np.int64), np.floor(z).astype(np.int64)
    fx, fz = fade(x - xi), fade(z - zi)
    v = lambda a, b: h32(xi + a, zi + b, seed) / M * 2 - 1
    top = v(0, 0) + (v(1, 0) - v(0, 0)) * fx
    bot = v(0, 1) + (v(1, 1) - v(0, 1)) * fx
    return top + (bot - top) * fz


GRAD = np.array([[np.cos(a), np.sin(a)] for a in np.arange(16) * 2 * np.pi / 16])


def perlin(x, z, seed):
    """Градиентный шум Перлина: в узлах - случайное НАПРАВЛЕНИЕ, значение - скалярное произведение на смещение."""
    xi, zi = np.floor(x).astype(np.int64), np.floor(z).astype(np.int64)
    fx, fz = x - xi, z - zi

    def g(a, b):
        gr = GRAD[h32(xi + a, zi + b, seed) & 15]
        return gr[..., 0] * (fx - a) + gr[..., 1] * (fz - b)
    u, w = fade(fx), fade(fz)
    top = g(0, 0) + (g(1, 0) - g(0, 0)) * u
    bot = g(0, 1) + (g(1, 1) - g(0, 1)) * u
    return (top + (bot - top) * w) * 1.4


F2, G2 = 0.5 * (np.sqrt(3) - 1), (3 - np.sqrt(3)) / 6


SIMPLEX_K = 0.6   # у симплекса ячейка мельче, чем у Перлина: приводим к тому же размеру черт для честного сравнения


def simplex(x, z, seed):
    """Симплекс-шум: сетка из треугольников, 3 вершины вместо 4, вклады затухают по радиусу - без осевых швов."""
    x, z = x * SIMPLEX_K, z * SIMPLEX_K
    s = (x + z) * F2
    i, j = np.floor(x + s).astype(np.int64), np.floor(z + s).astype(np.int64)
    t = (i + j) * G2
    x0, z0 = x - (i - t), z - (j - t)
    i1 = (x0 > z0).astype(np.int64); j1 = 1 - i1
    x1, z1 = x0 - i1 + G2, z0 - j1 + G2
    x2, z2 = x0 - 1 + 2 * G2, z0 - 1 + 2 * G2

    def corner(dx, dz, ci, cj):
        tt = 0.5 - dx * dx - dz * dz
        gr = GRAD[h32(ci, cj, seed) & 15]
        return np.where(tt > 0, (tt ** 4) * (gr[..., 0] * dx + gr[..., 1] * dz), 0.0)
    return 70 * (corner(x0, z0, i, j) + corner(x1, z1, i + i1, j + j1) + corner(x2, z2, i + 1, j + 1))


def fbm(fn, x, z, seed, octaves=OCT, lac=2.0, gain=0.5):
    """Сумма октав: каждая следующая - в lac раз чаще и в gain раз тише. Своё зерно у каждой октавы."""
    total, amp, freq, norm = 0.0, 1.0, 1.0, 0.0
    for o in range(octaves):
        total = total + fn(x * freq, z * freq, (seed + o * 0x9E3779B9) & M) * amp
        norm += amp
        amp *= gain
        freq *= lac
    return total / norm


def ridged(fn, x, z, seed):
    """Хребты: 1 - |шум|, возведённое в квадрат - острые гребни гор."""
    total, amp, freq, norm = 0.0, 1.0, 1.0, 0.0
    for o in range(OCT):
        n = 1 - np.abs(fn(x * freq, z * freq, (seed + o * 0x9E3779B9) & M))
        total = total + n * n * amp
        norm += amp; amp *= 0.5; freq *= 2
    return total / norm * 2 - 1


def warped(fn, x, z, seed):
    """Искажение области: координаты сдвигаются другим шумом - формы становятся текучими, «природными»."""
    # сдвиг - медленным шумом (две октавы, вдвое реже основной) и умеренной силы: формы текут, но не рябят
    wx = fbm(fn, x * 0.5 + 5.2, z * 0.5 + 1.3, seed + 11, 2)
    wz = fbm(fn, x * 0.5 + 9.7, z * 0.5 + 2.8, seed + 23, 2)
    return fbm(fn, x + 0.8 * wx, z + 0.8 * wz, seed)


def shade(hm):
    """Отмывка рельефа: свет с северо-запада по наклону, плюс высота цветом от низин к вершинам."""
    gx = np.gradient(hm, axis=1) * 40
    gz = np.gradient(hm, axis=0) * 40
    nx, nz, ny = -gx, -gz, np.ones_like(hm)
    ln = np.sqrt(nx * nx + nz * nz + ny * ny)
    light = np.clip((nx * -0.6 + nz * -0.6 + ny * 0.53) / ln, 0, 1)
    h = (hm - hm.min()) / (np.ptp(hm) + 1e-9)
    low = np.array([70, 110, 60]); mid = np.array([130, 120, 80]); high = np.array([235, 235, 240])
    col = np.where(h[..., None] < 0.6, low + (mid - low) * (h[..., None] / 0.6), mid + (high - mid) * ((h[..., None] - 0.6) / 0.4))
    water = h < 0.3
    col[water] = np.array([50, 80, 140])
    rgb = col * (0.35 + 0.65 * light[..., None])
    return np.clip(rgb, 0, 255).astype(np.uint8)


def main():
    zz, xx = np.mgrid[0:N, 0:N].astype(np.float64)
    x, z = xx * SCALE, zz * SCALE
    panels = [
        ("Шум значений (fBm)", fbm(value_noise, x, z, SEED)),
        ("Перлин (fBm)", fbm(perlin, x, z, SEED)),
        ("Симплекс (fBm)", fbm(simplex, x, z, SEED)),
        ("Симплекс, хребты", ridged(simplex, x, z, SEED)),
        ("Симплекс, искажение области", warped(simplex, x, z, SEED)),
        ("Перлин, искажение области", warped(perlin, x, z, SEED)),
    ]
    pad, cap = 8, 26
    W = 3 * N + 4 * pad
    H = 2 * (N + cap) + 3 * pad
    img = Image.new("RGB", (W, H), (24, 24, 28))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    except Exception:
        font = ImageFont.load_default()
    for k, (title, hm) in enumerate(panels):
        cx, cy = pad + (k % 3) * (N + pad), pad + (k // 3) * (N + cap + pad)
        d.text((cx + 2, cy + 2), title, fill=(220, 220, 220), font=font)
        img.paste(Image.fromarray(shade(hm)), (cx, cy + cap))
    out = os.path.join(os.path.dirname(__file__), "..", "docs", "шумы.png")
    img.save(out)
    print("сохранено:", os.path.abspath(out))


if __name__ == "__main__":
    main()
