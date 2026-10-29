#pragma once
// Целочисленный хеш (x, z, зерно) -> 32 бита (Р15). Одинаковый на любой машине: беззнаковая арифметика
// в C++ определена по модулю 2^32. Умножение на большие нечётные числа обратимо (разные входы не
// склеиваются) и гонит информацию к старшим битам; сдвиг вправо с xor возвращает её в младшие - вместе
// это «лавина»: соседние координаты дают совсем разные числа.
#include <cstdint>

namespace gen {

constexpr uint32_t hash2(int32_t x, int32_t z, uint32_t seed) {
    uint32_t h = seed;
    h ^= static_cast<uint32_t>(x) * 0x27D4EB2Du;
    h ^= static_cast<uint32_t>(z) * 0x165667B1u;
    h ^= h >> 15; h *= 0x2C1B3C6Du;
    h ^= h >> 12; h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}

// Зерно октавы: у каждой октавы своё, иначе узоры октав совпадают по узлам
constexpr uint32_t octaveSeed(uint32_t seed, int octave) { return seed + static_cast<uint32_t>(octave) * 0x9E3779B9u; }

}  // namespace gen
