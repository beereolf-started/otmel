# Выгрузка данных TFG (TerraFirmaGreg 0.12.5, TFC 3.2.21) в docs/data/tfg - чтобы не собирать массивы руками.
# Запуск: python -I tools/extract_tfg.py <папка игры TFG> <папка репозитория>
#   rocks.json   - 20 пород TFC: группа, слои разреза (порода -> слой под ней), типы районов
#   veins.json   - 74 жилы TFG как есть (форма, высоты, редкость, замены по породам, индикаторы)
#   biomes.json  - биомы TFC и TFG (земля): имя, климат и теги из файлов биомов
#   biome_tags.json - теги биомов TFG (горы, вулканы, карст...) - кто в них входит
#   soils.json   - почвы и грунты TFC (по именам блоков)
import json, os, re, sys, zipfile

game, repo = sys.argv[1], sys.argv[2]
out = os.path.join(repo, 'docs', 'data', 'tfg')
os.makedirs(out, exist_ok=True)
mods = os.path.join(game, 'mods')
tfc = zipfile.ZipFile(os.path.join(mods, next(f for f in os.listdir(mods) if f.startswith('TerraFirmaCraft-Forge'))))
tfg = zipfile.ZipFile(os.path.join(mods, next(f for f in os.listdir(mods) if f.startswith('TerraFirmaGreg-Core'))))

def save(name, obj):
    with open(os.path.join(out, name), 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    print(name, len(obj) if hasattr(obj, '__len__') else '')

# породы и разрез
preset = json.loads(tfc.read('data/tfc/worldgen/world_preset/overworld.json'))
rl = preset['dimensions']['minecraft:overworld']['generator']['tfc_settings']['rock_layer_settings']
groups = {'igneous_intrusive': ['granite', 'diorite', 'gabbro'], 'igneous_extrusive': ['rhyolite', 'basalt', 'andesite', 'dacite'],
          'sedimentary': ['shale', 'claystone', 'limestone', 'conglomerate', 'dolomite', 'chert', 'chalk'],
          'metamorphic': ['quartzite', 'slate', 'phyllite', 'schist', 'gneiss', 'marble']}
save('rocks.json', {'groups': groups, 'rock_layer_settings': rl})

# жилы
vd = os.path.join(game, 'kubejs', 'data', 'tfg', 'worldgen', 'configured_feature', 'earth', 'vein')
save('veins.json', {f[:-5]: json.load(open(os.path.join(vd, f), encoding='utf-8')) for f in sorted(os.listdir(vd)) if f.endswith('.json')})

# биомы
def biomes(z, prefix):
    res = {}
    for n in z.namelist():
        m = re.match(prefix + r'([a-z_]+)\.json$', n)
        if not m: continue
        j = json.loads(z.read(n))
        res[m.group(1)] = {k: j[k] for k in ('temperature', 'downfall', 'has_precipitation', 'temperature_modifier') if k in j}
    return res
save('biomes.json', {'tfc': biomes(tfc, 'data/tfc/worldgen/biome/'), 'tfg_earth': biomes(tfg, 'data/tfg/worldgen/biome/earth/')})

tags = {}
for z in (tfc, tfg):
    for n in z.namelist():
        if '/tags/worldgen/biome/' in n and n.endswith('.json'):
            tags[n.split('/tags/worldgen/biome/')[1][:-5] + ' (' + n.split('/')[1] + ')'] = json.loads(z.read(n)).get('values')
kt = os.path.join(game, 'kubejs', 'data')
for root, _, files in os.walk(kt):
    if 'worldgen' + os.sep + 'biome' in root and os.sep + 'tags' + os.sep in root:
        for f in files:
            if f.endswith('.json'):
                tags[os.path.relpath(os.path.join(root, f), kt)] = json.load(open(os.path.join(root, f), encoding='utf-8')).get('values')
save('biome_tags.json', tags)

# почвы TFC: по моделям блоков (тип почвы × вид блока)
soils = sorted({m.group(1) + '/' + m.group(2) for n in tfc.namelist()
                for m in [re.match(r'assets/tfc/blockstates/(dirt|grass|clay|clay_grass|mud|rooted_dirt|farmland|path|kaolin_clay|mud_bricks)/([a-z_]+)\.json$', n)] if m})
save('soils.json', soils)
