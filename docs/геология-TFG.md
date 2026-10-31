# Геология TFG и наш план: справка к обсуждению (01.11.2026)

Справка, а не решение. Решения о породах и биомах - в `решения.md`, когда пользователь их примет.
Источник - локальная сборка TerraFirmaGreg 0.12.5 (TFC 3.2.21): `world_preset/overworld.json` из jar TFC и 74 жилы
`kubejs/data/tfg/worldgen/configured_feature/earth/vein/*.json`.

## 1. Как устроено в TFG

### Породы: 20 штук в четырёх группах
- **изверженные глубинные:** гранит, диорит, габбро;
- **изверженные излившиеся:** риолит, базальт, андезит, дацит;
- **осадочные:** аргиллитовый сланец (shale), аргиллит (claystone), известняк, конгломерат, доломит, кремень (chert), мел;
- **метаморфические:** кварцит, кровельный сланец (slate), филлит, кристаллический сланец (schist), гнейс, мрамор.

В жилах TFG встречаются ещё ванильные туф (31 жила) и базальт (30): у руды бывает вариант «tuff_...».

### Разрез: стопки слоёв
У каждого вида слоя для каждой его породы записано, на каком слое она лежит. Цепочки сходятся к нижнему ярусу:
гнейс, кристаллический сланец, диорит, гранит, габбро. Тип района задаёт, с каких слоёв начинается столбец:

| район | верхние слои |
|---|---|
| дно океана | излившиеся |
| вулканический | излившиеся, двойной слой излившихся |
| суша | излившиеся или осадочные |
| поднятие (горы) | осадочные или «поднятие» |

Пары пород «сверху → под ней» геологически связаны:
- **излившиеся:** риолит → гранит; андезит и дацит → диорит; базальт → габбро;
- **осадочные:** shale, claystone и конгломерат → кровельный сланец → филлит → гнейс, кристаллический сланец;
  известняк, доломит и мел → мрамор; кремень → кварцит;
- **поднятие:** гранит, диорит и габбро лежат поверх осадочных - разрез гор перевёрнут, глубинное вынесено наверх.

В столбце 3-5 пород сверху вниз. Толщины слоёв задаёт шум. Районы - сотни и тысячи блоков. Их тип даёт генератор
тектоники TFC: плиты, их границы - горы и вулканы.

### Жилы: 74 штуки
Каждая жила задаётся так:
- **форма:**
  - `disc_vein` - диск: радиус `size`, толщина `height`;
  - `cluster_vein` - ком;
  - `pipe_vein` - вертикальная труба высотой 30-150, с наклоном и перекосом;
- **высота** y (мир TFC от -64 до 320, море на 63), редкость (1 на N чанков), плотность (доля заменённых блоков);
- **замены по породам:** блок породы → руды с весами. Руда - отдельный блок на каждую породу («гранитный халькопирит»),
  изредка - сплошной блок сырья;
- **индикатор:** камешки руды на поверхности над жилой (у всех 74), под землёй - россыпь мелочи.

Пояса:
- **surface:** от поверхности (y от -20 до -5, `project`);
- **high:** уголь, гипс и сера на 90-170, горные руды - от поверхности в горах;
- **normal:** y 10..70;
- **deep:** y -50..20;
- **sand:** россыпи в песке у поверхности.

**Порода решает, какие руды вообще возможны в месте** - это главное правило баланса. Биомы влияют на 14 жил через
метки:
- **горы:** 5 жил;
- **вулканы:** 3;
- **холмы:** 1;
- **засушливо:** 1;
- **карст:** 1;
- **берег и остров:** 1;
- **«обычные» места:** 2.

## 2. Что уже есть в Отмели
- **Р12:** руды - настоящие тела в объёме.
- **Р14, Р54:** генерируемое лежит блоками по сетке; блок = 1 м = 16³ вокселей.
- **Р10:** высота мира -500..+1000 блоков. Это вчетверо глубже TFC (384) - есть место для настоящих разрезов.
- **Р19:** биом = форма (равнины, холмы, горы - непрерывными весами) + климат (температура, осадки), без перечня.
- **Р26:** пещеры. Карст отложен до появления пород (Р27).
- **Р29-Р32:** реки прорезают долины, то есть вскрывают слои.
- **Нет:** пород (всё - «камень»), руд, вулканизма, тектоники (горы идут от шума рельефности, а не от плит).

## 3. Сверка
| TFG | Отмель | как сшить |
|---|---|---|
| 20 пород и пары «слой → под ним» | ничего | взять как есть (баланс TFG, Р7) |
| стопки по типу района | нет районов | тип района выводить из наших полей: океан - доля суши < 1; поднятие - вес гор; вулкан - **новое поле**; суша - остальное |
| метки биомов у жил | непрерывные поля | метки становятся условиями на поля: вес гор > x, вулканизм, осадки < y, карст (известняк или доломит у поверхности и дожди), берег (у моря), холмы - вес холмов |
| высоты жил в y TFC | -500..+1000 | пояса surface и normal - от поверхности; deep - абсолютные глубины, растянутые под нашу высоту |
| диски, комы, трубы | тела в объёме (Р12) | формулы тел, место - по хешу ячейки (как лес): без глобального прохода (Р2) |
| индикаторы на поверхности | - | камешки над жилой |

## 4. Предложение (Claude) - до обсуждения
- **Порода в точке - формулой** (Р2). По столбцу:
  - тип района из полей;
  - цепочка из 3-5 слоёв;
  - границы слоёв: z_i(x, z) = поверхность0 − Σ толщин, толщины - медленный шум.

  В блоке остаётся сравнить глубину с 3-5 границами - единицы наносекунд.
- **Границы - от поверхности до эрозии** (исходный рельеф без рек и долин, `baseReliefAt`), а не от нынешней.
  Тогда долина реки сама вскрывает нижние слои: на склонах каньона видны пласты. У TFC разрез привязан к нынешней
  поверхности и в каньоне пластов не видно.
- **Складки и наклон пластов в горах:** границы смещаются вдоль хребта (синус или шум по оси хребта) - горы
  выходят полосатыми, как настоящие разрезы.
- **Жилы - ячейками** (как лес): горизонтальная ячейка ~ размер жилы × редкость, по глубине - пояса.
  - Хеш ячейки даёт кандидата: вид жилы по весам пояса и района, центр, размер, форму, наклон.
  - Руда ставится только там, где порода в точке есть в списке жилы.
  - Индикатор - сверху над центром.
- **Карст:** известняк или доломит у поверхности плюс дожди - воронки и карстовые пещеры.

## 5. Вопросы пользователю
1. Породы: все 20 TFC (+ туф) или свой или меньший набор?
2. Разрез: (а) стопки TFC как есть; (б) с границами до эрозии, складками и наклоном; (в) TFC + складки только в горах?
3. Районы: тектоника плит или тип района из наших полей + новое поле вулканизма?
4. Толщины осадочного чехла, метаморфики и фундамента (в блоках) при высоте -500..+1000?
5. Руды: таблица TFG «порода → руды, веса» как есть? Пояса - от поверхности или абсолютные?
6. Размер тел: в TFG диск 20-45 блоков, а наш мир больше - масштабировать размер или редкость?
7. Биомы: оставить непрерывными (Р19) с условиями вместо меток, без перечня?
8. Пласты на склонах и в обрывах (полосатость гор) - хотим?

## 6. Жилы TFG (все 74)
Группы пород: ИГ-глуб - гранит, диорит, габбро; ИГ-изл - риолит, базальт, андезит, дацит; ОСАД - осадочные;
МЕТ - метаморфические. Высота - y TFC. У поясов surface и high (часть) - от поверхности.

| ���� | ����� | ������ (TFC y) | ������ / �������� | ������ | ���� (����) |
|---|---|---|---|---|---|
| deep_apatite | ��� | -50..20 | 37 / 190 | ��-���� | apatite 48%, tricalcium_phosphate 36%, pyrochlore 12%, raw_apatite_block 3% |
| deep_asbestos | ���� | -50..20 | 35 / 190 | ��� | cassiterite 53%, asbestos 32%, diatomite 11%, raw_cassiterite_block 5% |
| deep_banded_iron | ��� | -50..20 | 45 / 280 | ����, ��� | goethite 38%, yellow_limonite 32%, hematite 19%, raw_goethite_block 3%, tuff_goethite 3%, tuff_yellow_limonite 2%, tuff_hematite 1% |
| deep_bauxite | ��� | -50..20 | 27 / 420 | ��-���� | ilmenite 67%, bauxite 29%, raw_ilmenite_block 5% |
| deep_big_redstone | ���� | -50..20 | 42 / 270 | ��-���� | redstone 49%, ruby 29%, cinnabar 20%, raw_redstone_block 2% |
| deep_bismuth | ����� | -50..20 | h 60 / 140 | ��-���� | bismuth 31%, silver 27%, gold 27%, electrotine 14%, raw_electrotine_block 2% |
| deep_copper_iron | ��� | -50..20 | 45 / 270 | ��-����, ��-��� | chalcopyrite 36%, goethite 30%, pyrite 18%, copper 12%, raw_chalcopyrite_block 3% |
| deep_copper_sulfide | ���� | -60..-45 | 27 / 34 | ��� | sulfur 76%, bornite 10%, chalcocite 10%, raw_sulfur_block 5% |
| deep_copper_tin | ��� | -50..20 | 42 / 210 | ��-����, ��-��� | chalcopyrite 54%, zeolite 23%, cassiterite 23% |
| deep_galena | ��� | -50..20 | 24 / 230 | ��-���� | galena 50%, silver 25%, lead 21%, raw_galena_block 4% |
| deep_graphite | ����� | -50..0 | h 80 / 120 | ��-���� | graphite 67%, coal 19%, diamond 10%, raw_graphite_block 5% |
| deep_graphite_volcanic | ����� | -50..0 | h 150 / 140 | ��-���� | graphite 57%, coal 19%, diamond 19%, raw_graphite_block 5% |
| deep_iron | ��� | -50..20 | 37 / 160 | ����, ��� | goethite 29%, yellow_limonite 21%, hematite 21%, malachite 21%, raw_goethite_block 2%, tuff_goethite 2%, tuff_yellow_limonite 2%, tuff_hematite 2%, tuff_malachite 2% |
| deep_kyanite | ��� | -50..20 | 35 / 160 | ��� | kyanite 47%, mica 31%, pollucite 20%, raw_kyanite_block 2% |
| deep_lazurite_calcite | ����� | -50..20 | h 60 / 130 | ��� | lazurite 32%, sodalite 26%, lapis 19%, calcite 19%, raw_lazurite_block 3% |
| deep_magnetite | ��� | -50..20 | 41 / 160 | ��-���, ��� | magnetite 49%, vanadium_magnetite 29%, gold 20%, raw_magnetite_block 2% |
| deep_magnetite_olivine | ��� | -50..20 | 37 / 160 | ��-���, ��� | fullers_earth 44%, bentonite 33%, magnesite 15%, olivine 7%, raw_bentonite_block 2% |
| deep_manganese | ��� | -50..20 | 37 / 240 | ��-����, ���� | grossular 38%, pyrolusite 34%, tantalite 17%, tuff_grossular 4%, tuff_pyrolusite 3%, raw_tantalite_block 2%, tuff_tantalite 2% |
| deep_molybdenum | ��� | -50..20 | 38 / 210 | ��-���� | molybdenite 55%, wulfenite 28%, molybdenum 7%, powellite 7%, raw_molybdenite_block 3% |
| deep_monazite | ��� | -50..20 | 27 / 290 | ��� | bastnasite 57%, monazite 29%, neodymium 10%, raw_bastnasite_block 5% |
| deep_nickel | ��� | -50..20 | 39 / 180 | ��-���� | garnierite 36%, cobaltite 26%, nickel 21%, pentlandite 15%, raw_garnierite_block 3% |
| deep_redstone | ��� | -50..20 | 28 / 180 | ��-���� | redstone 53%, ruby 27%, cinnabar 13%, raw_redstone_block 7% |
| deep_saltpeter | ��� | -50..20 | 32 / 260 | ����, ��� | saltpeter 44%, diatomite 18%, electrotine 15%, alunite 15%, tuff_saltpeter 3%, raw_saltpeter_block 2%, tuff_diatomite 1%, tuff_electrotine 1%, tuff_alunite 1% |
| deep_sapphire | ����� | -50..20 | h 60 / 180 | ��� | sapphire 32%, green_sapphire 26%, pyrope 24%, almandine 16%, raw_almandine_block 2% |
| deep_scheelite | ��� | -50..20 | 25 / 420 | ��� | scheelite 41%, tungstate 41%, lithium 14%, raw_lithium_block 3% |
| deep_sphalerite | ��� | -50..20 | 29 / 180 | ��� | sphalerite 64%, pyrite 27%, sulfur 9% |
| deep_sulfur | ���� | -60..-45 | 35 / 34 | ��-����, ��-���, ����, ��� | sulfur 91%, raw_sulfur_block 5%, tuff_sulfur 5% |
| deep_tantalite | ��� | -50..20 | 31 / 290 | ��� | grossular 33%, spessartine 27%, pyrolusite 22%, tantalite 16%, raw_grossular_block 1% |
| deep_thorianite | ��� | -50..20 | 31 / 270 | ��-���� | thorium 42%, uraninite 21%, beryllium 21%, emerald 11%, raw_beryllium_block 5% |
| deep_tin | ��� | -50..20 | 33 / 210 | ��� | cassiterite 82%, tin 12%, raw_cassiterite_block 6% |
| deep_topaz | ��� | -50..20 | 27 / 230 | ��-���� | blue_topaz 42%, topaz 30%, chalcocite 18%, bornite 6%, raw_blue_topaz_block 3% |
| high_bismuth_mountain | ��� | -30..0 | 15 / 50 | ��-����, ��-���, ����, ��� | bismuth 95%, tuff_bismuth 5% |
| high_coal | ��� | 90..160 | 30 / 30 | ��-����, ��-���, ����, ��� | coal 93%, tuff_coal 5%, raw_coal_block 2% |
| high_gypsum | ��� | 90..140 | 15 / 40 | ��-����, ��-���, ����, ��� | gypsum 94%, tuff_gypsum 5%, raw_gypsum_block 1% |
| high_lead | ��� | -30..0 | 25 / 60 | ��-����, ��-���, ����, ��� | lead 57%, silver 38%, tuff_lead 3%, tuff_silver 2% |
| high_realgar | ���� | -30..1 | 20 / 70 | ��-����, ��-���, ����, ��� | chalcopyrite 54%, zeolite 27%, realgar 13%, tuff_chalcopyrite 3%, raw_chalcopyrite_block 1%, tuff_zeolite 1%, tuff_realgar 1% |
| high_sulfur | ��� | 100..170 | 15 / 30 | ��-����, ��-���, ����, ��� | sulfur 95%, tuff_sulfur 5% |
| high_tetrahedrite | ��� | -30..1 | 25 / 90 | ��-����, ��-���, ����, ��� | tetrahedrite 47%, copper 28%, stibnite 19%, tuff_tetrahedrite 2%, tuff_copper 1%, raw_tetrahedrite_block 1%, tuff_stibnite 1% |
| high_tin | ��� | -50..0 | 35 / 50 | ��-����, ��-���, ����, ��� | cassiterite 57%, tin 38%, tuff_cassiterite 3%, tuff_tin 2% |
| normal_asbestos_dry | ���� | 10..70 | 20 / 130 | ����, ��� | cassiterite 80%, asbestos 13%, tuff_cassiterite 6%, tuff_asbestos 1% |
| normal_coal | ���� | 10..70 | 34 / 230 | ��� | lignite 59%, coal 25%, tuff_lignite 10%, tuff_coal 4%, raw_lignite_block 1% |
| normal_copper | ���� | 10..70 | 30 / 130 | ��-����, ���� | chalcopyrite 48%, pyrite 28%, copper 14%, tuff_chalcopyrite 5%, tuff_pyrite 3%, raw_chalcopyrite_block 2%, tuff_copper 1% |
| normal_copper_gypsum | ����� | -30..80 | h 35 / 190 | ��-����, ��-���, ����, ��� | chalcocite 51%, cobalt 21%, gypsum 13%, sulfur 9%, tuff_chalcocite 3%, raw_chalcocite_block 1%, tuff_cobalt 1%, tuff_gypsum 1%, tuff_sulfur 0% |
| normal_galena | ���� | 10..70 | 23 / 150 | ��-����, ��-��� | galena 77%, silver 22%, raw_galena_block 1% |
| normal_garnierite | ���� | 10..70 | 25 / 170 | ���� | garnierite 45%, cobaltite 26%, nickel 15%, tuff_garnierite 6%, tuff_cobaltite 4%, tuff_nickel 2%, raw_garnierite_block 1% |
| normal_gold | ���� | 10..70 | 24 / 140 | ��-���, ��� | gold 57%, magnetite 43% |
| normal_iron | ���� | 10..70 | 24 / 130 | ��-���, ���� | yellow_limonite 58%, hematite 33%, tuff_yellow_limonite 5%, tuff_hematite 3%, raw_yellow_limonite_block 1% |
| normal_mica | ���� | 10..70 | 21 / 150 | ��-���, ��� | mica 58%, kyanite 42% |
| normal_quartz | ���� | 10..70 | 21 / 180 | ���� | quartzite 65%, barite 22%, tuff_quartzite 9%, tuff_barite 3%, raw_quartzite_block 1% |
| normal_quartz_borax | ����� | -30..70 | h 30 / 190 | ��-��� | quartzite 86%, borax 14% |
| normal_salt | ���� | 10..70 | 25 / 190 | ��-����, ��-���, ����, ��� | salt 57%, rock_salt 28%, borax 9%, tuff_salt 3%, tuff_rock_salt 1%, raw_salt_block 1%, tuff_borax 0% |
| normal_sphalerite | ���� | 10..70 | 18 / 150 | ���� | sphalerite 67%, galena 20%, tuff_sphalerite 10%, tuff_galena 3% |
| normal_spodumene | ���� | 10..70 | 20 / 220 | ����, ��� | spodumene 69%, rock_salt 23%, tuff_spodumene 5%, tuff_rock_salt 2%, raw_spodumene_block 1% |
| normal_sulfide | ���� | 10..70 | 32 / 150 | ��-����, ��-���, ����, ��� | bornite 53%, sulfur 24%, stibnite 18%, tuff_bornite 3%, raw_bornite_block 1%, tuff_sulfur 1%, tuff_stibnite 1% |
| normal_tarkianite | ����� | 10..70 | h 60 / 170 | ��-���, ��� | tarkianite 53%, borax 30%, trona 15%, raw_tarkianite_block 2% |
| sand_basaltic | ��� | -5..1 | 34 / 190 | �����/������ | sand_basaltic_mineral_sand 57%, sand_glauconite_sand 36%, sand_garnet_sand 7% |
| sand_granitic | ��� | -5..1 | 34 / 190 | �����/������ | sand_granitic_mineral_sand 62%, sand_fullers_earth 31%, sand_garnet_sand 8% |
| sand_tin | ��� | -5..1 | 15 / 270 | �����/������ | sand_cassiterite_sand 100% |
| surface_bismuth | ��� | -30..-10 | 15 / 70 | ��-���, ���� | bismuth 92%, tuff_bismuth 8% |
| surface_coal | ��� | -20..1 | 15 / 90 | ���� | lignite 100% |
| surface_cobaltite | ��� | -20..-5 | 15 / 120 | ��-���, ��� | cobaltite 99%, raw_cobaltite_block 1% |
| surface_gold | ��� | -20..-5 | 15 / 85 | ��-��� | gold 100% |
| surface_hematite | ��� | -20..-5 | 15 / 75 | ��-��� | hematite 99%, raw_hematite_block 1% |
| surface_limonite | ��� | -20..-5 | 15 / 120 | ���� | yellow_limonite 87%, tuff_yellow_limonite 12%, raw_yellow_limonite_block 1% |
| surface_magnetite | ��� | -20..-5 | 15 / 120 | ���� | magnetite 87%, tuff_magnetite 12%, raw_magnetite_block 1% |
| surface_malachite | ��� | -20..-5 | 15 / 70 | ����, ��� | malachite 93%, tuff_malachite 7% |
| surface_mica | ��� | -20..-5 | 10 / 120 | ��-����, ��-��� | mica 99%, raw_mica_block 1% |
| surface_native_copper | ��� | -20..-5 | 15 / 70 | ��-��� | copper 99%, raw_copper_block 1% |
| surface_nickel | ��� | -20..-5 | 15 / 90 | ���� | garnierite 88%, tuff_garnierite 12% |
| surface_salpeter | ��� | -20..-5 | 20 / 80 | ��-��� | saltpeter 99%, raw_saltpeter_block 1% |
| surface_sphalerite | ��� | -20..-5 | 15 / 80 | ��-����, ��-��� | sphalerite 100% |
| surface_sylvite_salt | ��� | -20..-5 | 15 / 80 | ����, ��� | salt 56%, rock_salt 37%, tuff_salt 4%, tuff_rock_salt 3% |
| surface_tetrahedrite | ��� | -20..-5 | 15 / 70 | ��-����, ��� | tetrahedrite 100% |
| surface_tricalcium_phosphate | ��� | -20..-5 | 15 / 90 | ���� | tricalcium_phosphate 88%, tuff_tricalcium_phosphate 12% |
