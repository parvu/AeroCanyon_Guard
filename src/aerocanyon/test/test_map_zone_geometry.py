"""map_zone_geometry parses real building footprints out of OpenStreetMap
XML -- these tests use small synthetic OSM fixtures, not the full
map_zone.osm (144 ways), except for one sanity check against the real
file.
"""
from aerocanyon.map_zone_geometry import BUILDINGS, OSM_PATH, _parse_buildings

_SQUARE_WITH_HEIGHT = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <node id="3" lat="44.4341000" lon="26.0481000"/>
 <node id="4" lat="44.4341000" lon="26.0480000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/>
  <tag k="height" v="15"/>
 </way>
</osm>
"""

_SQUARE_WITH_LEVELS = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <node id="3" lat="44.4341000" lon="26.0481000"/>
 <node id="4" lat="44.4341000" lon="26.0480000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/>
  <tag k="building:levels" v="4"/>
 </way>
</osm>
"""

_NON_BUILDING_WAY = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/>
  <tag k="highway" v="residential"/>
 </way>
</osm>
"""


def _write(tmp_path, xml):
    p = tmp_path / 'test.osm'
    p.write_text(xml)
    return p


def test_parses_a_building_way_with_an_explicit_height(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _SQUARE_WITH_HEIGHT))
    assert len(boxes) == 1
    assert boxes[0].sz == 15.0
    assert boxes[0].sx > 0.0 and boxes[0].sy > 0.0


def test_falls_back_to_building_levels_times_3m(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _SQUARE_WITH_LEVELS))
    assert len(boxes) == 1
    assert boxes[0].sz == 12.0  # 4 levels * 3.0m


def test_ignores_ways_not_tagged_building(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _NON_BUILDING_WAY))
    assert boxes == []


def test_real_osm_file_parses_to_at_least_one_building():
    assert OSM_PATH.exists(), 'map_zone.osm should be checked into the repo'
    boxes = _parse_buildings(OSM_PATH)
    assert len(boxes) == 26  # map_zone.osm has 26 ways tagged building=*
    for b in boxes:
        assert b.sx > 0.0 and b.sy > 0.0 and b.sz > 0.0


def test_module_level_buildings_matches_parsing_the_real_file():
    assert len(BUILDINGS) == len(_parse_buildings(OSM_PATH))
