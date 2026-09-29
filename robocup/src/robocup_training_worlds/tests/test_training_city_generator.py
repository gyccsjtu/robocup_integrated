#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline tests for the training-city generator.

No ROS, no Gazebo, no GUI, no network. Run with:

    python3 -m unittest discover -s tests -p 'test_*.py'
"""

import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.normpath(os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, SCRIPTS)

import training_city_lib as lib  # noqa: E402
import validate_training_city as validator  # noqa: E402

CONFIG = lib.load_config()
PRESETS = ("unit", "small", "full")
SEEDS = (1, 7, 42)


def build(preset, seed, category=None, **kwargs):
    name = lib.make_world_name(CONFIG, preset, seed, category)
    return lib.build_world(preset, seed, CONFIG, category=category, world_name=name, **kwargs)


def models_of(sdf_text):
    root = ElementTree.fromstring(sdf_text)
    world = root.find("world")
    return world.findall("model")


class DeterminismTests(unittest.TestCase):
    def test_same_seed_is_byte_identical(self):
        for preset in PRESETS:
            first_sdf, first_meta = build(preset, 42, "single_wall" if preset == "unit" else None)
            second_sdf, second_meta = build(preset, 42, "single_wall" if preset == "unit" else None)
            self.assertEqual(first_sdf, second_sdf, "%s: world bytes differ" % preset)
            self.assertEqual(json.dumps(first_meta, sort_keys=True),
                             json.dumps(second_meta, sort_keys=True),
                             "%s: metadata differs" % preset)

    def test_different_seeds_change_unit_layout(self):
        layouts = set()
        for seed in SEEDS:
            _sdf, meta = build("unit", seed, "single_wall")
            layouts.add(tuple(tuple(o["center"]) + (o["height"],) for o in meta["obstacles"]))
        self.assertGreater(len(layouts), 1, "single_wall never moved between seeds")

    def test_different_seeds_change_city_layout(self):
        for preset in ("small", "full"):
            layouts = set()
            for seed in SEEDS:
                _sdf, meta = build(preset, seed)
                layouts.add(json.dumps(meta["obstacles"], sort_keys=True))
            self.assertGreater(len(layouts), 1, "%s layout is seed independent" % preset)

    def test_metadata_has_no_volatile_fields(self):
        _sdf, meta = build("small", 42)
        self.assertNotIn("generated_utc", meta)


class UnitCategoryTests(unittest.TestCase):
    def test_every_category_validates(self):
        for category in lib.UNIT_CATEGORIES:
            expectation = CONFIG["unit_categories"][category]
            for seed in SEEDS:
                _sdf, meta = build("unit", seed, category)
                self.assertEqual([], meta["validation"]["problems"],
                                 "%s seed %d: %s" % (category, seed, meta["validation"]["problems"]))
                self.assertTrue(meta["validation"]["ok"])
                self.assertEqual(expectation["expect_reachable"],
                                 meta["connectivity"]["goals_reachable"] > 0
                                 and all(g["reachable"] for g in meta["goal_candidates"]))

    def test_blocked_categories_have_no_path(self):
        for category in ("blocked", "no_path"):
            for seed in SEEDS:
                _sdf, meta = build("unit", seed, category)
                self.assertFalse(any(g["reachable"] for g in meta["goal_candidates"]),
                                 "%s seed %d should be unreachable" % (category, seed))

    def test_goal_in_obstacle_is_reported_invalid(self):
        for seed in SEEDS:
            _sdf, meta = build("unit", seed, "goal_in_obstacle")
            goal = meta["goal_candidates"][0]
            self.assertFalse(goal["outdoors"])
            self.assertIsNotNone(goal["inside_obstacle"])
            self.assertFalse(goal["valid"])
            self.assertFalse(goal["reachable"])

    def test_positive_categories_are_reachable(self):
        for category in ("empty", "single_wall", "wall_with_gap", "u_shape", "narrow_corridor"):
            for seed in SEEDS:
                _sdf, meta = build("unit", seed, category)
                self.assertTrue(all(g["reachable"] for g in meta["goal_candidates"]),
                                "%s seed %d should be reachable" % (category, seed))

    def test_unit_bounds_and_scale(self):
        _sdf, meta = build("unit", 42, "single_wall")
        self.assertEqual(-5.0, meta["bounds"]["x_min"])
        self.assertEqual(5.0, meta["bounds"]["x_max"])
        self.assertEqual(-5.0, meta["bounds"]["y_min"])
        self.assertEqual(5.0, meta["bounds"]["y_max"])


class PresetScaleTests(unittest.TestCase):
    def test_small_preset_scale(self):
        _sdf, meta = build("small", 42)
        self.assertEqual((-15.0, 15.0, -10.0, 10.0),
                         (meta["bounds"]["x_min"], meta["bounds"]["x_max"],
                          meta["bounds"]["y_min"], meta["bounds"]["y_max"]))

    def test_full_preset_matches_rule_scale(self):
        _sdf, meta = build("full", 42)
        self.assertEqual(-100.0, meta["bounds"]["x_min"])
        self.assertEqual(100.0, meta["bounds"]["x_max"])
        self.assertEqual(-50.0, meta["bounds"]["y_min"])
        self.assertEqual(50.0, meta["bounds"]["y_max"])
        self.assertEqual(200.0, meta["bounds"]["x_max"] - meta["bounds"]["x_min"])
        self.assertEqual(100.0, meta["bounds"]["y_max"] - meta["bounds"]["y_min"])

    def test_all_presets_validate(self):
        for preset in PRESETS:
            for seed in SEEDS:
                _sdf, meta = build(preset, seed, "single_wall" if preset == "unit" else None)
                self.assertEqual([], meta["validation"]["problems"],
                                 "%s seed %d: %s" % (preset, seed, meta["validation"]["problems"]))


class GeometryTests(unittest.TestCase):
    def test_obstacles_stay_inside_bounds(self):
        for preset in PRESETS:
            _sdf, meta = build(preset, 42, "single_wall" if preset == "unit" else None)
            for obstacle in meta["obstacles"]:
                x0, y0, x1, y1 = obstacle["bbox"]
                self.assertGreaterEqual(x0, meta["bounds"]["x_min"] - 1e-6)
                self.assertGreaterEqual(y0, meta["bounds"]["y_min"] - 1e-6)
                self.assertLessEqual(x1, meta["bounds"]["x_max"] + 1e-6)
                self.assertLessEqual(y1, meta["bounds"]["y_max"] + 1e-6)

    def test_no_overlapping_obstacles(self):
        for preset in PRESETS:
            for seed in SEEDS:
                _sdf, meta = build(preset, seed, "single_wall" if preset == "unit" else None)
                obstacles = [lib.Obstacle.from_metadata(o) for o in meta["obstacles"]]
                for i in range(len(obstacles)):
                    for j in range(i + 1, len(obstacles)):
                        left, right = obstacles[i], obstacles[j]
                        if left.group is not None and left.group == right.group:
                            continue
                        self.assertGreater(left.distance_to(right), 1e-6,
                                           "%s seed %d: %s overlaps %s"
                                           % (preset, seed, left.id, right.id))

    def test_spawn_is_clear(self):
        for preset in PRESETS:
            for seed in SEEDS:
                _sdf, meta = build(preset, seed, "single_wall" if preset == "unit" else None)
                obstacles = [lib.Obstacle.from_metadata(o) for o in meta["obstacles"]]
                x, y = meta["spawn"]["center"]
                clearance = min(o.clearance_at(x, y) for o in obstacles if o.blocking)
                self.assertGreaterEqual(clearance, meta["safety"]["spawn_clearance_m"] - 1e-6,
                                        "%s seed %d: spawn clearance %s" % (preset, seed, clearance))
                self.assertGreaterEqual(x, meta["bounds"]["x_min"])
                self.assertLessEqual(x, meta["bounds"]["x_max"])

    def test_goal_candidates_are_outdoors(self):
        for preset in PRESETS:
            for seed in SEEDS:
                _sdf, meta = build(preset, seed, "single_wall" if preset == "unit" else None)
                for goal in meta["goal_candidates"]:
                    self.assertTrue(goal["outdoors"], "%s seed %d: %s" % (preset, seed, goal))
                    self.assertIsNone(goal["inside_obstacle"])

    def test_obstacles_cover_planning_altitude(self):
        for preset in PRESETS:
            _sdf, meta = build(preset, 42, "single_wall" if preset == "unit" else None)
            required = meta["planning_altitude_m"] + meta["safety"]["vertical_margin_m"]
            for obstacle in meta["obstacles"]:
                if not obstacle["blocking"]:
                    continue
                self.assertAlmostEqual(0.0, obstacle["z_min"], places=6)
                self.assertGreaterEqual(obstacle["z_max"], required - 1e-6,
                                        "%s: %s too low" % (preset, obstacle["id"]))

    def test_lamp_posts_have_cylinder_collision(self):
        _sdf, meta = build("small", 42)
        lamps = [o for o in meta["obstacles"] if o["type"] == "lamp_post"]
        self.assertTrue(lamps, "small preset generated no lamp posts")
        for lamp in lamps:
            self.assertEqual("cylinder", lamp["shape"])
            self.assertGreater(lamp["radius"], 0.05)
            self.assertEqual("circle", lamp["footprint"]["kind"])
        models = {model.get("name"): model for model in models_of(_sdf)}
        for lamp in lamps:
            link = models[lamp["id"]].find("link")
            geometry = link.find("collision").find("geometry")
            self.assertIsNotNone(geometry.find("cylinder"))

    def test_buildings_exist_in_city_presets(self):
        for preset in ("small", "full"):
            _sdf, meta = build(preset, 42)
            kinds = set(o["type"] for o in meta["obstacles"])
            self.assertIn("building", kinds)
            self.assertIn("lamp_post", kinds)
            self.assertIn("boundary_wall", kinds)


class ConnectivityTests(unittest.TestCase):
    def test_grid_matches_metadata(self):
        for preset in PRESETS:
            _sdf, meta = build(preset, 42, "single_wall" if preset == "unit" else None)
            problems, checks = lib.validate_metadata(meta)
            self.assertEqual([], problems)
            self.assertTrue(checks["grid_matches_metadata"])

    def test_flood_fill_does_not_squeeze_diagonally(self):
        """Two obstacle corners touching diagonally must not form a passage."""
        bounds = [0.0, 0.0, 2.0, 2.0]
        grid = lib.OccupancyGrid(bounds, 0.5)
        obstacles = [
            lib.Obstacle.box("a", "wall", (0.5, 0.5), (1.0, 1.0), 3.0),
            lib.Obstacle.box("b", "wall", (1.5, 1.5), (1.0, 1.0), 3.0),
        ]
        for obstacle in obstacles:
            grid.mark(obstacle, 0.0)
        visited = grid.flood_fill(0, 3)
        self.assertEqual(0, int(visited[0 * grid.width + 0]))

    def test_connectivity_is_independent_of_a_planner(self):
        _sdf, meta = build("unit", 42, "wall_with_gap")
        self.assertEqual("bfs_4connected_on_inflated_grid", meta["connectivity"]["method"])
        self.assertTrue(meta["connectivity"]["goals_reachable"] >= 1)

    def test_grid_decoding_roundtrip(self):
        _sdf, meta = build("small", 42)
        grid = meta["grid"]
        cells = lib.decode_rle(grid["data"], grid["width"], grid["height"])
        self.assertEqual(grid["width"] * grid["height"], len(cells))
        self.assertEqual(sum(count for _value, count in grid["data"]),
                         grid["width"] * grid["height"])


class SdfTests(unittest.TestCase):
    def _sdf_for(self, preset, category=None):
        sdf, _meta = build(preset, 42, category)
        return sdf

    def test_sdf_is_wellformed_and_versioned(self):
        for preset in PRESETS:
            sdf = self._sdf_for(preset, "single_wall" if preset == "unit" else None)
            root = ElementTree.fromstring(sdf)
            self.assertEqual("sdf", root.tag)
            self.assertEqual("1.5", root.get("version"))
            self.assertEqual(1, len(root.findall("world")))

    def test_model_names_are_unique(self):
        sdf = self._sdf_for("full")
        names = [model.get("name") for model in models_of(sdf)]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(names)

    def test_every_model_has_collision_and_visual(self):
        for preset in PRESETS:
            sdf = self._sdf_for(preset, "single_wall" if preset == "unit" else None)
            for model in models_of(sdf):
                link = model.find("link")
                self.assertIsNotNone(link, model.get("name"))
                collision = link.find("collision")
                visual = link.find("visual")
                self.assertIsNotNone(collision, model.get("name"))
                self.assertIsNotNone(visual, model.get("name"))
                for element in (collision, visual):
                    geometry = element.find("geometry")
                    self.assertIsNotNone(geometry)
                    self.assertTrue(geometry.find("box") is not None
                                    or geometry.find("cylinder") is not None
                                    or geometry.find("plane") is not None)

    def test_collision_and_visual_geometry_match(self):
        sdf = self._sdf_for("small")
        for model in models_of(sdf):
            link = model.find("link")
            collision = link.find("collision").find("geometry")
            visual = link.find("visual").find("geometry")
            self.assertEqual(ElementTree.tostring(collision), ElementTree.tostring(visual))

    def test_world_has_no_vehicle_and_no_plugins(self):
        for preset in PRESETS:
            sdf = self._sdf_for(preset, "single_wall" if preset == "unit" else None)
            root = ElementTree.fromstring(sdf)
            for element in root.iter():
                self.assertNotEqual("plugin", element.tag.rsplit("}", 1)[-1])
            for model in models_of(sdf):
                self.assertNotIn("iris", model.get("name").lower())

    def test_world_has_no_online_dependencies(self):
        for preset in PRESETS:
            sdf = self._sdf_for(preset, "single_wall" if preset == "unit" else None)
            self.assertNotIn("http://", sdf)
            self.assertNotIn("https://", sdf)
            self.assertNotIn("model://", sdf)
            self.assertNotIn("fuel", sdf.lower())

    def test_world_is_static_and_grounded(self):
        sdf = self._sdf_for("unit", "single_wall")
        for model in models_of(sdf):
            self.assertEqual("true", model.find("static").text)


class ValidatorTests(unittest.TestCase):
    def _write(self, sdf, meta, directory, name="world"):
        world_path = os.path.join(directory, name + ".world")
        meta_path = os.path.join(directory, name + ".json")
        with open(world_path, "w", encoding="utf-8") as stream:
            stream.write(sdf)
        with open(meta_path, "w", encoding="utf-8") as stream:
            json.dump(meta, stream, sort_keys=True)
        return world_path, meta_path

    def test_valid_world_passes(self):
        sdf, meta = build("unit", 42, "single_wall")
        with tempfile.TemporaryDirectory() as directory:
            world_path, meta_path = self._write(sdf, meta, directory)
            self.assertEqual([], validator.validate_sdf(world_path, meta["world_name"]))
            self.assertEqual([], validator.validate_sdf(world_path, meta["world_name"]))

    def test_validator_detects_missing_collision(self):
        sdf, meta = build("unit", 42, "single_wall")
        root = ElementTree.fromstring(sdf)
        for model in root.find("world").findall("model"):
            link = model.find("link")
            collision = link.find("collision")
            if collision is not None:
                link.remove(collision)
                break
        broken = ElementTree.tostring(root, encoding="unicode")
        with tempfile.TemporaryDirectory() as directory:
            world_path, _meta_path = self._write(broken, meta, directory, "broken")
            problems = validator.validate_sdf(world_path, meta["world_name"])
            self.assertTrue(any(p.startswith("COLLISION_MISSING") for p in problems))

    def test_validator_detects_duplicate_names(self):
        sdf, meta = build("unit", 42, "single_wall")
        broken = sdf.replace('name="unit_wall_0000"', 'name="boundary_north"', 1)
        with tempfile.TemporaryDirectory() as directory:
            world_path, _meta_path = self._write(broken, meta, directory, "dup")
            problems = validator.validate_sdf(world_path, meta["world_name"])
            self.assertTrue(any(p.startswith("DUPLICATE_MODEL_NAME") for p in problems))

    def test_validator_detects_goal_inside_obstacle(self):
        _sdf, meta = build("small", 42)
        tampered = copy.deepcopy(meta)
        building = [o for o in tampered["obstacles"] if o["type"] == "building"][0]
        tampered["goal_candidates"][0]["center"] = list(building["center"])
        tampered["goal_candidates"][0]["outdoors"] = True
        problems, _checks = lib.validate_metadata(tampered)
        self.assertTrue(any(p.startswith("GOAL_INSIDE_OBSTACLE") for p in problems))

    def test_validator_detects_spawn_inside_obstacle(self):
        _sdf, meta = build("small", 42)
        tampered = copy.deepcopy(meta)
        building = [o for o in tampered["obstacles"] if o["type"] == "building"][0]
        tampered["spawn"]["center"] = list(building["center"])
        problems, _checks = lib.validate_metadata(tampered)
        self.assertTrue(any(p.startswith("SPAWN_TOO_CLOSE") for p in problems))

    def test_validator_detects_out_of_bounds_obstacle(self):
        _sdf, meta = build("unit", 42, "empty")
        tampered = copy.deepcopy(meta)
        tampered["obstacles"].append(
            lib.Obstacle.box("runner", "wall", (40.0, 0.0), (1.0, 1.0), 4.0).to_metadata())
        problems, _checks = lib.validate_metadata(tampered)
        self.assertTrue(any(p.startswith("OBSTACLE_OUTSIDE_BOUNDS") for p in problems))

    def test_generator_refuses_to_write_invalid_world(self):
        """An impossible safety margin must abort generation (no stale map)."""
        config = copy.deepcopy(CONFIG)
        config["safety"]["spawn_clearance_m"] = 9.0
        with tempfile.TemporaryDirectory() as directory:
            config_path = os.path.join(directory, "config.yaml")
            try:
                import yaml
            except ImportError:  # pragma: no cover
                self.skipTest("PyYAML is unavailable")
            with open(config_path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(config, stream)
            world_path = os.path.join(directory, "invalid.world")
            result = subprocess.run(
                [sys.executable, os.path.join(SCRIPTS, "generate_training_city.py"),
                 "--config", config_path, "--preset", "unit", "--seed", "42",
                 "--output", world_path],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            self.assertNotEqual(0, result.returncode)
            self.assertFalse(os.path.exists(world_path),
                             "an invalid world must never be written")


class RasterizerTests(unittest.TestCase):
    def test_pgm_and_yaml_are_written(self):
        _sdf, meta = build("unit", 42, "single_wall")
        with tempfile.TemporaryDirectory() as directory:
            meta_path = os.path.join(directory, "unit.json")
            with open(meta_path, "w", encoding="utf-8") as stream:
                json.dump(meta, stream)
            result = subprocess.run(
                [sys.executable, os.path.join(SCRIPTS, "rasterize_metadata.py"),
                 "--metadata", meta_path],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            self.assertEqual(0, result.returncode, result.stderr)
            pgm_path = os.path.join(directory, "unit.pgm")
            yaml_path = os.path.join(directory, "unit.yaml")
            self.assertTrue(os.path.isfile(pgm_path))
            with open(pgm_path, "rb") as stream:
                self.assertTrue(stream.read(2) == b"P5")
            with open(yaml_path, "r", encoding="utf-8") as stream:
                text = stream.read()
            self.assertIn("resolution:", text)
            self.assertIn("origin:", text)
            self.assertIn("occupied_thresh: 0.65", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
