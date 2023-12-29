import unittest
import matplotlib.pyplot as plt
import math
import rclpy
from rclpy.node import Node
from local_planner.local_planner_node import LocalPlanner
import datetime
import os
import re
import pandas as pd
from parameterized import parameterized
from ament_index_python.packages import get_package_share_directory
import shapely.geometry as geom

from eufs_msgs.msg import ConeWithCovariance, ConeArrayWithCovariance
from geometry_msgs.msg import Point

base_dir = get_package_share_directory("local_planner")
csv_dir = f"{base_dir}/csv"
#vis_dir = f"{base_dir}/vis/{datetime.datetime.now().strftime('%d%m%y_%H%M%S')}"

#temporary vis dir
path = os.path.realpath(__file__)
vis_dir = f"{re.search('.*/', path)[0]}vis/{datetime.datetime.now().strftime('%d%m%y_%H%M%S')}"

ACCEPTABLE_DIST = 2 # distance planned point are allowed to be from human midline.
fail_msg = "Local planner failed. Output produced."
fake_covariance = [0] * 4

# rotate point r radians around origin
rotate = lambda x, y, r: (x * math.cos(r) - y * math.sin(r), y * math.cos(r) + x * math.sin(r))

# load a track from csv, returning a cone array with covariance and a dictionary of linestrings
def from_csv(filename):
    df = pd.read_csv(filename, index_col = 0)
    df["x"] -= df.loc["car_start"]["x"]
    df["y"] -= df.loc["car_start"]["y"]

    rads = df.loc["car_start"]["direction"]
    df["x"], df["y"] = rotate(df["x"], df["y"], rads)


    indexes = set(df.index)
    indexes.discard("car_start")

    make_linestring = lambda name: geom.LineString(zip(df.loc[name]["x"], df.loc[name]["y"]))
    cone = lambda row: ConeWithCovariance(point = Point(x = row["x"], y = row["y"]), covariance = fake_covariance)
    cones = lambda colour: [cone(row[1]) for row in df.loc[colour].iterrows()] if colour in indexes else []

    return (ConeArrayWithCovariance(blue_cones = cones("blue"),
                                   yellow_cones = cones("yellow"),
                                   orange_cones = cones("orange"),
                                   big_orange_cones = cones("big_orange"),
                                   unknown_color_cones = cones("unknown")),
                                   dict([(name, make_linestring(name)) for name in indexes]))

def visualise(planned_linestring, linestrings, filename, message, planned_point = None):
    good_dist_label, bad_dist_label = "fine distance!", "bad :("
    filename = re.search(".*\/(.*)\.csv\Z", filename)[1]
    plt.figure(figsize=(10, 10))

    if "blue" in linestrings: plt.plot(*linestrings["blue"].coords.xy, "x-", color = "blue", label = "blue cones")
    if "yellow" in linestrings: plt.plot(*linestrings["yellow"].coords.xy, "x-", color = "yellow", label = "yellow cones")
    if "midpoint" in linestrings: plt.plot(*linestrings["midpoint"].coords.xy, "x-", color = "red", label = "human drawn path")
    if "orange" in linestrings: plt.plot(*linestrings["orange"].coords.xy, "x", color = "orange", label = "orange cones")
    if "unknown" in linestrings: plt.plot(*linestrings["unknown"].coords.xy, "x", color = "grey", label = "unknown colour cones")

    if planned_linestring:
        plt.plot(*planned_linestring.coords.xy, "x-", color = "dodgerblue", label = "planned path")
        for midpoint in planned_linestring.coords:
            if "midpoint" not in linestrings: break
            midpoint = geom.Point(*midpoint)
            point_on_line = linestrings["midpoint"].interpolate(linestrings["midpoint"].project(midpoint))
            if midpoint.distance(linestrings["midpoint"]) <= ACCEPTABLE_DIST:
                plt.plot([midpoint.x, point_on_line.x], [midpoint.y, point_on_line.y], "-", color = "lime", label = good_dist_label)
                good_dist_label = ""
            else:
                plt.plot([midpoint.x, point_on_line.x], [midpoint.y, point_on_line.y], "-", color = "maroon", label = bad_dist_label)
                bad_dist_label = ""
    elif planned_point and "midpoint" in linestrings:
        midpoint = geom.Point(*planned_point)
        point_on_line = linestrings["midpoint"].interpolate(linestrings["midpoint"].project(midpoint))
        if midpoint.distance(linestrings["midpoint"]) <= ACCEPTABLE_DIST:
            plt.plot([midpoint.x, point_on_line.x], [midpoint.y, point_on_line.y], "-", color = "lime", label = good_dist_label)
        else:
            plt.plot([midpoint.x, point_on_line.x], [midpoint.y, point_on_line.y], "-", color = "maroon", label = bad_dist_label)

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    if "midpoint" in linestrings:
        human_length = linestrings["midpoint"].length
    else:
        human_length = 0.00000001
    if planned_linestring: planned_length = planned_linestring.length
    else: planned_length = 0
    plt.xlabel(f"human length: {round(human_length, 3)}\nplanned length: {round(planned_length, 3)}\n{round(100 * planned_length / human_length, 3)} %\n{message}")
    ax.legend()
    if not(os.path.isdir(vis_dir)): os.mkdir(vis_dir)
    print(f"{vis_dir}/{filename}.png")
    plt.savefig(f"{vis_dir}/{filename}.png")
    plt.close()

def test_csv(filename):
    DO_VIS = True
    to_return = None
    planned_point = None

    cone_array, linestrings = from_csv(filename)
    if "midpoint" not in linestrings:
        to_return = "No human-drawn centreline, can't test"
        DO_VIS = True

    mock = LocalPlanner("mock")
    path = mock.cones_callback(cone_array) # returns smooth_path
    mock.destroy_node()

    points = [*zip(path[0], path[1])]
    points = path
    if len(points) == 1:
        to_return = "only one point in planned path. linestring would shit itself."
        planned_point = points[0]

    planned_linestring = geom.LineString([*zip(path[0], path[1])])

    if not to_return and planned_linestring.intersects(linestrings["blue"]) or planned_linestring.intersects(linestrings["yellow"]):
        DO_VIS = True
        to_return = "collided with blue or yellow cones"

    if not to_return and abs(planned_linestring.length / linestrings["midpoint"].length - 1) > 0.1:
        DO_VIS = True
        to_return = "planned path was too short."

    for midpoint in planned_linestring.coords:
        midpoint = geom.Point(*midpoint)
        if not to_return and midpoint.distance(linestrings["midpoint"]) > ACCEPTABLE_DIST:
            DO_VIS = True
            to_return = "planned point was too far from human drawn midpoint"

    if DO_VIS:
        if to_return: message = to_return
        else: message = "LGTM"
        visualise(planned_linestring, linestrings, filename, message, planned_point = planned_point)
    
    return to_return


csv_files = os.listdir(csv_dir)
#to_test = ["a.csv", "curve_with_midpoint.csv", "it_lives.csv", "test.csv", "test2.csv", "bad_test.csv", "u_turn.csv", "spiralish.csv", "distance_troll.csv", "complex.csv",
#            "u_turn.csv"]
#to_test = ["peanut.csv", "hairpins_increasing_difficulty.csv", "comp_2021.csv", "bone.csv", "boa_constrictor.csv"]
to_test = ["comp_2021.csv", "bone.csv", "boa_constrictor.csv"]

#to_test = [x for x in to_test if x in csv_files]
to_test = [x for x in csv_files if x != "complex.csv"]
print(to_test)
class TestLocalPlanner(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        
    def tearDown(self):
        rclpy.shutdown()

    @parameterized.expand(to_test)
    def test_sequence(self, filename):
                self.assertIsNone(test_csv(f"{csv_dir}/{filename}"), f"{filename}: {fail_msg}")

if __name__ == "__main__":
    unittest.main()
