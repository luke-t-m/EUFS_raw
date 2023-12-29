import os
import matplotlib.pyplot as plt
import shapely.geometry as geom

from planning_tests.test_track import TestTrack


colour_map = {"blue": "blue", "yellow": "yellow", "orange": "orange", "big_orange": "red", "unknown": "grey"}
def __get_colour_from_name(name):
    if name in colour_map:
        return colour_map[name]
    else:
        return "magenta"


def visualise(planned_path: geom.LineString, track: TestTrack, save_path, acceptable_dist=2, evaluation_results=(-1,-1,-1)):
    plt.figure(figsize=(10, 10))

    for cone_type in ["blue", "yellow", "orange", "big_orange", "unknown"]:
        if cone_type in track.track_df.index:
            plt.scatter(track.track_df.loc[cone_type]["x"],track.track_df.loc[cone_type]["y"], color=__get_colour_from_name(cone_type), label=f"{cone_type} cones")

    plt.plot(*track.target_middline().coords.xy, "--", color="black", label="target midline")

    plt.plot(*planned_path.coords.xy, "x-", color="dodgerblue", label="planned path")

    # draw coloured lines between planned path and target midline
    # TIDY THIS UP!!
    coords_along_planned_path = track.target_middline().segmentize(0.2).coords
    points_along_planned_path = [geom.Point(*coord) for coord in coords_along_planned_path]

    target_midline = planned_path

    good_dist_label, bad_dist_label = "fine distance!", "bad :("
    for point_on_planned_path in points_along_planned_path:
        point_on_target_midline = target_midline.interpolate(target_midline.project(point_on_planned_path))
        if point_on_planned_path.distance(target_midline) <= acceptable_dist:
                plt.plot([point_on_planned_path.x, point_on_target_midline.x], [point_on_planned_path.y, point_on_target_midline.y], "-", color = "lime", label = good_dist_label)
                good_dist_label = ""
        else:
            plt.plot([point_on_planned_path.x, point_on_target_midline.x], [point_on_planned_path.y, point_on_target_midline.y], "-", color = "maroon", label = bad_dist_label)
            bad_dist_label = ""

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.legend()

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    plt.xlabel(f"evaluation goes here: {evaluation_results}")

    plt.savefig(f"{save_path}/{track.track}.png")
    plt.close()
