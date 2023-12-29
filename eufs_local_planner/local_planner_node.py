# Python
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation

# ROS
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import shapely.geometry as geom
from shapely.ops import nearest_points
from sklearn.neighbors import KDTree




class LocalPlanner(Node):
    def __init__(self, name):
        super().__init__(name)

        #ALL COMMENTED OUT FOR TESTING WITH SIM, PLAYING BAGS BREAKS DECLARE_PARAMETER

        # The search radius to consider cones from
        self.search_radius = 5.9 #self.declare_parameter("search_radius", 5.9).value

        # The max distance between midpoints
        self.min_midpoint_distance = 0.25 #self.declare_parameter("min_midpoint_distance", 0.25).value
    
        # the max difference in angle between two consecutive cones
        self.max_angle_change = np.pi * 0.5 #self.declare_parameter("max_angle_change", np.pi * 0.5).value

        # the max angle change for uncoloured cones
        self.max_angle_change_uncoloured = np.pi * 0.1 #self.declare_parameter("max_angle_change_uncoloured", np.pi * 0.1).value

        # the distance the offset lines are made to the coloured cones (ignored for single lane)
        self.offset_distance = 2.0 #self.declare_parameter("offset_distance", 2.0).value

        # Cones topic
        cones_topic = "/cones/lenient" #self.declare_parameter("cones_topic", "/cones/lenient").value

        # PUBLISHERS
        # Main publisher for trajectory
        self.track_line_pub = self.create_publisher(WaypointArrayStamped, "output_path_topic", 1)

        # SUBSCRIBERS
        # Main subscriber for cones
        self.cones_sub = self.create_subscription(ConeArrayWithCovariance, cones_topic, self.cones_callback, 1)

        # Transform
        self.out_frame = self.declare_parameter("out_frame", "base_footprint").value
        self.tf_buffer = Buffer(rclpy.time.Duration(seconds=1))
        self.tf_listener = TransformListener(self.tf_buffer, self)


    # how many points needed for a line of a certain length based on the min midpoint distance
    def how_many_points(self, line_length):
        return max(2, int(line_length / self.min_midpoint_distance))

    def atan2b(self, xy1, xy2):
        return np.arctan2(xy1[0] - xy2[0], xy1[1] - xy2[1]) + np.pi

    # function to make lists of (x, y) from data
    def from_data(self, el_in_data):
        return [(cone.point.x, cone.point.y) for cone in el_in_data]

    # difference in two angles. sweep "direction" = which side of b the angle is on: 1 = right, -1 = left
    # eg: diff_angles(pi / 2, pi, -1) == 3 * pi / 2 ... diff_angles(pi / 2, pi, 1) == pi / 2
    def diff_angles(self, a, b, d):
        return (d * (b - a)) % (2 * np.pi)
    
    def make_tree(self, points):
        return KDTree(np.array(points), leaf_size = 2, metric = "euclidean")


    def cones_callback(self, data):
        self.cones_frame = data.header.frame_id
        self.cones_timetamp = data.header.stamp

        blues = self.from_data(data.blue_cones)
        yellows = self.from_data(data.yellow_cones)

        # group all cones that aren't blue or yellow
        uncoloured = self.from_data(data.orange_cones + data.big_orange_cones + data.unknown_color_cones)

        ordered_blues = self.order_points(blues, uncoloured, -1)
        uncoloured = [point for point in uncoloured if point not in ordered_blues]
        ordered_yellows = self.order_points(yellows, uncoloured, 1)

        #handle if we don't have enough points for lines
        if len(ordered_blues) < 2 and len(ordered_yellows) < 2:
            return None
        elif len(ordered_blues) < 2:
            return self.single_lane_planning(geom.LineString(ordered_yellows), 1)
        elif len(ordered_yellows) < 2:
            return self.single_lane_planning(geom.LineString(ordered_blues), -1)

        blue_line = geom.LineString(ordered_blues)
        yellow_line = geom.LineString(ordered_yellows)

        # handle if we don't have lines (could have snuck through checks above if eg. multiple points were the same)
        if yellow_line.length == 0 and blue_line.length == 0:
            return None
        elif blue_line.length == 0:
            path = self.single_lane_planning(yellow_line, 1)
        elif yellow_line.length == 0:
            path = self.single_lane_planning(blue_line, -1)
        else:
            path = self.double_lane_planning(blue_line, yellow_line)

        if path != None:
            tpath = np.array(path).T
            self.publish_path(tpath)
            return tpath


    def double_lane_planning(self, blue_line, yellow_line):
        right_of_blue = blue_line.offset_curve(-self.offset_distance, join_style = "round")
        left_of_yellow = yellow_line.offset_curve(self.offset_distance, join_style = "round")

        # docs said offset_curve can, very rarely return multilinestrings which we can't work with. Fall back to single lane planning.
        if type(right_of_blue) != geom.linestring.LineString and type(left_of_yellow) != geom.linestring.LineString:
            return None
        elif type(right_of_blue) != geom.linestring.LineString:
            return self.single_lane_planning(yellow_line, 1)
        elif type(left_of_yellow) != geom.linestring.LineString:
            return self.single_lane_planning(blue_line, -1)


        if right_of_blue.length < left_of_yellow.length:
            shorter, longer = right_of_blue, left_of_yellow
        else:
            shorter, longer = left_of_yellow, right_of_blue
        
        midpoints = []
        no_points = self.how_many_points(shorter.length)

        # generate that many midpoints by interpolating a fraction along the shorter line, finding the nearest point to that point on the longer line,
        # and adding their average
        for a in range(no_points):
            fraction_of_line = a / (no_points - 1)
            p1 = geom.Point(shorter.interpolate(fraction_of_line, normalized = True))
            p2 = nearest_points(p1, longer)[1]
            halfway = ((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
            midpoints.append(halfway)

        # add extra midpoints to extend the line back to the car and further forwards in a straight line
        penum, last = midpoints[-2:]
        xdiff = last[0] - penum[0]
        ydiff = last[1] - penum[1]
        mag = 10
        extended = last[0] + mag * xdiff, last[1] + mag * ydiff

        midpoints = [(-3, 0), (0, 0)] + midpoints + [extended]

        return self.interpolate_line(geom.LineString(midpoints))


    def order_points(self, coloured, uncoloured, sweep_direction):
        coloured_tree = self.make_tree(coloured)

        if len(uncoloured) != 0:
            uncoloured_tree = self.make_tree(uncoloured)

        #find start point (must be on the correct side)
        knn = coloured_tree.query([(3, 0)], k = min(5, len(coloured)), return_distance = False, sort_results = True)[0]
        for point in knn:
            first = coloured[point]
            if first[1] * sweep_direction < 0: break

        ordered = [(first[0] - 2, first[1]), first]
        seen = set()

        # "sweeps" to find closest point by angle. Imagine a clock hand moving round a point, and returning the first other point it hits
        while True:
            xyl, xy = ordered[-2:]
            # best_next = current candidate for next point
            # best_diff = the difference in the angles "to last" and the "to next" of best_next (both from "at")
            best_next = None
            best_diff = 2 * np.pi

            # find the yaw from the current point ("at") to the last point
            to_last = self.atan2b(xy, xyl)

            # loop over all points within search radius of current point
            for xyn_ind in coloured_tree.query_radius([xy], r = self.search_radius)[0]:
                xyn = coloured[xyn_ind]
                if xyn in seen:
                    continue

                # angle from current point to next point
                to_next = self.atan2b(xy, xyn)

                # difference in the angles "to last" and "to next" (both from "at")
                diff = self.diff_angles(to_last, to_next, sweep_direction)

                # if the difference is smaller than the max allowed angle change and it's the smallest diff
                if np.pi - self.max_angle_change < diff < np.pi + self.max_angle_change and (type(best_diff) == None or diff < best_diff):
                    best_diff = diff
                    best_next = xyn

            # if a sweep on coloured fails, attempt to find a viable uncoloured cone for the next cone
            if best_next == None and len(uncoloured) != 0:
                possibles = uncoloured_tree.query_radius([xy], r = self.search_radius)[0]
                for xyp_ind in possibles:
                    xyp = uncoloured[xyp_ind]
                    if xyp in seen:
                        continue
                    to_poss = self.atan2b(xy, xyp)
                    diff = self.diff_angles(to_last, to_poss, sweep_direction)
                    # if the difference is smaller than the max allowed angle change * and it's the smallest diff
                    # * - different for uncoloured cones, to avoid hitting uncoloured but actually opposite colour
                    if np.pi - self.max_angle_change_uncoloured < diff < np.pi + self.max_angle_change_uncoloured and (type(best_diff) == None or diff < best_diff):
                        best_diff = diff
                        best_next = xyp

            if best_next == None:
                break

            seen.add(best_next)
            ordered.append(best_next)

        return ordered[1:]
    
    #turn linestring into list of points
    def interpolate_line(self,line):
        midpoints = []
        no_points = self.how_many_points(line.length)

        # interpolate that many points at fractions along the line
        for a in range(no_points):
            p = geom.Point(line.interpolate(a / (no_points - 1), normalized = True))
            midpoints.append((p.x, p.y))
        return midpoints


    def single_lane_planning(self, line, side):
        # side: 1 for right lane, -1 for left lane

        # create line offset by 1.5 metres left or right of the line we have
        line = line.offset_curve(1.5 * side, join_style = "round")

        return self.interpolate_line(line)



    def transform_waypoints(self, waypoints):
        if self.out_frame == self.cones_frame:
            return waypoints.T, True

        try:
            trans = self.tf_buffer.lookup_transform(
                self.out_frame, self.cones_frame, rclpy.time.Time()).transform
        except TransformException as ex:
            self.get_logger().debug(f'{ex}')
            return waypoints.T, False

        R = Rotation.from_quat([trans.rotation.x, trans.rotation.y,
                               trans.rotation.z, trans.rotation.w])

        transformed = R.apply(
            np.vstack((waypoints, np.zeros(len(waypoints[0])))).T)
        transformed[:, 0] += trans.translation.x
        transformed[:, 1] += trans.translation.y
        return transformed[:, :2], True

    def publish_path(self, midpoints_array):
        way, transform_flag = self.transform_waypoints(midpoints_array)
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = self.out_frame if transform_flag else self.cones_frame
        waypoint_array.header.stamp = self.cones_timetamp

        for p in way:
            point = Point(x=p[0], y=p[1])
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner("local_planner")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
