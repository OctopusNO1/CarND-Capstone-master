#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
WP_PUBLISH_RATE = 20
max_local_dist = 20.0
light_change_pub = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = []
        self.original_wpvel = []
        self.next_waypoint = None
        self.redlight_wp = None
        self.msg_seq = 0

        self.stop_on_red = rospy.get_param('~stop_on_red', True)      # Enable/disable stopping on red lights
        self.force_stop_on_last_waypoint = rospy.get_param('~force_stop_on_last_waypoint', True)   # Enable/disable stopping on last waypoint
        self.unsubscribe_base_wp = rospy.get_param('/unregister_base_waypoints', False)
        self.accel = rospy.get_param('~target_brake_accel', -1.)
        self.stop_distance = rospy.get_param('~stop_distance', 5.0)
        try:
            self.accel = max(rospy.get_param('/dbw_node/decel_limit') / 2, self.accel)
        except KeyError:
            pass

        # rospy.spin()
        rate = rospy.Rate(WP_PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.updatePublish()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
        pass

    def sameWP(self, wp1, wp2, max_d=0.5, max_v=0.5):

        # dist
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        # dist difference
        distdif = dl(wp1.pose.pose.position, wp2.pose.pose.position)

        if distdif < max_d:
           return True
        return False

    def next_wpUpdate(self):

        if not self.base_waypoints:
            return False

        if not self.current_pose:
            return False

        # Get car vars
        x_carpos = self.current_pose.position.x
        y_carpos = self.current_pose.position.y
        theta_carpos = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)


        # look for next_waypint 

        wp = None
        dist = 500000

        if self.next_waypoint:
            idx_offset = self.next_waypoint
            full_search = False
        else:
            idx_offset = 0
            full_search = True
        num_base_wp = len(self.base_waypoints)

        for i in range(num_base_wp):
            idx = (i + idx_offset)%(num_base_wp)
            wp_x = self.base_waypoints[idx].pose.pose.position.x
            wp_y = self.base_waypoints[idx].pose.pose.position.y
            wp_d = math.sqrt((x_carpos - wp_x)**2 + (y_carpos - wp_y)**2)

            if wp_d < dist:
                dist = wp_d
                wp = idx
            elif not full_search:
                if dist < max_local_dist:
                    # we found it
                    break;
                else:
                    full_search = True

        if wp is None:
            return False

        self.next_waypoint = wp
        return True
    
    def restore_velocities(self, indexes):
        for idx in indexes:
            self.set_waypoint_velocity(self.base_waypoints, idx, self.original_wpvel[idx])

    def deaccel(self, waypoints, stop_index, stop_distance):

        if stop_index <= 0:
            return

        dist = self.distance(waypoints, 0, stop_index)
        step = dist / stop_index
        vel = 0.
        d = 0.

        for idx in reversed(range(len(waypoints))):
            if idx < stop_index:
                d += step
                if d > self.stop_distance:
                    vel = math.sqrt(2*abs(self.accel)*(d-stop_distance))
            if vel < self.get_waypoint_velocity(waypoints[idx]):
                self.set_waypoint_velocity(waypoints, idx, vel)

    def msgPublish(self, final_waypoints):
            waypoint_msg = Lane()
            waypoint_msg.header.seq = self.msg_seq
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.header.frame_id = '/world'
            waypoint_msg.waypoints = final_waypoints
            self.final_waypoints_pub.publish(waypoint_msg)
            self.msg_seq += 1

    def updatePublish(self):

        if self.next_wpUpdate():

            num_base_wp = len(self.base_waypoints)
            last_base_wp = num_base_wp-1
            waypoint_idx = [idx % num_base_wp for idx in range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_idx]

            if self.stop_on_red:
                self.restore_velocities(waypoint_idx)
                try:
                    red_idx = waypoint_idx.index(self.redlight_wp)
                    self.deaccel(final_waypoints, red_idx, self.stop_distance)
                except ValueError:
                    red_idx = None
            if self.force_stop_on_last_waypoint or self.original_wpvel[-1] < 1e-5:
                try:
                    last_wp_idx = waypoint_idx.index(last_base_wp)
                    self.deaccel(final_waypoints, last_wp_idx, 0)
                except ValueError:
                    pass

            self.msgPublish(final_waypoints)

    def waypoints_cb(self, msg):

        waypoints = msg.waypoints
        num_wp = len(waypoints)
        if self.base_waypoints and self.next_waypoint is not None: 

            if not self.sameWP(self.base_waypoints[self.next_waypoint],
                                         waypoints[self.next_waypoint]):
                self.next_waypoint = None # We can't assume previous knowledge of waypoint
                self.base_waypoints = None
                rospy.logwarn("Base waypoint list changed")
        else:
            pass

        self.original_wpvel = [self.get_waypoint_velocity(waypoints[idx]) for idx in range(num_wp)]

        self.base_waypoints = waypoints

        if self.unsubscribe_base_wp:
            # unsubscribe
            self.base_wp_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        prev_red_light_waypoint = self.redlight_wp
        self.redlight_wp = msg.data if msg.data >= 0 else None
        
        if prev_red_light_waypoint != self.redlight_wp:
            if light_change_pub:
                self.updatePublish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
