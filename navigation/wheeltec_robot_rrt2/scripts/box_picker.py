import cv2
import os
import sys
import yaml
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult
from rclpy.duration import Duration
import rclpy

def main():
    # read map image
    image = cv2.imread(os.path.join(sys.argv[1], 'map.pgm'))

    # mirror map for matching with costmap convention
    mirror_image = cv2.flip(image, 0)

    # convert map to grayscale
    img_gray = cv2.cvtColor(mirror_image, cv2.COLOR_BGR2GRAY)

    # convert map to binary
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)
    cv2.imshow('Binary image', thresh)

    # find all contours
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

    # read map yaml for retrieving map origin position and resolution
    with open(os.path.join(sys.argv[1], 'map.yaml'), "r") as stream:
        try:
            map_yaml = yaml.safe_load(stream)
            print(map_yaml['resolution'])
        except yaml.YAMLError as exc:
            print(exc)

    # calculate list of waypoints(box locations) that the bot should follow
    way_points =  []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 40.0 and area > 30.0:
            M = cv2.moments(contour)
            mx = M["m10"] / M["m00"]
            my = M["m01"] / M["m00"]
            gx = map_yaml['origin'][0] + (mx + 0.5) * map_yaml['resolution']
            gy = map_yaml['origin'][1] + (my + 0.5) * map_yaml['resolution']
            way_points.append((gx,gy))

    print(way_points)

    # initialise navigator
    rclpy.init()
    navigator = BasicNavigator()
    goal_poses = []


    # convert all way_points to stamped pose messages
    for way_point in way_points:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = way_point[0]
        goal_pose.pose.position.y = way_point[1]
        goal_pose.pose.orientation.w = 0.707
        goal_pose.pose.orientation.z = 0.707
        goal_poses.append(goal_pose)

    # send goal_poses to waypoint follower
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    # Print feedback
    i = 0
    while not navigator.isNavComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelNav()


    # Display result
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

        exit(0)

if __name__ == '__main__':
    main()