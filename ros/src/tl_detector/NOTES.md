

## Subscribes to topic:

1. /base_waypoints: [] Complete list of waypoints for the course
2. /current_pos: Our Vehicle location
3. /image_color: image stream from the car camera. Used to determine color of upcoming traffic
4. /vehicle/traffic_lights: (x, y, z) coordinate of the traffic light


## Publishes to topic;
1. /traffic_waypoints: returns the idx of the waypoint for nearest red light stop line.


# Task:
1. Find the (x, y) coordinates to the nearest traffic lights visible.
2. Use camera image data to classify the color of the traffic light:
    -> Multi-head:
        1. Model head to find traffic light
        2. Model head to classify color.


Building the classifier:
1. download a free annotated dataset:
    -> https://hci.iwr.uni-heidelberg.de/content/bosch-small-traffic-lights-dataset
    -> How to download the data: https://hiro.bsd.uchicago.edu/node/3168
    -> clone the repo https://github.com/bosch-ros-pkg/bstld
        -> This repo has the annotation under /label_files