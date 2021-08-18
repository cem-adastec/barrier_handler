# Barrier Filter

A node to filter and cluster lidar points inside a given volume. Specifically used for detecting barrier poles that are otherwise harder to detect.

## Prerequisites
In order for the barrier_filter node to function properly, a config file must be created. Therefore, first complete barrier_selection process to select and generate crop area for each barrier on the map. By default, barrier_config.yaml file will be saved under /home directory. For more details on barrier_selection process, follow [here](https://github.com/adastec/barrier_handler/tree/master/barrier_selection).

## Instructions
1. ```git clone https://github.com/adastec/barrier_handler.git```
2. Add package to your workspace and build.
3. Build the package.
4. ```roscore```
5. rosparam set use_sim_time true
6. Play a bag file which has a /points_raw topic.
7. Localize on the map.
8. Point of insterest information will be read from the barrier_config.yaml file which is by default saved to /home directory. Therefore, barrier_filter node will look for it under /home. But it is possible to change the path from terminal by passing its argument. (See Examples below).
9. ```roslaunch barrier_filter barrier_filter.launch```
10. Continue playing bag file to see results. /is_barrier_safe message must have at least 1 subscriber, otherwise the node won't do the processing.
11. On RViz, the clustered points, as well as the crop box in the form of a green (if barrier is open) or red (if barrier is closed) transparent marker can be seen by adding their respective topics.
12. /is_barrier_safe message should read False if a point cluster is detected, True otherwise.
13. If there are more than one barrier, then by running barrier_selection node, it is possible to switch between saved barriers. (See Examples below).

## Examples
* Launching as is: 
  ```roslaunch barrier_filter barrier_filter.launch```
* Launching with different path to search for config file:
  ```roslaunch barrier_filter barrier_filter.launch path_to_config:=/any/path/barrier_config.yaml```
* To select different barriers, run the following alongside the barrier filter node:
  ```roslaunch barrier_activator barrier_activator.launch```
* Sliding the slider between available barriers will switch currently active barrier.

![](images/barf.gif)
