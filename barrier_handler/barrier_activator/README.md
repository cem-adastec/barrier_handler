# Barrier Activator

A node to enable dynamically switching between which barrier is being actively processed by barrier_filter node.

## Instructions
1. ```git clone https://github.com/adastec/barrier_handler.git```
2. Add package to your workspace and build.
3. Run [barrier_filter](https://github.com/adastec/barrier_handler/tree/master/barrier_filter) node as instructed.
4. Run: ```roslaunch barrier_activator barrier_activator.launch```
5. Switch between available barriers by sliding the slider on rgt_reconfigure. Integer value selected on the slider will correspond to the barrier ID inside barrier_config.yaml (see [barrier_selection](https://github.com/adastec/barrier_handler/tree/master/barrier_selection) node).

