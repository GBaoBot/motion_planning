# Motion Planning

This repository is used for implementing and testing motion planning algorithms. The package includes several algorithms such as PRM (Probabilistic Roadmap) and Bidirectional RRT (Rapidly-exploring Random Tree). Following OOP principles, the algorithms are implemented in the way that can be reusable and further developed in the future.

## Algorithms

- **PRM (Probabilistic Roadmap)**: A sampling-based algorithm that constructs a roadmap of the free space. It uses Dijkstra's algorithm to find the shortest path from the generated roadmap.
- **Bidirectional RRT (Rapidly-exploring Random Tree)**: An algorithm that grows two trees from the start and goal points and attempts to connect them.

## Side Notes

- **Testing Only**: This package is designed for testing algorithms only. The environment and map are included inside the package.
- **Not Suitable for Real Projects**: The algorithms in this package are not yet suitable for integration with real projects.
- **Future Work**: To make the package integrable and reusable as a separate library, the algorithms can be turned into ROS action servers. ROS topics or services can be used to enable communication within the system.

## Usage
To reconfigure the map as well as the algorithms, all you need to do is adjusting the value of parameters in YAML files inside `cfg` directory. For example, to select the algorithm, please navigate to cfg/map.yaml and change the value of `algorithm`, currently supporting `'prm'` and `'rrt'`.

To run the motion planning algorithms, use the provided launch files `launch.launch` and configure the parameters as needed. The launch files `launch.launch` load the necessary parameters and start the main node, to launch, please run this command line (after starting roscore):
```bash
roslaunch motion_planning launch.launch
```

## Configuration

The package includes several configuration files in the `cfg` directory:

- `map.yaml`: Configuration for the map.
- `post_processing.yaml`: Configuration for post-processing steps.
- `prm.yaml`: Configuration for the PRM algorithm.
- `rrt.yaml`: Configuration for the RRT algorithm.

In the `src` directory, the following files are included:
- `prm.py`: Implements the PRM algorithm, generating samples and graphs that can be used with other graph-based pathfinding algorithms such as BFS and Dijkstra's.
- `rrt.py`: Implements the Bidirectional RRT algorithm on the given map.
- `post_processing.py`: Applies post-processing techniques to the generated path, currently supporting path-shortcutting.
- `main.py`: Contains the main pipeline to process PRM or RRT with post-processing, reading configurations from the `cfg` directory.

The `utils` directory contains essential components for implementing and testing the algorithms, including environment setup, map handling, and search algorithms. 

## Results

Here are some results of the motion planning algorithms:

| PRM Algorithm | PRM Algorithm with Post-Processing |
|---------------------------------------|------------------------------------|
| ![PRM Result](src/motion_planning/docs/imgs/prm_no_shorcutting.png) | ![PRM Result](src/motion_planning/docs/imgs/prm_shorcutting.png) |

| Bidirectional RRT Algorithm | Bidirectional RRT Algorithm with Post-Processing |
|-----------------------------|--------------------------------------------------|
| ![RRT Result](src/motion_planning/docs/imgs/rrt_no_shortcutting.png) | ![RRT Result](src/motion_planning/docs/imgs/rrt_shortcutting.png) |

## Dependencies

- `numpy`
- `pylab`
- `rospy`

Make sure to install the required dependencies before running the package.