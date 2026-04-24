## Project Summary

This project was part of a robotics course at TU Berlin, where we worked in a team to build a ROS-based robot that can navigate a maze.

The robot operates on a grid and needs to figure out where it is, plan a path to a target position, and move there reliably. To do this, we combined several components: movement control, wall detection using laser data, localization within the maze, and path planning.

Over the course of the project, we started with simple movement (drive forward, turn 90°) and gradually improved it. One of the main challenges was that small movement errors quickly add up, so we worked on making the motion more stable and consistent. We also added logic to make the robot move more efficiently through corridors instead of stopping after every step.

In later stages, we implemented a graph-based representation of the maze and used it to plan paths automatically. The robot can take a target position, compute a path, and execute it, while continuously checking if it got lost and correcting itself if needed.

The project was very iterative. We often built simple versions first and then improved them step by step (e.g. adding better control, smoother movement, and more robust localization). Most of the work was done directly on the robot, so testing and debugging in the real environment played a big role.
