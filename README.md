# Jetbot automatic driving car project

## Introduction

We developed the software of Jetbot to make it a mini self-driving car. To be more specific, it should complete the following mission automatically by its front camera:

1. Driving along the right lane of the road
2. Detect the traffic light
3. Stop in front of the stop line
4. Turn left, go straight or turn right at the intersection
5. Avoid the pedestrian
6. Positioning and finding the shortest path

The map of our testing ground is shown below. Our Jetbot go from START to GOAL. The pedestrian appears randomly in the straight line. Once the Jetbot meet the pedestrian, it should avoid it.

![fig. 1](https://i.imgur.com/Ml0KB8q.png =80%x)


**Feel free to check out our testing video**  
https://youtu.be/88C9GjCfox8



## Code structure

To complete the mission mentioned above, we design 4 state: **lane following**, **intersection**, **crossing intersection** and **obstacle avoidance**. The condtions to jump to each state are shown in the flow digram below. 

![](https://i.imgur.com/dTYePxP.png)

The mission in each state is described as following:

1. **Lane following**:  
(1) Follow the right lane of the road  
(2) Do stop line detection and human detection
2. **Intersection**:  
(1) Look at the traffic light  
(2) Look at the aruco maker to get the current position  
(3) Path finding
3. **Crossing intersection**  
Go straight / turning left / turn right (depends on the path finding result)
4. **Obstacle avoidance**  
Avoiding colliding with the pedestrian




<!-- 
### Lane following

Related codes: `LaneFollower.py`, `Controller.py` and `ColorDetector.py`

![](https://i.imgur.com/fmE62P1.png =100%x)

### Traffic light and stop line detection

Related codes: `StopLineDetector.py`, `ColorDetector.py`, `HSV_and_Area_Slider.py`

### Passing the intersection -->







