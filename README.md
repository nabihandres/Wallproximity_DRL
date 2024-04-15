
# Wall proximity 

### Overview
This repository introduces neural network results designed for autonomous navigation, which include features
data about the robot, humans, static obstacles, and path constraints.
The README outlines the test environments: curvepath and crosspath.
The Reward functions and the introduction of new subfunctions are introduced to better handle robot navigation concerning wall proximity.

## Reward Functions
- **R1**:

$$
R(J_t, a_t) = 
\begin{cases} 
R_g=10 & \text{if Reach the goal} \\
R_c=-0.5 & \text{elif Collision} \\
R_d = 0.5(d_{i} - d_{\text{und}}) & \text{elif Danger} \\
R_{hg}= 0.01(d_{g}(t - 1) - d_{g}(t)) & \text{otherwise}
\end{cases}
$$

- **R2**:

$$
R(J, a) = R_g + R_c + R_d + R_{hg} + R_s
$$

- **R3(Ours)**:

$$
R(J, a) = R_g + R_c + R_d + R_{hg} + R_s + R_{pf} + R_{wall}
$$

## Example of How Wall Proximity R3(Ours) Subfunctions Work
(wall_proximity.gif)
![robot_movement](wall_proximity.gif)


## Simulation results on Curvepath Environment
### Dynamic Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R1_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R2_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R2_LSTM.gif.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (19.50s)</td>
    <td>R2(CADRL) Collision</td>
    <td>R1(LSTM) Collision</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Collision  with wall</td>
    <td>R1(SARL) Collision with wall</td>
    <td>R2(SARL) Collision </td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/dyn_R3(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (16.00s)</td>
    <td></td>
  </tr>
</table>

### Mixed Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R1_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R2_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (19.25s)</td>
    <td>R2(CADRL) Collision</td>
    <td>R1(LSTM) Time out</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R2_LSTM.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Collision </td>
    <td>R1(SARL) Collision</td>
    <td>R2(SARL) Collision with wall</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/mixed_R3(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (20.00s)</td>
    <td></td>
  </tr>
</table>

### No Obstacles

<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R1_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R2_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (14.75s)</td>
    <td>R2(CADRL) Reach the goal (15.25s)</td>
    <td>R1(LSTM) Reach the goal (14.25s)</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Reach the goal (14.25s)</td>
    <td>R1(SARL) Reach the goal (13.75s)</td>
    <td>R2(SARL) Reach the goal (13.75s)</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/no_R3(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (13.75s)</td>
    <td></td>
  </tr>
</table>

### Static Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R1_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R2_CADRL.gif.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (16.25s)</td>
    <td>R2(CADRL) Reach the goal (14.50s)</td>
    <td>R1(LSTM) Reach the goal (14.50s)</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Reach the goal (14.50s) </td>
    <td>R1(SARL) Time out</td>
    <td>R2(SARL) Time out</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/curvepath/static_R3(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (14.50s)</td>
    <td></td>
  </tr>
</table>

## Simulation results on Crosspath Environment
### Dynamic Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R1_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R2_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Collision</td>
    <td>R2(CADRL) Collision with wall</td>
    <td>R1(LSTM) Collision</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R2_SARL(R2).gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Collision</td>
    <td>R1(SARL) Collision</td>
    <td>R2(SARL) Reach the goal (17.50s) </td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/dyn_R3_(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (14.00s)</td>
    <td></td>
  </tr>
</table>

### Mixed Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R1_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R2_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Collision</td>
    <td>R2(CADRL) Collision</td>
    <td>R1(LSTM) Collision with wall</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Collision </td>
    <td>R1(SARL) Collision</td>
    <td>R2(SARL)  Reach the goal (20.75s)</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/mixed_R3(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (19.5s)</td>
    <td></td>
  </tr>
</table>

### No Obstacles

<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R1_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R2_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (10s)</td>
    <td>R2(CADRL) Reach the goal (9.75s)</td>
    <td>R1(LSTM) Reach the goal (14.75s)</td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) Reach the goal (10s)</td>
    <td>R1(SARL) Reach the goal (9.75s)</td>
    <td>R2(SARL) Reach the goal (9.5s)</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/no_R3_(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (9.75s)</td>
    <td></td>
  </tr>
</table>

### Static Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R1_CADRL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R2_cadrl.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R1_LSTM.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R1(CADRL) Reach the goal (12s)</td>
    <td>R2(CADRL) Reach the goal (10.50s)</td>
    <td>R1(LSTM) Time out </td>
  </tr>
  <tr>
     <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R2_LSTM.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R1_SARL.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R2_SARL.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>R2(LSTM) collision </td>
    <td>R1(SARL) Time out</td>
    <td>R2(SARL) Reach the goal (14.00s)</td>
  </tr>
  <tr>
    <td></td>
    <td class="center-text"><img src="https://github.com/nabihandres/RAL/blob/main/crosspath/static_R3_(ours).gif" alt="Alt Text 1" width="300"/></td>
    <td></td>
  </tr>
  <tr> 
    <td></td>
    <td>R3(OURS) Reach the goal (10.50s)</td>
    <td></td>
  </tr>
</table>

## Experimental results

### Scenario 1: Limited space without obstacles (Distance to goal: 7 meters)

https://github.com/nabihandres/Wallproximity_DRL/assets/44639920/b6355036-4b33-40c2-b926-22bc5e2f4680

https://github.com/nabihandres/Wallproximity_DRL/assets/44639920/60364009-8fb7-4235-ae28-d0dea7e77356
### Scenario 2: Dynamic environment (Path constraints, 5 Humans, and 1 Robot as static obstacle) (Distance to goal: 12 meters)

https://github.com/nabihandres/Wallproximity_DRL/assets/44639920/cfe2cfc4-a539-4fa4-8e80-377e495d0063

https://github.com/nabihandres/Wallproximity_DRL/assets/44639920/9f1db444-588e-4e51-b90f-f94854dfd989
### Scenario 3: Dynamic environment ( 7 Humans) (Distance to goal: 21 meters)

https://github.com/nabihandres/Wallproximity_DRL/assets/44639920/f16fa629-1633-4021-ac75-5027f9a0f30a



## Wall Proximity Subfunctions
The Wall Proximity Subfunctions aim to ensure that the robot maintains a safe distance from walls. The robot is penalized for being too close or too far from a wall and rewarded for maintaining an optimal distance.
Environment (crosspath and blockpath) has blocks and each block can be divided to walls or line segments.
The subfunctions include:

#### `compute_min_distance_to_block(self, robot_position, wall_areas)`
Computes the minimum distance from the robot to the nearest block (wall area).
In this function, we iterate through all blocks in the environment and call *compute_distance_to_wall(robot_position, wall_area)* function for each wall area.
- `robot_position`: Current position of the robot.
- `wall_areas`: List of wall area coordinates.
  ```python
  def compute_min_distance_to_block(self, robot_position, wall_areas):
      min_distance = float('inf')
      max_distance = 0

      for wall_area in wall_areas:
          distance_to_wall, point_on_line = self.compute_distance_to_wall(robot_position, wall_area)
          if distance_to_wall < min_distance:
              min_distance = distance_to_wall
              closest_point = point_on_line

      return min_distance, closest_point

#### `compute_distance_to_wall(self, robot_position, wall_area)`
Calculates the distance from the robot to a specified wall area.
In this function, we iterate though all walls or line segments that the block has and call *point_to_line_segment_distance(x, y, x1, y1, x2, y2)* function.
- `robot_position`: Current position of the robot.
- `wall_area`: Coordinates of a specific wall area.
  ```python
  def compute_distance_to_wall(self, robot_position, wall_area):
	    x, y = robot_position
	    distances = {}
	    distances_list = []

	    for i in range(len(wall_area)):
	        x1, y1 = wall_area[i]
	        x2, y2 = wall_area[(i + 1) % len(wall_area)]  
	        distance, point = self.point_to_line_segment_distance(x, y, x1, y1, x2, y2)
	        distances[distance] = point
	        distances_list.append(distance)

	    min_dist = min(distances_list)
	    min_point = distances[min_dist]

	    return min_dist, min_point

#### `point_to_line_segment_distance(self, x, y, x1, y1, x2, y2)`
Calculates the shortest distance from a point to a line segment.
In this function, we are given coordinates of the wall (line segment) and we calculate distance from a robot to that line segment.
- `(x, y)`: Point coordinates of the robot.
- `(x1, y1)`, `(x2, y2)`: Endpoints of the line segment.
  ```python
  def point_to_line_segment_distance(self, x, y, x1, y1, x2, y2):
      length_squared = (x2 - x1)**2 + (y2 - y1)**2
      if length_squared == 0:
          return math.sqrt((x - x1)**2 + (y - y1)**2)

      t = max(0, min(1, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / length_squared))
      px = x1 + t * (x2 - x1)
      py = y1 + t * (y2 - y1)
      distance = math.sqrt((x - px)**2 + (y - py)**2)

      return distance, [px, py]

#### `farthest_point_and_distance(self, robot_position, wall_areas)`
Finds the farthest point and distance from the robot to the wall areas.
- `robot_position`: Current position of the robot.
- `wall_areas`: List of wall area coordinates.
  ```python
  def farthest_point_and_distance(self, robot_position, wall_areas):
      farthest_point = None
      max_distance = float('-inf')

      for area in wall_areas:
          for vertex in area:
              dist = self.distance(robot_position, vertex)
              if dist > max_distance:
                  max_distance = dist
                  farthest_point = vertex

      return farthest_point, max_distance

#### `distance(self, point1, point2)`
Calculates the Euclidean distance between two points.
- `point1`, `point2`: Coordinates of the two points.
  ```python
  def distance(self, point1, point2):
      return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


### Reward/Penalty Mechanism
The robot's behavior is influenced by the following reward/penalty mechanism:

#### `R_wall_min_dist`
- If the robot is too close to a wall (`< 0.01` m), it receives a penalty of `-0.5`.
- Otherwise, it gets a small reward of `0.005`.
  ```python
  min_distance, closest_point = self.compute_min_distance_to_block(self.robot.get_position(), self.blocks)
  min_distance -= self.robot.radius
  R_wall_min_dist = 0
  min_wall_bool = False
  if min_distance < 0.01:
      R_wall_min_dist = -0.5
      min_wall_bool = True
  else:
      R_wall_min_dist = 0.005
  
#### `R_wall_max_dist`
- If the robot is too far from a wall (`> 20` m), it receives a penalty of `-0.5`.
  ```python
  farthest_point, max_distance = self.farthest_point_and_distance(self.robot.get_position(), self.blocks)
  R_wall_max_dist = 0
  if max_distance > 20:	
      R_wall_max_dist = -0.5



  




