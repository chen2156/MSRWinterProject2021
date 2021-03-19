<h1>Turtlebot3 frontier explorer with shape tracing</h1>

<h2>Frontier explorer on Turtlbot3</h2>
Before running thie code, make sure you have turtlebot3_simulation downloaded and installed in the src directory of the ROS workspace

Link to <a href="https://github.com/ROBOTIS-GIT/turtlebot3_simulations" target="_top">turtlebot3_simulations</a>

Go to the ROS workspace folder where the two packages are stored and run

`export TURTLEBOT3_MODEL=burger`
`roslaunch newturtlebot3 exploreMap.launch`

Depending on what gazebo file was used in the launch file, you will get a robot that starts exploring all frontiers on the map.  

<h3>Frontier Exploration Algorithm</h3>

1. Getting map data from the /map topic,this map is 1D list map value generated from slam_toolbox where 100 = occupied cell, 0 = unoccupied cell, -1 =unknown cell.  We want to find all cells that lie between unoccupied and unknown.  These are frontier cells

2. Build a 2D numpy array representation a grayscale image of the map 100 = occupied, 0 = unoccupied, unknown = -1

3. Replace unknown value(-1) with 255 and occupied value(100) with 128 such that only select high gradients between unoccupied and unknown are found when image Gradient is applied

4. Get image gradient of the map.  I wrote my own code where I applied Sobel filter on the map

5. Get map data from GlobalcostMap

6. Build a 2D numpy array representation of the globalcostmap

7. Find the cell position where the gradient is above a certain threshold and the globalcostmap is below a certain threshold.  I divided this into 2 seperate cases, The first cases handles subregions where the turtlebot3 is still exploring.  This is to prevent the turtlebot3 from getting stuck on the frontier edge, allowing it to still move and build the map.  The second case occurs once the turtlebot3 is done exploring a subregion, it moves onto the next subregion with the lowest cost

8. Convert the x and y position of the goal into the map frame and send it to move_base.  

9. Wait for the robot to finish the trajectory

10. Go back to the first step and keep doing this until the map is done being explored

<h2>In-between directions</h2>




