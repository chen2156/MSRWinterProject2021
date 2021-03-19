<h1>Turtlebot3 frontier explorer with shape tracing</h1>

<h2>Frontier explorer on Turtlebot3</h2>

Before running thie code, make sure you have turtlebot3_simulation downloaded and installed in the src directory of the ROS workspace

Link to <a href="https://github.com/ROBOTIS-GIT/turtlebot3_simulations" target="_top">turtlebot3_simulations</a>

Go to the ROS workspace folder where the two packages are stored and run

`export TURTLEBOT3_MODEL=burger`

`roslaunch newturtlebot3 exploreMap.launch`

Depending on what gazebo file was used in the launch file, you will get a robot that starts autonomously exploring all frontiers in the world.  

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

Once you believe that the map is sufficently explored, you can call the map_server to save the map file.  In this repo, the map for the turtlebot3 house is saved in the newturtlebot3 directpry.  You can save the map by running 

`rosrun map_server map_saver -f <name of map file> `

<h2>Turtlebot3 shape tracing</h2>

To run the shape tracing part of the project, you can run

`roslaunch turltlebotDraw nav_stack.launch`

This will run both a gazebo and rviz simulation with whatever map you used to run frontier explorer.  Be sure to change line 11 in the launch file to the same world as the one used to run frontier explorer.  In my repo, I used turtlebot3 house to run the launch file, which is shown in the videos in the turltlebotDraw package.  Make sure to give turtlebot3 a 2D initial pose in rviz in order for the trajectory node to start running

<h3>Shape Tracing Algorithm</h3>

1. Getting map data from the /map topic,this map is 1D list map value generated from slam_toolbox where 100 = occupied cell, 0 = unoccupied cell.  We 

2. Upload an image of a shape as a grayscale image

3. Check if image can fit in the map and there are enough consecutive unoccupied cells to fit the image  The code will stop running if either condition is not met

4. Use Opencv's Canny Edge Detection to generate all edges of the shape  

5. Use opencv to figure out the location of the centroid of the image.

6. Loop through each pixel the edge image until you hit a pixel value greater than 150.  Add the pixel location to the points list and mark it as visited, and set it to the current pixel
  
	6a. From the current pixel, look at its surrounding 8 pixels, and choose the maximum pixel value that is greater than 150, which hasn't been visited
	
	6b. If there is exists a adjacent pixel that has a pixel value bigger than 150 and hasn't been visited, add the pixel location to the points list and mark it as visited, set that pixel to the current pixel and repeat step 6a.  Otherwise, move on to step 7
	
7. Sort all location in the points list based off the angle generated between between itself and the centroid of the image.  

8. Loop through the locations in the points list, transform into the map frame and send it to move_base.  

9. Wait for the robot to finish the trajectory

10. Keep looping through map coordinates until you reached the end









