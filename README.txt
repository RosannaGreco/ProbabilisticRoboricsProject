We provide a ready to use dataset. It is composed by the following files:

- map.dat: the position of all the landmarks in the world with their respective id.
	Every row contains:
  	LANDMARK_ID (1) POSITION (2:4)
- param.dat: information about the cameras used to gather data
	For each camera you have:
		- camera_id (Remember you have 3 cameras)
		- camera matrix
		- width/height of images
		- z_near/z_far how close/far the camera can perceive stuff
		- cam_transform: pose of the camera w.r.t. robot
- meas.dat: all measurements are divided into epoch.
	For every epoch you have several measurements, where each is composed as following:
		- camera_id (which camera produced this observation)
		- landmark_id (the id of the corresponding landmark. You SHOULD NOT use this except for debugging, to confirm your correspondences)
		- image_point (represents the pair [col;row] where the landmark is observed in the image)
- traj.dat: all ground-truth poses of the robot divided by epoch.
	For every epoch you have one pose, and it is composed as following:
		- epoch_num (1) robot_position tx ty tz (2-4) robot_orientation in quaternions qx qy qz qw (5-8)
	
=====================================================================================================================
What we expect you to do in Multi-PICP Localization?

1. position tracking (with Data Association)
   The robot moves in a continuous trajectory. implement an odometry-like schema that uses the 3 cameras and the known map (with known data association) to move from one epoch to the next

2. add to the previous mechanics a robust kernel and a data association policy to address unknown association case. You should get a trajectory reasonably close to the GT.
 
3. at this point it is time to address the global localization case: you can use a combination of RANSAC/8PTs on a single camera or something else smarter to initialize the 1st camera pose, then you go back to step 3.

TIP:

Initially using the trajectory, the camera parameters and the map, you can unproject world points to the image frame. Check that it returns the measurements in the measurements file,
   if this is true, your projection model is correct.
   
   Work in an icremental manner, where you need to finish a part of the code, check that it works(with the groud truth data), and then continue.
