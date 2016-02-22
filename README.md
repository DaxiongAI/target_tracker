This ROS package detects color blobs and transforms their position to the world reference frame (uses OpenCV)

There are two nodes to track the position of these targets:
   - One uses PCL with a K-NN search algorithm to cluster the samples and track the position of multiple targets (uses PCL)
   - The other one uses an extended kalman filter to estimate the positions (uses hector_object_tracker)
