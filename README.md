# sgpr_ros
ROS package for a 3D SLAM loop closure pipeline. SGPR (Spectral Graph Place Recogntion) uses semantically segemented input point cloud scenes to extract object class features from each object present. The feature vector for each object is comprised of the eigenvalues which are computed through the following three steps

1. Convert each object point cloud to a graph representation through the novel Minimally Connected Adaptive Radius graph formulation algorithm
2. Create a Geometric Laplacian representation of the graph using inverse distance weights and curvature values
3. Compute the eigenvalues for the Geometric Laplacian through eigendecomposition

Objects between scenes are compared though a two sample Anderson-Darling test to determine the probability of the eigenvalue spectra match potential. Once all candidate matches are determined betwen reference and query scenes, a scene score is calculated to determine the place recogntion probability. If the probability score exceeds a specific user specified threshold, a place recognition event is recorded.

The full SGPR method is summariezed by the image below
![Fig1_final_1(1)](https://user-images.githubusercontent.com/45575958/231811520-7f5480e3-9d93-4d66-bad4-7b66da0cdf38.png)

##Place Recogntion Demo
This video demonstrates place recogntion using SGPR on the SemanticKitti Dataset. The green markers represent place recognition events when the driver revisits a previously visited strip of road.
<iframe src="https://player.vimeo.com/video/813694525?h=9e865c5615" width="560" height="315" frameborder="0" allowfullscreen></iframe>


<iframe src="https://player.vimeo.com/video/813694525?h=9e865c5615&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="1920" height="1080" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen title="semnaticKittiDemo"></iframe>

##Evaluation GUI Demo
The evaluation GUI allows a user to analyze various spectral graph algorithms when computing the feature vector. The GUI allows you to:

<iframe src="https://player.vimeo.com/video/808135753?h=2c3c58a134&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="1848" height="1024" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen title="eval_pipeline"></iframe>

1. Change datasets
2. Change filtering/sampling algorithms
3. Choose object point cloud sample sizes
4. Change graph formulation algorithms
5. Choose a Laplacian
6. Change the number of eigenvalues returned
7. Change keyframes being compared
8. Step through each object comparison
9. OpenGL visualization window which shows the object point clouds being compared
10. Histogram for the resulting spectral graph features
11. Evaluation window for candidate match prediction
12. Visualization for the 3D graph formulation
