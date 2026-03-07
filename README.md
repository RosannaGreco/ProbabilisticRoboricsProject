# Multi-PICP localization Project
# Abstract
The aim of the project was to estimate the position of a multi camera system provided with three cameras observing a set of known 3D point landmarks. 

# Methodology
## First Step: Position Tracking with Known Data Association
In the first step, a Projective ICP schema was implemented in order to address the problem considering the known data association case.


The provided data included: 
- map.dat, containing the position of the landmarks in the world reference frame.
- param.dat, containing information about the cameras mounted on the robot.
- meas.dat, containing the 2D coordinates measurements captured by the cameras during each epoch, plus some additional information:
    - camera_id
    - landmark_id 
- traj.dat, the ground truth of the trajectory. 

After retrieving the position of the landmarks and the camera parameters, the problem was addressed following the Projective ICP schema presented during the course.

The implementation of this part is in the `icp.m` file.
### ICP
The ICP algorithm is and application of Least Squares used to compute the state of the robot starting from a set of points in the world frame and a set of observations seen by the robot (in this case, in the frame of the three cameras).

 This is done by estimating the pose that minimizes the distance between corresponding points (in particular, we calculate the pose of the world wrt the robot).

#### Problem definition
##### State and Boxplus operator
The state was provided in the form of translation and an orientation expressed in quaternions.
$$
x = [t q] = \begin{pmatrix}
tx & ty & tz & qx & qy & qz & qw
\end{pmatrix}
$$
It can be expressed as a transformation matrix.
$$
X \in SE(3):  X = (R|t)
$$
We can hence define a perturbation in the euclidean space $ \Delta x \in \mathbb{R}^6 $ and the boxplus operator:

$$
X \boxplus \Delta x : v2t(\Delta x)X
$$


The 7d vector used as initial guess for ICP is expressed as the pose of the world wrt robot and converted in a transformation matrix $X$ by a function called `v2t_quaternion`. Subsequently, the state is updated applying the boxplus operation. 

A function named `t2v_quaternion` converts the estimated state $X_result$ in a 7d vector format, which is then expressed as robot wrt world, enabling a comparison with the ground truth pose. 

All the functions used to work with the 7d vector representation (i.e. to handle quaternions) are stored in a file called `quaternions_helper.m`. 

##### Measurements and Prediction
The measurements are the 2D coordinates of the points perceived by the cameras, and can hence be condisered Euclidean. 
$$ 
z \in \mathbb{R}^2
$$
We don't need to define a boxminus operator. 
The prediction is given by
$$
h^{[n]}(X) = proj(KT^{-1}X p_{world})
$$

##### Projection model details
To project a point, we apply the following transformations:
- bring the point in the robot frame using the estimated transformation X (expressed as world wrt robot). The first tests were performed considering the gt of the first pose.
- bring the point in the camera frame using the camera transformation matrix T provided in `param.dat`. 
- compute the 2d coordinated of the point applying the camera matrix $K$ and projecting 

$p_{img} = proj(KT^{-1}X p_{world})$, with $proj(p) = ( \frac{x}{z} , \frac{y}{z} )$

During this first step (considering known data association), for each measurement, it was possible to access the landmark id of the point. So each world point was projected according to the parameters from which the measurement came from.


##### Error and Jacobian
The error can be defined as the difference between prediction and measurement.
$$
e^{[n,m]}(X) = h^{[n]}(X) - z^{[m]}
$$
The Jacobian can be computed as follows: 
$$
J = J_{proj}(\hat{p}_{cam}^{[n]})KJ_{icp}
$$
$$
Jproj = \begin{pmatrix}
\frac{1}{z} & 0 &  -\frac{x}{z^2}\\
0 & \frac{1}{z} &  -\frac{y}{z^2}
\end{pmatrix}
$$

$$
J_{icp} = \begin{bmatrix} J_t & J_r \end{bmatrix} \quad
J_t = R_{camera}^T, \quad
J_r = - R_{camera}^T \, \lfloor p_{robotframe} \rfloor_\times
$$

Where $\hat{p}_{cam}^{[n]} $ is the point in the camera frame after the application of matrix $K$, $p_{robotframe} $ is the point expressed in the robot frame and $R_{camera}$ is the rotation matrix extracted from the transformation matrix of the camera.
The `errorAndJacobian` function computes the error and the Jacobian according to the parameters of the camera the measurement is perceived from. 


#### Algorithm
The core ICP algorithm in the case of the manifold is described by the following schema: 

<img src="images/icp.png" alt="ICP Algorithm" width="400"/>




In the proposed implementation, for each epoch, the algorithm takes the result computed by ICP for the previous epoch as initial guess. The obtained result is compared to the ground truth.

## Second Step: Robust Kernel and Data Association Policy
In the second step, a robust kernel was added to the ICP algorithm in order to lessen the contribution of outliers. Furthermore, the unknown data association case was addressed. 

### Data Association 
This part is handled in the `dataAssociation.m` file. 

Firstly, all the landmarks are projected in the cameras according to $X$, forming three matrices (one for each camera). Then, for each measurement, the association matrix is computed taking in account the matrix obtained considering the camera from which the measurement was perceived. 

A gating strategy is used to compute the associations, considering the projection error. In our case, the same landmark can be observed by two cameras in the same epoch (and appear in a different position in each of them, because the cameras are mounted on the robot with different positions and orientations), so we cannot assume that one landmark should be paired with just one measurement. 


### Third Step: Addressing the Global Localization Case with RANSAC
RANSAC (Random Sample Consensus) is a methodology used for 3D point registration in order to estimate the pose when the correspondences and the initial guess are unknown. 

It can be summed up as follows: 
- Firstly, a minimal set of correcpondences is sampled from the candidate ones
- Then an alignment is computed
- Said alignment is used to determine the number of good/bad correspondences
- The 'consensus' of the guess is computed as a function of the number of inliers and the error
- The whole procedure is repeated N times, and at each time the best solution is kept
- Then the inliers are used to refine the solution
So, in the last step of the project, the global localization case was addressed in order to compute an initial guess to be used by ICP. 
The code of this part is in the `Ransac.m` file.

In the proposed implementation, the global localization case is addressed running RANSAC on one of the cameras (camera 0). The pose used for the first initialization is the identity. 

At each iteration: 
- Candidate correspondences are built projecting the points according to the best estimation found (during the first iteration, the identity) and handling data association with the same strategy used for the previous step, but with a less restrictive value for gating tau. 
- 10 correspondences are selected randomly among the candidates
- Said correspondences are used to compute a new pose with ICP (in this case, we call a different function (`doIcpRANSAC`), which doesn't handle the data association part in the loop). The best soultion found is used as initial guess
- The landmarks are projected in the new pose estimated by ICP
- The number of inliers is computed according to the chosen gating tau, which works as a threshold. We also calculate the cost summing the distance values.
- If the result is better in terms of inliers and cost, or if the number of inliers is the same, but we have a lower cost, R_best and t_best are updated


Finally, the best solution is refined using all the inliers found. 

# Results and Plots
The system computes an accurate estimate of the whole trajectory, with a trajectory error in the order of $10^{-5}$. 
## Trajectory Plots
The images below shows a comparison between the estimated trajectory (red) and the ground truth trajectory (blue). A file containing all the estimated poses (`poses.dat`)  is contained in the output folder. 

<p align="center">
    <img src="output/gt_trajectory.png" alt="Trajectory Plot gt" width="300"/>
    <img src="output/estimated_trajectory.png" alt="Trajectory Plot est" width="300"/>
    <img src="output/trajectory.png" alt="Trajectory Plot" width="400"/>
    
    
</p>


## ICP error evolution
The below plots were obtained running ICP in order to estimate the pose of the first epoch starting from different initial guesses. 

The first two images show the error obtained with a good initial guess and a bad initial guess using a kernel threshold = 5. In this case, the system reaches a good estimation only if the pose chosen as initial guess is close enough. 
<p align="center">
    <img src="output/icp_error_good_initial_guess.png" alt="icp_error_good_init_guess" width="400"/>
    <img src="output/icp_error_bad_initial_guess.png" alt="icp_error_bad_init_guess" width="400"/>
    
</p>

These, instead, are the results obtained running the same test with a kernel threshold = 20. As we can see, using a robust kernel helps handling the case in which we have a greater number of outliers. 
<p align="center">
    <img src="output/icp_error_good_initial_guess_kt.png" alt="icp_error_good_init_guess_kt" width="400"/>
    <img src="output/icp_error_bad_initial_guess_kt.png" alt="icp_error_bad_init_guess" width="400"/>
    
</p>

## Trajectory error evolution 
The table below shows the trajectory error computed during the first epochs. As we can see, the values remain on the order of $10^{-5}$. The output folder contains a file (`error.dat`) storing the error values during the whole trajectory.

| Epoch | tx | ty | tz | qx | qy | qz | qw |
|------|------|------|------|------|------|------|------|
| 0 (ransac) | 0.000016| -0.000009| -0.000008 |0.000044| 0.000049| -0.000045 |0.000000|
| 1 |0.000013 |0.000003 |0.000019 |0.000034| 0.000025| 0.000004| 0.000000|
| 2 | 0.000009| 0.000003| 0.000009| -0.000008| 0.000012| 0.000001 |0.000001|
| 3 | 0.000006| 0.000000| 0.000011 |0.000035 |-0.000016| -0.000022| 0.000004|
| 4 |  0.000002 |-0.000014| 0.000015| 0.000008| -0.000022| -0.000048| 0.000007|
| 5 | 0.000010| -0.000037 |0.000006| 0.000015| 0.000035| 0.000048| 0.000013|
|6  | 0.000001| 0.000028 |0.000008 |0.000023| -0.000002| -0.000005| 0.000020|
|7 | 0.000005 |-0.000025| -0.000003 |0.000010 |-0.000022 |0.000041 |0.000021|
|8 | 0.000005| 0.000019| -0.000010 |0.000000 |0.000015| -0.000044| 0.000027|
|9 | 0.000005| 0.000047| -0.000013| 0.000038| -0.000045| -0.000045| 0.000032|
|10 | 0.000004| -0.000040| -0.000021| -0.000038| -0.000009 |0.000010| 0.000037|
|11 | 0.000007| -0.000041| -0.000029| 0.000011| 0.000020 |-0.000022| 0.000044|

Here we can see some plots displaying the behavior of the single components of the error during the whole trajectory.
<p align="center">
    <img src="output/x_error_evolution.png" alt="x_error_evolution" width="400"/>
    <img src="output/y_error_evolution.png" alt="y_error_evolution" width="400"/>
    <img src="output/z_error_evolution.png" alt="z_error_evolution" width="400"/>
    <img src="output/qx_error_evolution.png" alt="qx_error_evolution" width="400"/>
    <img src="output/qy_error_evolution.png" alt="qy_error_evolution" width="400"/>
    <img src="output/qz_error_evolution.png" alt="qz_error_evolution" width="400"/>
    <img src="output/qw_error_evolution.png" alt="qw_error_evolution" width="400"/>
</p>