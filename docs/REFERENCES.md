# References

A curated reading list for Kestrel SLAM. The theory chapters in [`theory/`](theory/README.md) cite
entries by their **[Key]**. Each entry has a one-line note on *why* it is worth reading.

Identifiers (arXiv numbers) are given only where we are certain of them. Otherwise, search by
title.

---

## Textbooks and surveys

* **[Barfoot17]** T. D. Barfoot. *State Estimation for Robotics.* Cambridge University Press, 2017.
  — The clearest treatment of estimation on SO(3)/SE(3), from Gaussians to batch optimization. Free author PDF.
* **[Hartley04]** R. Hartley, A. Zisserman. *Multiple View Geometry in Computer Vision*, 2nd ed.
  Cambridge University Press, 2004. — The reference for projective geometry, epipolar geometry, triangulation and RANSAC.
* **[Thrun05]** S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics.* MIT Press, 2005.
  — Occupancy grids, inverse sensor models and the Bayesian view of mapping.
* **[Nocedal06]** J. Nocedal, S. J. Wright. *Numerical Optimization*, 2nd ed. Springer, 2006.
  — Gauss–Newton, Levenberg–Marquardt and trust regions done properly.
* **[Dellaert17]** F. Dellaert, M. Kaess. "Factor Graphs for Robot Perception." *Foundations and
  Trends in Robotics* 6(1–2), 2017. — Why SLAM is a sparse factor graph and how to exploit that.
* **[Cadena16]** C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid,
  J. J. Leonard. "Past, Present, and Future of Simultaneous Localization and Mapping: Toward the
  Robust-Perception Age." *IEEE Transactions on Robotics* 32(6), 2016. — The field's map; includes open problems in long-term and dynamic SLAM.
* **[Grisetti10]** G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard. "A Tutorial on Graph-Based
  SLAM." *IEEE Intelligent Transportation Systems Magazine*, 2010. — Gentle introduction to pose graphs.
* **[Woodman07]** O. J. Woodman. "An introduction to inertial navigation." Technical Report
  UCAM-CL-TR-696, University of Cambridge, 2007. — Short, practical intro to IMU error growth.

## Lie groups and rotations

* **[Sola18]** J. Solà, J. Deray, D. Atchuthan. "A micro Lie theory for state estimation in
  robotics." arXiv:1812.01537, 2018. — Best first read on Exp/Log, Jacobians and ⊞/⊟. Has a cheat sheet.
* **[Sola17q]** J. Solà. "Quaternion kinematics for the error-state Kalman filter."
  arXiv:1711.02508, 2017. — Hamilton vs. JPL conventions and quaternion calculus, explained with care.
* **[Trawny05]** N. Trawny, S. I. Roumeliotis. "Indirect Kalman Filter for 3D Attitude
  Estimation." Technical Report 2005-002, University of Minnesota, 2005. — The JPL-convention reference used by much of the MSCKF literature.

## Cameras, geometry and calibration

* **[Brown66]** D. C. Brown. "Decentering distortion of lenses." *Photogrammetric Engineering*,
  1966. — Origin of the radial–tangential ("radtan") distortion model.
* **[Kannala06]** J. Kannala, S. S. Brandt. "A generic camera model and calibration method for
  conventional, wide-angle, and fish-eye lenses." *IEEE TPAMI* 28(8), 2006. — The equidistant / fisheye model.
* **[HartleySturm97]** R. Hartley, P. Sturm. "Triangulation." *Computer Vision and Image
  Understanding* 68(2), 1997. — Optimal two-view triangulation and why the linear method is only an initializer.
* **[Furgale13]** P. Furgale, J. Rehder, R. Siegwart. "Unified temporal and spatial calibration
  for multi-sensor systems." *IROS*, 2013. — The method behind Kalibr; joint camera–IMU extrinsics and time offset.

## Features, tracking and robust estimation

* **[Lucas81]** B. D. Lucas, T. Kanade. "An iterative image registration technique with an
  application to stereo vision." *IJCAI*, 1981. — The original Lucas–Kanade tracker.
* **[Shi94]** J. Shi, C. Tomasi. "Good Features to Track." *CVPR*, 1994. — Min-eigenvalue corners and why they make the tracker well-conditioned.
* **[Bouguet00]** J.-Y. Bouguet. "Pyramidal Implementation of the Lucas Kanade Feature Tracker:
  Description of the Algorithm." Technical report, Intel Corporation, Microprocessor Research Labs
  (distributed with OpenCV), c. 2000. — Exactly what `cv::calcOpticalFlowPyrLK` does.
* **[Kalal10]** Z. Kalal, K. Mikolajczyk, J. Matas. "Forward-Backward Error: Automatic Detection of
  Tracking Failures." *ICPR*, 2010. — The forward–backward check used by Kestrel's front-end.
* **[Rosten06]** E. Rosten, T. Drummond. "Machine learning for high-speed corner detection."
  *ECCV*, 2006. — FAST corners (used inside ORB).
* **[Calonder10]** M. Calonder, V. Lepetit, C. Strecha, P. Fua. "BRIEF: Binary Robust Independent
  Elementary Features." *ECCV*, 2010. — Binary descriptors compared by Hamming distance.
* **[Rublee11]** E. Rublee, V. Rabaud, K. Konolige, G. Bradski. "ORB: An efficient alternative to
  SIFT or SURF." *ICCV*, 2011. — The descriptor Kestrel uses for loop closure.
* **[Lowe04]** D. G. Lowe. "Distinctive Image Features from Scale-Invariant Keypoints." *IJCV*
  60(2), 2004. — SIFT, and the ratio test for descriptor matching.
* **[Fischler81]** M. A. Fischler, R. C. Bolles. "Random Sample Consensus: A Paradigm for Model
  Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the
  ACM* 24(6), 1981. — RANSAC.
* **[Kneip11]** L. Kneip, D. Scaramuzza, R. Siegwart. "A Novel Parametrization of the
  Perspective-Three-Point Problem for a Direct Computation of Absolute Camera Position and
  Orientation." *CVPR*, 2011. — A fast, stable minimal P3P solver.
* **[Lepetit09]** V. Lepetit, F. Moreno-Noguer, P. Fua. "EPnP: An Accurate O(n) Solution to the
  PnP Problem." *IJCV* 81(2), 2009. — Non-minimal PnP for refinement on inliers.
* **[Huber64]** P. J. Huber. "Robust Estimation of a Location Parameter." *Annals of Mathematical
  Statistics* 35(1), 1964. — The origin of M-estimators and the Huber loss.
* **[Zhang97]** Z. Zhang. "Parameter estimation techniques: a tutorial with application to conic
  fitting." *Image and Vision Computing* 15(1), 1997. — Practical overview of M-estimators and IRLS.

## Optimization and solvers

* **[Triggs00]** B. Triggs, P. McLauchlan, R. Hartley, A. Fitzgibbon. "Bundle Adjustment — A
  Modern Synthesis." *Vision Algorithms: Theory and Practice* (LNCS), 2000. — Everything about BA: Schur complement, gauge, robustification.
* **[Ceres]** S. Agarwal, K. Mierle, and the Ceres Solver Team. *Ceres Solver* (software).
  http://ceres-solver.org — The solver Kestrel uses. Its documentation on manifolds and robust losses is excellent.
* **[Kummerle11]** R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, W. Burgard. "g2o: A General
  Framework for Graph Optimization." *ICRA*, 2011. — An alternative graph-optimization library, widely used in ORB-SLAM.
* **[Kaess12]** M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard, F. Dellaert. "iSAM2:
  Incremental smoothing and mapping using the Bayes tree." *IJRR* 31(2), 2012. — Incremental smoothing, the alternative to fixed-lag windows (GTSAM).

## IMU modelling and preintegration

* **[Allan66]** D. W. Allan. "Statistics of atomic frequency standards." *Proceedings of the IEEE*
  54(2), 1966. — Origin of the Allan variance.
* **[IEEE952]** IEEE Std 952-1997. "IEEE Standard Specification Format Guide and Test Procedure for
  Single-Axis Interferometric Fiber Optic Gyros." — Annex with the standard Allan-variance noise identification.
* **[Lupton12]** T. Lupton, S. Sukkarieh. "Visual-Inertial-Aided Navigation for High-Dynamic Motion
  in Built Environments Without Initial Conditions." *IEEE Transactions on Robotics* 28(1), 2012.
  — The original IMU preintegration idea.
* **[Forster17]** C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza. "On-Manifold Preintegration
  for Real-Time Visual–Inertial Odometry." *IEEE Transactions on Robotics* 33(1), 2017. — **The** preintegration reference: on-manifold noise propagation and bias Jacobians.

## Visual–inertial odometry and SLAM systems

* **[Mourikis07]** A. I. Mourikis, S. I. Roumeliotis. "A Multi-State Constraint Kalman Filter for
  Vision-aided Inertial Navigation." *ICRA*, 2007. — MSCKF, the classic filter-based VIO.
* **[Leutenegger15]** S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, P. Furgale.
  "Keyframe-based visual–inertial odometry using nonlinear optimization." *IJRR* 34(3), 2015.
  — OKVIS: keyframe sliding window with marginalization.
* **[Qin18]** T. Qin, P. Li, S. Shen. "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial
  State Estimator." *IEEE Transactions on Robotics* 34(4), 2018. — The design Kestrel's estimator and 4-DoF pose graph follow most closely.
* **[Qin19]** T. Qin, J. Pan, S. Cao, S. Shen. "A General Optimization-based Framework for Local
  Odometry Estimation with Multiple Sensors." arXiv, 2019. — VINS-Fusion: the stereo / stereo-inertial extension of VINS-Mono.
* **[MurArtal15]** R. Mur-Artal, J. M. M. Montiel, J. D. Tardós. "ORB-SLAM: A Versatile and
  Accurate Monocular SLAM System." *IEEE Transactions on Robotics* 31(5), 2015. — Keyframe SLAM with covisibility, loop closing and relocalization.
* **[MurArtal17]** R. Mur-Artal, J. D. Tardós. "ORB-SLAM2: An Open-Source SLAM System for
  Monocular, Stereo, and RGB-D Cameras." *IEEE Transactions on Robotics* 33(5), 2017. — Stereo extension; a strong baseline.
* **[Campos21]** C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós.
  "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM."
  *IEEE Transactions on Robotics* 37(6), 2021. — Current SOTA baseline; the Atlas multi-map idea parallels Kestrel's sequences.
* **[Usenko20]** V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers. "Visual-Inertial
  Mapping With Non-Linear Factor Recovery." *IEEE Robotics and Automation Letters* 5(2), 2020.
  — Basalt: excellent accuracy/speed trade-off; a key baseline.
* **[Geneva20]** P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, G. Huang. "OpenVINS: A Research Platform
  for Visual-Inertial Estimation." *ICRA*, 2020. — State-of-the-art filter-based VIO, with superb documentation.
* **[Rosinol20]** A. Rosinol, M. Abate, Y. Chang, L. Carlone. "Kimera: an Open-Source Library for
  Real-Time Metric-Semantic Localization and Mapping." *ICRA*, 2020. — VIO plus metric-semantic mapping in one system.
* **[Strasdat12]** H. Strasdat, J. M. M. Montiel, A. J. Davison. "Visual SLAM: Why filter?" *Image
  and Vision Computing* 30(2), 2012. — Why optimization beats filtering per unit of compute.

## Consistency and observability

* **[Sibley10]** G. Sibley, L. Matthies, G. Sukhatme. "Sliding window filter with application to
  planetary landing." *Journal of Field Robotics* 27(5), 2010. — Sliding-window estimation with marginalization.
* **[DongSi11]** T.-C. Dong-Si, A. I. Mourikis. "Motion Tracking with Fixed-lag Smoothing:
  Algorithm and Consistency Analysis." *ICRA*, 2011. — Consistency of fixed-lag smoothers (FEJ in a window).
* **[Huang10]** G. P. Huang, A. I. Mourikis, S. I. Roumeliotis. "Observability-based Rules for
  Designing Consistent EKF SLAM Estimators." *IJRR* 29(5), 2010. — Why mismatched linearization points create false information; FEJ.
* **[Kelly11]** J. Kelly, G. S. Sukhatme. "Visual-Inertial Sensor Fusion: Localization, Mapping and
  Sensor-to-Sensor Self-calibration." *IJRR* 30(1), 2011. — Observability analysis of visual–inertial systems.
* **[Hesch14]** J. A. Hesch, D. G. Kottas, S. L. Bowman, S. I. Roumeliotis. "Consistency Analysis
  and Improvement of Vision-aided Inertial Navigation." *IEEE Transactions on Robotics* 30(1),
  2014. — The 4 unobservable directions of VIO and how to preserve them.

## Place recognition and loop closure

* **[Sivic03]** J. Sivic, A. Zisserman. "Video Google: A Text Retrieval Approach to Object Matching
  in Videos." *ICCV*, 2003. — Bag of visual words with tf–idf.
* **[Nister06]** D. Nistér, H. Stewénius. "Scalable Recognition with a Vocabulary Tree." *CVPR*,
  2006. — Hierarchical k-means vocabularies.
* **[GalvezLopez12]** D. Gálvez-López, J. D. Tardós. "Bags of Binary Words for Fast Place
  Recognition in Image Sequences." *IEEE Transactions on Robotics* 28(5), 2012. — DBoW2: binary vocabularies, normalized scores, islands, temporal consistency.
* **[Cummins08]** M. Cummins, P. Newman. "FAB-MAP: Probabilistic Localization and Mapping in the
  Space of Appearance." *IJRR* 27(6), 2008. — Probabilistic appearance-based place recognition.
* **[Arandjelovic16]** R. Arandjelović, P. Gronat, A. Torii, T. Pajdla, J. Sivic. "NetVLAD: CNN
  architecture for weakly supervised place recognition." *CVPR*, 2016. — Learned global descriptors.
* **[DeTone18]** D. DeTone, T. Malisiewicz, A. Rabinovich. "SuperPoint: Self-Supervised Interest
  Point Detection and Description." *CVPR Workshops*, 2018. — Learned local features.
* **[Sarlin20]** P.-E. Sarlin, D. DeTone, T. Malisiewicz, A. Rabinovich. "SuperGlue: Learning
  Feature Matching with Graph Neural Networks." *CVPR*, 2020. — Learned matching.
* **[Lindenberger23]** P. Lindenberger, P.-E. Sarlin, M. Pollefeys. "LightGlue: Local Feature
  Matching at Light Speed." *ICCV*, 2023. — Faster learned matching; a candidate GPU plug-in.

## Dynamic environments

* **[Saputra18]** M. R. U. Saputra, A. Markham, N. Trigoni. "Visual SLAM and Structure from Motion
  in Dynamic Environments: A Survey." *ACM Computing Surveys* 51(2), 2018. — Taxonomy of approaches.
* **[Bescos18]** B. Bescos, J. M. Fácil, J. Civera, J. Neira. "DynaSLAM: Tracking, Mapping, and
  Inpainting in Dynamic Scenes." *IEEE Robotics and Automation Letters* 3(4), 2018. — Semantic + geometric dynamic-object removal on ORB-SLAM2.
* **[Yu18]** C. Yu et al. "DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments." *IROS*,
  2018. — Semantic segmentation combined with a moving-consistency check.
* **[Scona18]** R. Scona, M. Jaimez, Y. R. Petillot, M. Fallon, D. Cremers. "StaticFusion:
  Background Reconstruction for Dense RGB-D SLAM in Dynamic Environments." *ICRA*, 2018.
  — Joint motion estimation and static/dynamic segmentation.

## Lifelong and long-term mapping

* **[Rosen16]** D. M. Rosen, J. Mason, J. J. Leonard. "Towards Lifelong Feature-Based Mapping in
  Semi-Static Environments." *ICRA*, 2016. — **The persistence filter**, the core of Kestrel's map maintenance.
* **[Biber05]** P. Biber, T. Duckett. "Dynamic Maps for Long-Term Operation of Mobile Service
  Robots." *Robotics: Science and Systems*, 2005. — Maps at multiple timescales.
* **[Churchill13]** W. Churchill, P. Newman. "Experience-based navigation for long-term
  localisation." *IJRR* 32(14), 2013. — Store multiple experiences instead of one map.
* **[Muhlfellner16]** P. Mühlfellner, M. Bürki, M. Bosse, W. Derendarz, R. Philippsen, P. Furgale.
  "Summary Maps for Lifelong Visual Localization." *Journal of Field Robotics*, 2016. — Keep the landmarks that proved useful over many sessions.
* **[Krajnik17]** T. Krajník, J. P. Fentanes, J. M. Santos, T. Duckett. "FreMEn: Frequency Map
  Enhancement for Long-Term Mobile Robot Autonomy in Changing Environments." *IEEE Transactions on
  Robotics* 33(4), 2017. — Periodic models of environment change.
* **[Labbe19]** M. Labbé, F. Michaud. "RTAB-Map as an open-source lidar and visual simultaneous
  localization and mapping library for large-scale and long-term online operation." *Journal of
  Field Robotics* 36(2), 2019. — Memory management for long-running maps; a practical ROS baseline.

## Dense and volumetric mapping

* **[Moravec85]** H. Moravec, A. Elfes. "High resolution maps from wide angle sonar." *ICRA*, 1985.
  — The original occupancy grid.
* **[Hornung13]** A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, W. Burgard. "OctoMap: An
  efficient probabilistic 3D mapping framework based on octrees." *Autonomous Robots* 34(3), 2013.
  — Log-odds occupancy with clamping; Kestrel's default parameters come from here.
* **[Amanatides87]** J. Amanatides, A. Woo. "A Fast Voxel Traversal Algorithm for Ray Tracing."
  *Eurographics*, 1987. — The 3-D DDA used for ray casting.
* **[Teschner03]** M. Teschner, B. Heidelberger, M. Müller, D. Pomeranets, M. Gross. "Optimized
  Spatial Hashing for Collision Detection of Deformable Objects." *Vision, Modeling, and
  Visualization (VMV)*, 2003. — The prime-XOR spatial hash.
* **[Niessner13]** M. Nießner, M. Zollhöfer, S. Izadi, M. Stamminger. "Real-time 3D Reconstruction
  at Scale using Voxel Hashing." *ACM Transactions on Graphics (SIGGRAPH Asia)*, 2013. — Hashed voxel storage for unbounded maps.
* **[Curless96]** B. Curless, M. Levoy. "A volumetric method for building complex models from range
  images." *SIGGRAPH*, 1996. — The TSDF.
* **[Newcombe11]** R. A. Newcombe et al. "KinectFusion: Real-time dense surface mapping and
  tracking." *ISMAR*, 2011. — Real-time TSDF fusion.
* **[Dai17]** A. Dai, M. Nießner, M. Zollhöfer, S. Izadi, C. Theobalt. "BundleFusion: Real-time
  Globally Consistent 3D Reconstruction using On-the-fly Surface Re-integration." *ACM
  Transactions on Graphics*, 2017. — De-integration / re-integration after pose corrections.
* **[Oleynikova17]** H. Oleynikova, Z. Taylor, M. Fehr, R. Siegwart, J. Nieto. "Voxblox:
  Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning." *IROS*, 2017.
  — TSDF/ESDF mapping for planning on robots.

## Datasets and evaluation

* **[Burri16]** M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. W.
  Achtelik, R. Siegwart. "The EuRoC micro aerial vehicle datasets." *IJRR* 35(10), 2016.
  — The standard stereo-inertial benchmark.
* **[Schubert18]** D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stückler, D. Cremers. "The TUM VI
  Benchmark for Evaluating Visual-Inertial Odometry." *IROS*, 2018. — Long, challenging handheld sequences.
* **[Shi20]** X. Shi et al. "Are We Ready for Service Robots? The OpenLORIS-Scene Datasets for
  Lifelong SLAM." *ICRA*, 2020. — Changing, dynamic, revisited scenes; the closest public match to Kestrel's goals.
* **[Sturm12]** J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers. "A Benchmark for the
  Evaluation of RGB-D SLAM Systems." *IROS*, 2012. — Introduced the ATE/RPE definitions used everywhere.
* **[Zhang18]** Z. Zhang, D. Scaramuzza. "A Tutorial on Quantitative Trajectory Evaluation for
  Visual(-Inertial) Odometry." *IROS*, 2018. — Which alignment to use for which sensor setup, and how to compute RPE.
* **[Umeyama91]** S. Umeyama. "Least-squares estimation of transformation parameters between two
  point patterns." *IEEE TPAMI* 13(4), 1991. — Closed-form similarity alignment.
* **[Grupp17]** M. Grupp. *evo: Python package for the evaluation of odometry and SLAM* (software),
  2017. https://github.com/MichaelGrupp/evo — The evaluation tool Kestrel's scripts wrap.
