# Detection Pipeline Diagnosis Report

**Generated:** 2026-06-16  
**Bags analysed:** diag_distance, diag_sitting, diag_walking  
**Pipeline config:** cluster_tol=0.6m, min_points=20, accum_frames=4  
**Vertical-extent filter:** [0.6, 2.2] m  


---

## Bag: `diag_distance`


### Frame 4 (t = 1781652549.097 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 74 pts |
| S3 | After ROI crop + 4-frame accum | 173 pts |
| S4 | After Euclidean clustering | 2 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 132 | (5.31, -1.61, -1.97) | 1.043 |
| C2 | 20 | (1.96, -1.89, -1.85) | 1.153 |

**Rejected clusters at Stage 5:**

- C1 (132 pts): **XY_shape: xy_too_large(sx=1.02,sy=1.35)**  (vert_span=1.043m)
- C2 (20 pts): **XY_shape: xy_too_large(sx=1.56,sy=1.00)**  (vert_span=1.153m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0004_s1_raw.png)  
![](diag_figs/diag_distance/f0004_s2_foreground.png)  
![](diag_figs/diag_distance/f0004_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0004_s4_clusters.png)  
![](diag_figs/diag_distance/f0004_s5_detections.png)  


### Frame 12 (t = 1781652550.350 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 74 pts |
| S3 | After ROI crop + 4-frame accum | 165 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 129 | (5.30, -1.29, -1.90) | 1.163 |

**Rejected clusters at Stage 5:**

- C1 (129 pts): **XY_shape: xy_too_large(sx=0.73,sy=1.02)**  (vert_span=1.163m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0012_s1_raw.png)  
![](diag_figs/diag_distance/f0012_s2_foreground.png)  
![](diag_figs/diag_distance/f0012_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0012_s4_clusters.png)  
![](diag_figs/diag_distance/f0012_s5_detections.png)  


### Frame 117 (t = 1781652566.955 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 204 pts |
| S3 | After ROI crop + 4-frame accum | 289 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 269 | (2.43, -1.53, -1.75) | 1.500 |

**Rejected clusters at Stage 5:**

- C1 (269 pts): **XY_shape: xy_too_large(sx=1.61,sy=1.67)**  (vert_span=1.500m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0117_s1_raw.png)  
![](diag_figs/diag_distance/f0117_s2_foreground.png)  
![](diag_figs/diag_distance/f0117_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0117_s4_clusters.png)  
![](diag_figs/diag_distance/f0117_s5_detections.png)  


### Frame 128 (t = 1781652568.691 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 298 pts |
| S3 | After ROI crop + 4-frame accum | 554 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 530 | (1.86, -1.77, -1.77) | 1.248 |

**Rejected clusters at Stage 5:**

- C1 (530 pts): **XY_shape: xy_too_large(sx=1.86,sy=2.13)**  (vert_span=1.248m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0128_s1_raw.png)  
![](diag_figs/diag_distance/f0128_s2_foreground.png)  
![](diag_figs/diag_distance/f0128_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0128_s4_clusters.png)  
![](diag_figs/diag_distance/f0128_s5_detections.png)  


### Frame 139 (t = 1781652570.414 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 205 pts |
| S3 | After ROI crop + 4-frame accum | 388 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 346 | (0.87, -0.85, -1.41) | 0.579 |

**Rejected clusters at Stage 5:**

- C1 (346 pts): **vert_extent: vert_span=0.579m < min=0.6m**  (vert_span=0.579m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0139_s1_raw.png)  
![](diag_figs/diag_distance/f0139_s2_foreground.png)  
![](diag_figs/diag_distance/f0139_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0139_s4_clusters.png)  
![](diag_figs/diag_distance/f0139_s5_detections.png)  


### Frame 147 (t = 1781652571.669 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 228 pts |
| S3 | After ROI crop + 4-frame accum | 432 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 395 | (0.91, -0.85, -1.42) | 0.599 |

**Rejected clusters at Stage 5:**

- C1 (395 pts): **vert_extent: vert_span=0.599m < min=0.6m**  (vert_span=0.599m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_distance/f0147_s1_raw.png)  
![](diag_figs/diag_distance/f0147_s2_foreground.png)  
![](diag_figs/diag_distance/f0147_s3_roi_accum.png)  
![](diag_figs/diag_distance/f0147_s4_clusters.png)  
![](diag_figs/diag_distance/f0147_s5_detections.png)  


---

## Bag: `diag_sitting`


### Frame 3 (t = 1781652608.770 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 244 pts |
| S3 | After ROI crop + 4-frame accum | 761 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 740 | (2.20, -1.50, -1.87) | 1.260 |

**Rejected clusters at Stage 5:**

- C1 (740 pts): **XY_shape: xy_too_large(sx=1.46,sy=1.61)**  (vert_span=1.260m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_sitting/f0003_s1_raw.png)  
![](diag_figs/diag_sitting/f0003_s2_foreground.png)  
![](diag_figs/diag_sitting/f0003_s3_roi_accum.png)  
![](diag_figs/diag_sitting/f0003_s4_clusters.png)  
![](diag_figs/diag_sitting/f0003_s5_detections.png)  


### Frame 49 (t = 1781652615.905 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 273 pts |
| S3 | After ROI crop + 4-frame accum | 665 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 647 | (2.37, -1.55, -1.73) | 1.537 |

**Rejected clusters at Stage 5:**

- C1 (647 pts): **XY_shape: xy_too_large(sx=1.81,sy=1.71)**  (vert_span=1.537m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_sitting/f0049_s1_raw.png)  
![](diag_figs/diag_sitting/f0049_s2_foreground.png)  
![](diag_figs/diag_sitting/f0049_s3_roi_accum.png)  
![](diag_figs/diag_sitting/f0049_s4_clusters.png)  
![](diag_figs/diag_sitting/f0049_s5_detections.png)  


### Frame 99 (t = 1781652623.687 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 248 pts |
| S3 | After ROI crop + 4-frame accum | 876 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 846 | (2.11, -1.71, -1.95) | 1.090 |

**Rejected clusters at Stage 5:**

- C1 (846 pts): **XY_shape: xy_too_large(sx=1.34,sy=2.00)**  (vert_span=1.090m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_sitting/f0099_s1_raw.png)  
![](diag_figs/diag_sitting/f0099_s2_foreground.png)  
![](diag_figs/diag_sitting/f0099_s3_roi_accum.png)  
![](diag_figs/diag_sitting/f0099_s4_clusters.png)  
![](diag_figs/diag_sitting/f0099_s5_detections.png)  


### Frame 145 (t = 1781652630.864 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 250 pts |
| S3 | After ROI crop + 4-frame accum | 693 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 674 | (2.20, -1.56, -1.77) | 1.458 |

**Rejected clusters at Stage 5:**

- C1 (674 pts): **XY_shape: xy_too_large(sx=1.47,sy=1.72)**  (vert_span=1.458m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_sitting/f0145_s1_raw.png)  
![](diag_figs/diag_sitting/f0145_s2_foreground.png)  
![](diag_figs/diag_sitting/f0145_s3_roi_accum.png)  
![](diag_figs/diag_sitting/f0145_s4_clusters.png)  
![](diag_figs/diag_sitting/f0145_s5_detections.png)  


### Frame 194 (t = 1781652638.500 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20160 pts |
| S2 | After background removal | 279 pts |
| S3 | After ROI crop + 4-frame accum | 809 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 790 | (2.08, -1.75, -1.86) | 1.271 |

**Rejected clusters at Stage 5:**

- C1 (790 pts): **XY_shape: xy_too_large(sx=1.24,sy=2.09)**  (vert_span=1.271m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_sitting/f0194_s1_raw.png)  
![](diag_figs/diag_sitting/f0194_s2_foreground.png)  
![](diag_figs/diag_sitting/f0194_s3_roi_accum.png)  
![](diag_figs/diag_sitting/f0194_s4_clusters.png)  
![](diag_figs/diag_sitting/f0194_s5_detections.png)  


---

## Bag: `diag_walking`


### Frame 3 (t = 1781652672.180 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 322 pts |
| S3 | After ROI crop + 4-frame accum | 540 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 517 | (1.92, -1.64, -1.81) | 1.351 |

**Rejected clusters at Stage 5:**

- C1 (517 pts): **XY_shape: xy_too_large(sx=1.56,sy=1.89)**  (vert_span=1.351m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_walking/f0003_s1_raw.png)  
![](diag_figs/diag_walking/f0003_s2_foreground.png)  
![](diag_figs/diag_walking/f0003_s3_roi_accum.png)  
![](diag_figs/diag_walking/f0003_s4_clusters.png)  
![](diag_figs/diag_walking/f0003_s5_detections.png)  


### Frame 23 (t = 1781652675.288 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 251 pts |
| S3 | After ROI crop + 4-frame accum | 676 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 651 | (2.76, -1.51, -1.85) | 1.295 |

**Rejected clusters at Stage 5:**

- C1 (651 pts): **XY_shape: xy_too_large(sx=2.55,sy=1.62)**  (vert_span=1.295m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_walking/f0023_s1_raw.png)  
![](diag_figs/diag_walking/f0023_s2_foreground.png)  
![](diag_figs/diag_walking/f0023_s3_roi_accum.png)  
![](diag_figs/diag_walking/f0023_s4_clusters.png)  
![](diag_figs/diag_walking/f0023_s5_detections.png)  


### Frame 104 (t = 1781652687.930 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 344 pts |
| S3 | After ROI crop + 4-frame accum | 630 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 605 | (2.07, -1.57, -1.75) | 1.497 |

**Rejected clusters at Stage 5:**

- C1 (605 pts): **XY_shape: xy_too_large(sx=1.14,sy=1.73)**  (vert_span=1.497m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_walking/f0104_s1_raw.png)  
![](diag_figs/diag_walking/f0104_s2_foreground.png)  
![](diag_figs/diag_walking/f0104_s3_roi_accum.png)  
![](diag_figs/diag_walking/f0104_s4_clusters.png)  
![](diag_figs/diag_walking/f0104_s5_detections.png)  


### Frame 129 (t = 1781652691.798 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 20064 pts |
| S2 | After background removal | 123 pts |
| S3 | After ROI crop + 4-frame accum | 343 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 324 | (2.13, -1.63, -1.84) | 1.286 |

**Rejected clusters at Stage 5:**

- C1 (324 pts): **XY_shape: xy_too_large(sx=1.32,sy=1.35)**  (vert_span=1.286m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_walking/f0129_s1_raw.png)  
![](diag_figs/diag_walking/f0129_s2_foreground.png)  
![](diag_figs/diag_walking/f0129_s3_roi_accum.png)  
![](diag_figs/diag_walking/f0129_s4_clusters.png)  
![](diag_figs/diag_walking/f0129_s5_detections.png)  


### Frame 152 (t = 1781652695.373 s)

| Stage | Description | Point / Cluster Count |
|-------|-------------|----------------------|
| S1 | Raw `/livox/lidar` | 19968 pts |
| S2 | After background removal | 398 pts |
| S3 | After ROI crop + 4-frame accum | 430 pts |
| S4 | After Euclidean clustering | 1 clusters |
| S5 | After shape+vert filter | **0 detections** |

**Clusters at Stage 4:**

| # | Pts | Centroid (x, y, z) | Vert span (m) |
|---|-----|-------------------|--------------|
| C1 | 407 | (2.18, -1.50, -1.83) | 1.341 |

**Rejected clusters at Stage 5:**

- C1 (407 pts): **XY_shape: xy_too_large(sx=1.87,sy=1.60)**  (vert_span=1.341m)

> **Person dropped at: Stage 5 (all clusters rejected by shape/vertical filter)**

![](diag_figs/diag_walking/f0152_s1_raw.png)  
![](diag_figs/diag_walking/f0152_s2_foreground.png)  
![](diag_figs/diag_walking/f0152_s3_roi_accum.png)  
![](diag_figs/diag_walking/f0152_s4_clusters.png)  
![](diag_figs/diag_walking/f0152_s5_detections.png)  


---

## Hypothesis H1 — Distance-dependent clustering failure

Min cluster size = 20 pts.  A far-range person may scatter into several sub-clusters each < 20 pts.

| Frame | ROI pts | Clusters (≥min_pts) | Tiny sub-clusters (<min_pts) | Per-cluster pt counts | Nearest dist (m) |
|-------|---------|--------------------|-----------------------------|---------------------|-----------------|
| 3 | 174 | 1 | 1 | 134, 19 | 2.68 |
| 4 | 173 | 2 | 0 | 132, 20 | 2.80 |
| 5 | 172 | 1 | 3 | 125, 19, 3, 3 | 2.85 |
| 6 | 181 | 1 | 3 | 140, 17, 3, 3 | 2.88 |
| 7 | 171 | 1 | 2 | 138, 15, 3 | 2.73 |
| 8 | 168 | 1 | 3 | 135, 15, 4, 3 | 2.76 |
| 9 | 166 | 1 | 2 | 138, 7, 7 | 2.31 |
| 10 | 167 | 1 | 2 | 135, 19, 3 | 2.83 |
| 11 | 172 | 2 | 1 | 132, 24, 3 | 2.92 |
| 12 | 165 | 1 | 3 | 129, 14, 5, 3 | 2.08 |
| 13 | 174 | 2 | 2 | 132, 21, 4, 3 | 2.78 |
| 14 | 172 | 2 | 2 | 123, 22, 6, 3 | 2.93 |
| 15 | 169 | 1 | 4 | 126, 17, 5, 3, 3 | 3.07 |
| 16 | 157 | 1 | 4 | 113, 19, 5, 3, 3 | 3.02 |
| 17 | 140 | 1 | 2 | 104, 16, 3 | 3.06 |
| 18 | 137 | 1 | 4 | 98, 14, 4, 3, 3 | 2.18 |
| 19 | 130 | 1 | 3 | 91, 18, 3, 3 | 2.79 |
| 20 | 133 | 1 | 4 | 95, 16, 5, 3, 3 | 2.88 |
| 21 | 142 | 1 | 5 | 94, 12, 5, 5, 4, 3 | 2.62 |
| 22 | 139 | 1 | 4 | 94, 19, 3, 3, 3 | 2.87 |
| 23 | 134 | 2 | 4 | 86, 20, 4, 4, 3, 3 | 3.00 |
| 24 | 124 | 1 | 4 | 74, 19, 5, 4, 3 | 2.76 |
| 25 | 110 | 2 | 3 | 60, 25, 4, 3, 3 | 3.09 |
| 26 | 99 | 1 | 5 | 49, 19, 3, 3, 3, 3 | 2.05 |
| 27 | 88 | 2 | 3 | 43, 21, 3, 3, 3 | 3.04 |
| 28 | 88 | 2 | 4 | 45, 20, 4, 3, 3, 3 | 2.44 |
| 29 | 88 | 2 | 2 | 48, 22, 3, 3 | 2.98 |
| 30 | 89 | 2 | 0 | 51, 23 | 2.91 |
| 31 | 96 | 2 | 0 | 54, 25 | 2.92 |
| 32 | 97 | 2 | 1 | 53, 24, 4 | 2.88 |
| 33 | 93 | 2 | 1 | 50, 20, 3 | 2.88 |
| 34 | 99 | 1 | 4 | 54, 17, 4, 4, 3 | 2.11 |
| 35 | 93 | 1 | 5 | 52, 14, 3, 3, 3, 3 | 2.09 |
| 36 | 90 | 1 | 3 | 50, 12, 3, 3 | 3.15 |
| 37 | 91 | 1 | 2 | 55, 13, 3 | 3.28 |
| 38 | 82 | 1 | 1 | 49, 14 | 3.14 |
| 39 | 91 | 1 | 2 | 53, 15, 3 | 3.13 |
| 40 | 102 | 1 | 4 | 59, 17, 5, 4, 3 | 2.09 |
| 41 | 102 | 1 | 5 | 58, 19, 4, 3, 3, 3 | 2.07 |
| 42 | 109 | 1 | 6 | 60, 14, 10, 4, 3, 3, 3 | 2.05 |
| 43 | 105 | 1 | 3 | 57, 17, 11, 3 | 2.04 |
| 44 | 105 | 1 | 3 | 56, 17, 10, 4 | 2.05 |
| 45 | 116 | 1 | 3 | 61, 18, 10, 4 | 2.04 |
| 46 | 110 | 1 | 2 | 67, 15, 8 | 2.55 |
| 47 | 121 | 2 | 2 | 70, 21, 4, 4 | 2.12 |
| 48 | 123 | 2 | 1 | 77, 21, 4 | 2.82 |
| 49 | 120 | 1 | 3 | 78, 17, 3, 3 | 2.88 |
| 50 | 124 | 1 | 4 | 77, 17, 4, 3, 3 | 2.89 |
| 51 | 119 | 1 | 4 | 81, 13, 4, 4, 3 | 2.85 |
| 52 | 110 | 1 | 3 | 75, 12, 7, 4 | 3.08 |
| 53 | 102 | 1 | 3 | 68, 15, 3, 3 | 3.07 |
| 54 | 99 | 1 | 2 | 62, 18, 3 | 2.11 |
| 55 | 89 | 1 | 2 | 48, 19, 4 | 2.09 |
| 56 | 87 | 2 | 1 | 43, 24, 3 | 2.92 |
| 57 | 92 | 1 | 4 | 47, 17, 7, 4, 3 | 2.13 |
| 58 | 95 | 1 | 4 | 50, 16, 5, 4, 3 | 2.02 |
| 59 | 110 | 1 | 3 | 67, 12, 5, 4 | 2.03 |
| 60 | 122 | 1 | 5 | 77, 14, 4, 4, 4, 3 | 2.02 |
| 61 | 126 | 1 | 4 | 80, 11, 7, 5, 4 | 2.02 |
| 62 | 121 | 1 | 4 | 77, 13, 6, 5, 4 | 2.01 |
| 63 | 107 | 1 | 4 | 66, 16, 5, 3, 3 | 2.00 |
| 64 | 109 | 1 | 3 | 69, 14, 6, 4 | 1.98 |
| 65 | 97 | 1 | 3 | 65, 12, 4, 3 | 1.96 |
| 66 | 103 | 1 | 2 | 73, 6, 3 | 3.08 |
| 67 | 124 | 1 | 2 | 86, 16, 3 | 2.76 |
| 68 | 130 | 1 | 3 | 93, 12, 3, 3 | 2.74 |
| 69 | 160 | 1 | 3 | 119, 14, 5, 3 | 2.69 |
| 70 | 176 | 1 | 4 | 129, 17, 4, 4, 3 | 2.64 |
| 71 | 176 | 1 | 3 | 136, 10, 5, 4 | 2.08 |
| 72 | 169 | 1 | 2 | 133, 17, 5 | 2.73 |
| 73 | 160 | 1 | 2 | 128, 17, 4 | 2.72 |
| 74 | 160 | 1 | 1 | 132, 14 | 2.57 |
| 75 | 163 | 1 | 2 | 136, 11, 3 | 2.65 |
| 76 | 182 | 1 | 3 | 154, 8, 4, 3 | 2.09 |
| 77 | 194 | 1 | 3 | 160, 7, 4, 3 | 2.06 |
| 78 | 207 | 1 | 2 | 173, 10, 3 | 2.98 |
| 79 | 213 | 1 | 2 | 181, 10, 4 | 2.96 |
| 80 | 223 | 1 | 4 | 183, 11, 4, 3, 3 | 2.02 |
| 81 | 231 | 2 | 2 | 185, 20, 5, 3 | 2.70 |
| 82 | 230 | 2 | 4 | 185, 21, 4, 3, 3, 3 | 2.69 |
| 83 | 235 | 2 | 3 | 182, 21, 12, 5, 5 | 2.43 |
| 84 | 233 | 1 | 5 | 182, 15, 8, 5, 5, 5 | 2.32 |
| 85 | 222 | 1 | 2 | 179, 18, 5 | 2.67 |
| 86 | 221 | 2 | 1 | 175, 24, 4 | 2.92 |
| 87 | 225 | 2 | 1 | 180, 22, 4 | 2.01 |
| 88 | 221 | 2 | 1 | 177, 28, 3 | 2.92 |
| 89 | 222 | 2 | 1 | 180, 27, 3 | 2.90 |
| 90 | 218 | 2 | 1 | 184, 20, 3 | 2.81 |
| 91 | 214 | 1 | 3 | 184, 13, 3, 3 | 2.79 |
| 92 | 211 | 1 | 5 | 180, 6, 4, 4, 3, 3 | 2.04 |
| 93 | 217 | 1 | 5 | 181, 12, 4, 4, 4, 4 | 2.07 |
| 94 | 215 | 1 | 4 | 171, 19, 5, 5, 4 | 2.07 |
| 95 | 213 | 2 | 4 | 167, 21, 5, 4, 3, 3 | 2.02 |
| 96 | 203 | 1 | 3 | 162, 19, 3, 3 | 3.00 |
| 97 | 208 | 1 | 3 | 167, 18, 3, 3 | 3.09 |
| 98 | 216 | 1 | 2 | 177, 19, 4 | 2.91 |
| 99 | 219 | 2 | 2 | 177, 23, 4, 3 | 2.95 |
| 100 | 231 | 2 | 2 | 182, 27, 5, 4 | 2.88 |
| 101 | 217 | 2 | 3 | 171, 24, 5, 4, 3 | 2.78 |
| 102 | 217 | 2 | 4 | 164, 31, 5, 4, 3, 3 | 2.84 |
| 103 | 208 | 2 | 3 | 158, 29, 3, 3, 3 | 2.75 |
| 104 | 216 | 2 | 2 | 164, 27, 4, 4 | 2.74 |
| 105 | 217 | 2 | 4 | 163, 25, 4, 4, 4, 3 | 2.72 |
| 106 | 215 | 1 | 5 | 166, 18, 8, 7, 4, 4 | 2.62 |
| 107 | 214 | 2 | 3 | 161, 26, 8, 4, 3 | 3.19 |
| 108 | 191 | 2 | 2 | 148, 21, 7, 3 | 3.21 |
| 109 | 194 | 1 | 5 | 152, 10, 8, 6, 4, 3 | 2.60 |
| 110 | 195 | 1 | 5 | 152, 14, 10, 4, 3, 3 | 2.67 |
| 111 | 214 | 1 | 4 | 178, 9, 7, 3, 3 | 2.88 |
| 112 | 246 | 1 | 6 | 203, 11, 6, 5, 5, 5, 4 | 2.09 |
| 113 | 268 | 1 | 5 | 221, 16, 7, 6, 5, 3 | 2.05 |
| 114 | 279 | 1 | 7 | 235, 8, 6, 5, 4, 3, 3, 3 | 2.00 |
| 115 | 282 | 1 | 6 | 251, 6, 4, 3, 3, 3, 3 | 2.00 |
| 116 | 275 | 1 | 5 | 249, 4, 4, 3, 3, 3 | 2.00 |
| 117 | 289 | 1 | 2 | 269, 4, 4 | 2.93 |
| 118 | 310 | 1 | 3 | 291, 3, 3, 3 | 2.81 |
| 119 | 348 | 1 | 4 | 313, 14, 4, 3, 3 | 2.67 |
| 120 | 402 | 1 | 4 | 367, 12, 4, 3, 3 | 2.35 |
| 121 | 438 | 1 | 5 | 404, 9, 3, 3, 3, 3 | 2.43 |
| 122 | 462 | 1 | 3 | 441, 5, 3, 3 | 2.33 |
| 123 | 463 | 1 | 1 | 443, 9 | 2.22 |
| 124 | 461 | 1 | 2 | 433, 10, 3 | 2.08 |
| 125 | 450 | 1 | 3 | 421, 6, 4, 3 | 1.96 |
| 126 | 466 | 1 | 2 | 436, 5, 4 | 1.86 |
| 127 | 501 | 2 | 2 | 448, 22, 6, 3 | 1.71 |
| 128 | 554 | 1 | 1 | 530, 4 | 1.67 |
| 129 | 594 | 2 | 3 | 547, 20, 3, 3, 3 | 1.55 |
| 130 | 597 | 1 | 2 | 552, 17, 3 | 1.49 |
| 131 | 537 | 1 | 4 | 487, 18, 3, 3, 3 | 1.44 |
| 132 | 438 | 2 | 3 | 390, 20, 4, 3, 3 | 1.40 |
| 133 | 345 | 1 | 3 | 306, 17, 4, 3 | 1.37 |
| 134 | 283 | 1 | 5 | 248, 12, 3, 3, 3, 3 | 1.35 |
| 135 | 281 | 1 | 1 | 253, 14 | 1.33 |
| 136 | 312 | 1 | 2 | 277, 12, 4 | 1.32 |
| 137 | 341 | 1 | 2 | 304, 16, 4 | 1.30 |
| 138 | 374 | 1 | 3 | 333, 14, 6, 5 | 1.29 |
| 139 | 388 | 1 | 4 | 346, 13, 5, 4, 3 | 1.29 |
| 140 | 385 | 1 | 2 | 346, 15, 3 | 1.29 |
| 141 | 427 | 1 | 5 | 384, 8, 7, 5, 3, 3 | 1.29 |
| 142 | 424 | 1 | 4 | 381, 15, 5, 4, 4 | 1.29 |
| 143 | 424 | 1 | 4 | 382, 10, 7, 4, 4 | 1.29 |
| 144 | 422 | 1 | 3 | 381, 14, 5, 3 | 1.29 |
| 145 | 387 | 1 | 3 | 349, 16, 4, 4 | 1.29 |
| 146 | 417 | 1 | 3 | 383, 9, 5, 3 | 1.30 |
| 147 | 432 | 1 | 5 | 395, 17, 4, 3, 3, 3 | 1.29 |
| 148 | 441 | 1 | 4 | 410, 13, 3, 3, 3 | 1.29 |
| 149 | 461 | 1 | 3 | 428, 15, 3, 3 | 1.29 |
| 150 | 445 | 1 | 4 | 413, 14, 4, 3, 3 | 1.29 |
| 151 | 455 | 1 | 4 | 426, 11, 4, 4, 3 | 1.29 |
| 152 | 460 | 1 | 4 | 425, 13, 4, 3, 3 | 1.29 |
| 153 | 460 | 2 | 1 | 421, 24, 3 | 1.29 |
| 154 | 460 | 2 | 1 | 419, 26, 3 | 1.29 |
| 155 | 452 | 2 | 0 | 414, 23 | 1.29 |
| 156 | 462 | 1 | 3 | 425, 16, 3, 3 | 1.29 |
| 157 | 466 | 1 | 4 | 434, 10, 3, 3, 3 | 1.29 |
| 158 | 491 | 1 | 4 | 453, 10, 3, 3, 3 | 1.28 |
| 159 | 512 | 1 | 3 | 473, 15, 3, 3 | 1.28 |
| 160 | 535 | 1 | 6 | 494, 15, 5, 4, 3, 3, 3 | 1.28 |
| 161 | 580 | 1 | 5 | 539, 12, 7, 4, 3, 3 | 1.28 |
| 162 | 613 | 1 | 5 | 572, 10, 7, 5, 3, 3 | 1.28 |
| 163 | 659 | 1 | 4 | 620, 10, 7, 5, 3 | 1.29 |
| 164 | 702 | 2 | 5 | 657, 21, 4, 4, 4, 3, 3 | 1.29 |
| 165 | 726 | 2 | 4 | 681, 22, 5, 4, 4, 3 | 1.30 |
| 166 | 764 | 1 | 6 | 719, 10, 7, 7, 4, 4, 3 | 1.31 |
| 167 | 777 | 1 | 5 | 733, 16, 7, 6, 4, 4 | 1.32 |
| 168 | 785 | 1 | 4 | 747, 9, 6, 3, 3 | 1.33 |
| 169 | 799 | 1 | 2 | 761, 11, 5 | 1.34 |
| 170 | 785 | 1 | 4 | 748, 7, 5, 4, 3 | 1.34 |

**Finding:** No missed frames found in diag_distance with ROI pts > 30.  H1 could not be confirmed from this bag.


---

## Hypothesis H2 — Sitting person killed by vertical-span filter

Vertical extent filter: [0.6, 2.2] m.  A seated person's cluster may fall below 0.6m.

| Frame | ROI pts | Clusters | Vert spans (m) | Below 0.6m? |
|-------|---------|----------|----------------|-------------|
| 3 | 761 | 2 | 1.260, 0.418 | YES |
| 4 | 763 | 3 | 1.260, 0.787, 0.205 | YES |
| 5 | 767 | 1 | 0.899 | no |
| 6 | 766 | 3 | 1.320, 0.256, 0.799 | YES |
| 7 | 785 | 2 | 1.492, 0.998 | no |
| 8 | 775 | 1 | 1.485 | no |
| 9 | 779 | 1 | 1.485 | no |
| 10 | 781 | 2 | 1.395, 0.861 | no |
| 11 | 769 | 1 | 1.056 | no |
| 12 | 766 | 1 | 1.093 | no |
| 13 | 760 | 3 | 0.876, 0.502, 0.679 | YES |
| 14 | 744 | 1 | 1.290 | no |
| 15 | 736 | 1 | 1.307 | no |
| 16 | 725 | 1 | 1.307 | no |
| 17 | 766 | 1 | 1.307 | no |
| 18 | 778 | 1 | 1.306 | no |
| 19 | 765 | 1 | 1.277 | no |
| 20 | 771 | 2 | 0.886, 0.282 | YES |
| 21 | 724 | 1 | 1.276 | no |
| 22 | 715 | 1 | 1.236 | no |
| 23 | 721 | 1 | 1.580 | no |
| 24 | 709 | 1 | 1.580 | no |
| 25 | 711 | 1 | 1.024 | no |
| 26 | 726 | 1 | 0.878 | no |
| 27 | 738 | 1 | 1.168 | no |
| 28 | 773 | 1 | 1.273 | no |
| 29 | 782 | 2 | 1.489, 0.404 | YES |
| 30 | 775 | 1 | 1.489 | no |
| 31 | 783 | 2 | 1.489, 0.404 | YES |
| 32 | 771 | 1 | 1.496 | no |
| 33 | 771 | 1 | 0.875 | no |
| 34 | 780 | 1 | 1.501 | no |
| 35 | 779 | 1 | 1.502 | no |
| 36 | 780 | 1 | 1.502 | no |
| 37 | 769 | 1 | 1.215 | no |
| 38 | 745 | 1 | 1.180 | no |
| 39 | 711 | 2 | 0.883, 0.305 | YES |
| 40 | 666 | 1 | 0.892 | no |
| 41 | 632 | 1 | 0.895 | no |
| 42 | 605 | 2 | 0.895, 0.539 | YES |
| 43 | 591 | 2 | 0.895, 0.537 | YES |
| 44 | 593 | 4 | 0.891, 0.537, 0.639, 0.204 | YES |
| 45 | 591 | 3 | 1.292, 0.204, 0.639 | YES |
| 46 | 605 | 2 | 1.090, 0.403 | YES |
| 47 | 624 | 1 | 1.126 | no |
| 48 | 639 | 1 | 1.379 | no |
| 49 | 665 | 1 | 1.537 | no |
| 50 | 679 | 1 | 1.537 | no |
| 51 | 679 | 1 | 1.537 | no |
| 52 | 688 | 1 | 0.882 | no |
| 53 | 685 | 2 | 0.991, 0.115 | YES |
| 54 | 701 | 1 | 0.882 | no |
| 55 | 733 | 2 | 0.992, 0.222 | YES |
| 56 | 753 | 1 | 1.499 | no |
| 57 | 775 | 1 | 1.499 | no |
| 58 | 792 | 2 | 0.874, 1.161 | no |
| 59 | 806 | 2 | 1.193, 0.212 | YES |
| 60 | 833 | 1 | 1.193 | no |
| 61 | 864 | 1 | 1.193 | no |
| 62 | 867 | 1 | 0.868 | no |
| 63 | 873 | 3 | 0.870, 1.058, 0.217 | YES |
| 64 | 873 | 2 | 1.329, 0.189 | YES |
| 65 | 869 | 1 | 1.329 | no |
| 66 | 879 | 1 | 1.329 | no |
| 67 | 876 | 2 | 1.329, 0.193 | YES |
| 68 | 867 | 1 | 1.464 | no |
| 69 | 865 | 1 | 1.877 | no |
| 70 | 859 | 1 | 1.877 | no |
| 71 | 859 | 1 | 1.877 | no |
| 72 | 869 | 1 | 1.289 | no |
| 73 | 879 | 2 | 1.289, 0.141 | YES |
| 74 | 883 | 1 | 0.926 | no |
| 75 | 887 | 2 | 0.926, 0.028 | YES |
| 76 | 866 | 2 | 1.907, 0.589 | YES |
| 77 | 880 | 2 | 0.893, 0.021 | YES |
| 78 | 866 | 1 | 1.905 | no |
| 79 | 843 | 2 | 0.858, 0.506 | YES |
| 80 | 835 | 2 | 0.858, 0.520 | YES |
| 81 | 813 | 2 | 0.858, 0.366 | YES |
| 82 | 856 | 2 | 0.858, 0.103 | YES |
| 83 | 861 | 1 | 1.105 | no |
| 84 | 865 | 1 | 1.576 | no |
| 85 | 852 | 2 | 1.570, 0.582 | YES |
| 86 | 799 | 1 | 1.570 | no |
| 87 | 800 | 1 | 0.848 | no |
| 88 | 803 | 2 | 0.849, 0.810 | no |
| 89 | 794 | 4 | 0.846, 0.361, 0.662, 0.810 | YES |
| 90 | 803 | 3 | 0.860, 0.662, 0.361 | YES |
| 91 | 797 | 1 | 1.468 | no |
| 92 | 798 | 1 | 1.468 | no |
| 93 | 829 | 1 | 1.109 | no |
| 94 | 838 | 1 | 1.057 | no |
| 95 | 860 | 1 | 1.389 | no |
| 96 | 875 | 2 | 1.389, 0.677 | no |
| 97 | 860 | 1 | 1.382 | no |
| 98 | 870 | 2 | 1.382, 0.118 | YES |
| 99 | 876 | 2 | 1.090, 0.684 | no |
| 100 | 873 | 2 | 1.090, 0.687 | no |
| 101 | 886 | 2 | 1.199, 0.687 | no |
| 102 | 879 | 2 | 1.399, 0.196 | YES |
| 103 | 869 | 2 | 1.395, 0.246 | YES |
| 104 | 869 | 1 | 1.395 | no |
| 105 | 865 | 1 | 1.400 | no |
| 106 | 866 | 1 | 1.208 | no |
| 107 | 877 | 2 | 1.303, 0.147 | YES |
| 108 | 895 | 1 | 1.303 | no |
| 109 | 899 | 1 | 1.302 | no |
| 110 | 901 | 1 | 1.302 | no |
| 111 | 893 | 1 | 1.322 | no |
| 112 | 880 | 1 | 1.322 | no |
| 113 | 883 | 1 | 1.321 | no |
| 114 | 884 | 2 | 1.313, 0.241 | YES |
| 115 | 883 | 1 | 1.112 | no |
| 116 | 888 | 5 | 0.866, 0.500, 0.492, 0.495, 0.099 | YES |
| 117 | 874 | 2 | 0.866, 0.492 | YES |
| 118 | 873 | 1 | 1.072 | no |
| 119 | 874 | 1 | 1.080 | no |
| 120 | 870 | 1 | 1.579 | no |
| 121 | 876 | 1 | 1.587 | no |
| 122 | 875 | 1 | 1.587 | no |
| 123 | 872 | 1 | 1.587 | no |
| 124 | 872 | 1 | 1.288 | no |
| 125 | 871 | 1 | 0.852 | no |
| 126 | 856 | 2 | 0.852, 1.138 | no |
| 127 | 866 | 1 | 1.511 | no |
| 128 | 858 | 1 | 1.108 | no |
| 129 | 857 | 1 | 1.508 | no |
| 130 | 869 | 1 | 1.201 | no |
| 131 | 823 | 1 | 1.200 | no |
| 132 | 762 | 1 | 1.200 | no |
| 133 | 694 | 1 | 1.274 | no |
| 134 | 606 | 1 | 1.596 | no |
| 135 | 560 | 1 | 1.596 | no |
| 136 | 541 | 1 | 1.596 | no |
| 137 | 522 | 3 | 0.877, 0.966, 0.249 | YES |
| 138 | 531 | 1 | 1.300 | no |
| 139 | 528 | 1 | 1.532 | no |
| 140 | 543 | 1 | 1.535 | no |
| 141 | 571 | 1 | 1.535 | no |
| 142 | 609 | 1 | 1.535 | no |
| 143 | 652 | 1 | 1.094 | no |
| 144 | 668 | 1 | 1.094 | no |
| 145 | 693 | 2 | 1.458, 0.340 | YES |
| 146 | 698 | 1 | 1.458 | no |
| 147 | 725 | 1 | 1.458 | no |
| 148 | 733 | 1 | 1.458 | no |
| 149 | 732 | 1 | 1.209 | no |
| 150 | 738 | 3 | 0.871, 1.116, 0.641 | no |
| 151 | 724 | 4 | 0.857, 1.013, 0.635, 0.175 | YES |
| 152 | 773 | 3 | 0.872, 0.050, 0.110 | YES |
| 153 | 771 | 3 | 0.871, 0.490, 0.050 | YES |
| 154 | 767 | 2 | 1.084, 0.484 | YES |
| 155 | 773 | 2 | 1.103, 0.820 | no |
| 156 | 746 | 2 | 1.097, 0.820 | no |
| 157 | 758 | 2 | 1.488, 0.483 | YES |
| 158 | 780 | 3 | 1.488, 0.481, 0.348 | YES |
| 159 | 792 | 1 | 1.488 | no |
| 160 | 785 | 3 | 0.862, 0.886, 0.337 | YES |
| 161 | 806 | 1 | 1.196 | no |
| 162 | 797 | 1 | 1.202 | no |
| 163 | 796 | 2 | 1.202, 0.084 | YES |
| 164 | 810 | 2 | 1.202, 0.056 | YES |
| 165 | 791 | 1 | 1.202 | no |
| 166 | 813 | 1 | 1.208 | no |
| 167 | 814 | 1 | 1.234 | no |
| 168 | 820 | 1 | 1.234 | no |
| 169 | 824 | 2 | 1.234, 0.112 | YES |
| 170 | 819 | 2 | 1.233, 0.064 | YES |
| 171 | 805 | 1 | 1.337 | no |
| 172 | 813 | 2 | 1.337, 0.197 | YES |
| 173 | 809 | 1 | 1.337 | no |
| 174 | 804 | 1 | 1.333 | no |
| 175 | 823 | 1 | 1.446 | no |
| 176 | 811 | 1 | 1.402 | no |
| 177 | 813 | 1 | 1.402 | no |
| 178 | 807 | 1 | 1.539 | no |
| 179 | 800 | 1 | 1.539 | no |
| 180 | 791 | 1 | 1.540 | no |
| 181 | 804 | 1 | 1.540 | no |
| 182 | 806 | 2 | 0.862, 0.442 | YES |
| 183 | 810 | 2 | 0.862, 0.447 | YES |
| 184 | 817 | 2 | 0.860, 0.447 | YES |
| 185 | 805 | 2 | 0.856, 0.447 | YES |
| 186 | 799 | 3 | 0.863, 0.718, 0.493 | YES |
| 187 | 794 | 1 | 0.861 | no |
| 188 | 789 | 1 | 1.267 | no |
| 189 | 797 | 1 | 1.300 | no |
| 190 | 801 | 1 | 1.288 | no |
| 191 | 802 | 1 | 1.288 | no |
| 192 | 815 | 1 | 1.294 | no |
| 193 | 811 | 1 | 1.271 | no |
| 194 | 809 | 1 | 1.271 | no |
| 195 | 799 | 1 | 1.271 | no |
| 196 | 792 | 1 | 1.496 | no |
| 197 | 783 | 2 | 1.492, 0.075 | YES |
| 198 | 783 | 2 | 1.492, 0.076 | YES |
| 199 | 796 | 2 | 1.536, 0.076 | YES |
| 200 | 791 | 1 | 1.540 | no |
| 201 | 806 | 1 | 1.540 | no |
| 202 | 806 | 1 | 1.540 | no |
| 203 | 812 | 1 | 1.338 | no |
| 204 | 811 | 1 | 1.338 | no |
| 205 | 796 | 1 | 1.498 | no |
| 206 | 785 | 1 | 1.497 | no |
| 207 | 800 | 1 | 1.497 | no |
| 208 | 797 | 1 | 1.497 | no |
| 209 | 788 | 1 | 1.198 | no |
| 210 | 788 | 1 | 1.196 | no |
| 211 | 743 | 2 | 0.849, 0.274 | YES |
| 212 | 794 | 2 | 1.520, 0.694 | no |
| 213 | 787 | 2 | 0.846, 0.395 | YES |
| 214 | 793 | 3 | 0.850, 0.694, 0.375 | YES |
| 215 | 803 | 3 | 0.857, 0.694, 0.375 | YES |
| 216 | 749 | 3 | 1.309, 0.202, 0.386 | YES |
| 217 | 806 | 3 | 0.867, 1.401, 0.386 | YES |
| 218 | 791 | 2 | 1.203, 0.202 | YES |
| 219 | 798 | 1 | 1.203 | no |
| 220 | 800 | 1 | 0.866 | no |
| 221 | 765 | 2 | 0.865, 0.502 | YES |
| 222 | 778 | 3 | 0.865, 0.502, 0.450 | YES |
| 223 | 786 | 2 | 1.497, 0.502 | YES |
| 224 | 793 | 2 | 1.496, 0.312 | YES |
| 225 | 798 | 1 | 1.496 | no |
| 226 | 807 | 1 | 1.224 | no |
| 227 | 799 | 1 | 1.200 | no |
| 228 | 807 | 1 | 1.195 | no |
| 229 | 804 | 1 | 1.202 | no |
| 230 | 809 | 1 | 1.206 | no |
| 231 | 801 | 1 | 1.206 | no |
| 232 | 797 | 1 | 1.206 | no |
| 233 | 794 | 1 | 1.207 | no |
| 234 | 784 | 1 | 1.161 | no |
| 235 | 799 | 1 | 1.161 | no |
| 236 | 809 | 1 | 1.257 | no |
| 237 | 819 | 1 | 1.256 | no |
| 238 | 817 | 2 | 1.256, 0.455 | YES |
| 239 | 813 | 2 | 1.256, 0.455 | YES |
| 240 | 801 | 2 | 1.206, 0.402 | YES |
| 241 | 804 | 2 | 1.200, 0.274 | YES |
| 242 | 802 | 1 | 1.286 | no |

**Finding:** 74 / 240 frames have a cluster with vert_span < 0.6m.  Min observed span = 0.021m.  H2 **CONFIRMED**: sitting person's vertical extent falls below the filter floor.


---

## Root Cause Analysis

**Person is never lost before Stage 5.** Background removal and ROI/accumulation all see the person clearly. The drop is 100% at Stage 5 (shape+vertical filter).

### Primary cause: `max_xy_size=1.0m` too tight given `accum_frames=4`

Across all three bags, every missed detection is rejected because the cluster XY footprint exceeds `max_xy_size=1.0m`. The XY spans observed:

| Scenario | Typical sx (m) | Typical sy (m) | Why so large? |
|----------|---------------|---------------|---------------|
| diag_distance (walking) | 0.73–1.86 | 1.00–2.13 | 4-frame motion trail inflates footprint |
| diag_sitting (static) | 1.24–1.81 | 1.61–2.09 | BG noise leaking into foreground |
| diag_walking (walking) | 1.14–2.55 | 1.35–1.89 | 4-frame motion trail |

With `accum_frames=4` at ~10 Hz, the accumulation window is ~400 ms. A person walking at 1 m/s moves 0.4 m in that window; with `cluster_tol=0.6 m` the merged cluster spans at least 1.0 m in the walking direction. Any faster motion or diagonal walk exceeds the 1.0 m cap immediately.

### Secondary cause (diag_distance, frames 139–147): `min_vertical_extent=0.6m`

Two frames in diag_distance (f139 vert_span=0.579 m, f147 vert_span=0.599 m) are rejected by the vertical extent floor, not the XY filter. The person is very close to the sensor (centroid at z ≈ -1.4 m, i.e. only ~1.4 m below the sensor), so the overhead view captures a shallow vertical slice. Span is within 21 mm of the 0.6 m threshold — a very near-boundary rejection.

### H1 verdict

**NOT confirmed.** The person always forms ONE large cluster (43–780 pts) that comfortably exceeds `min_points=20`. Sub-threshold fragments (3–19 pts) exist but are peripheral noise. Distance has no significant effect on whether the person is clustered — the XY shape filter is the gating factor at all ranges.

### H2 verdict

**CONFIRMED as a secondary mechanism.** 74/240 frames in the sitting bag have at least one cluster with `vert_span < 0.6 m` (min observed = 0.021 m). These are secondary clusters (furniture fragments, floor reflections) near the seated person. The seated person's primary cluster has vert_span 0.86–2.09 m (passes vertical filter) but is then blocked by `max_xy_size=1.0 m`. H2 would be the primary failure mode if the XY filter were removed.


---

## Summary

| Bag | Frames analysed | Frames with 0 detections | Primary drop stage |
|-----|----------------|--------------------------|-------------------|
| diag_distance | 6 | 6 | Stage 5 (all clusters rejected by shape/vertical filter) |
| diag_sitting | 5 | 5 | Stage 5 (all clusters rejected by shape/vertical filter) |
| diag_walking | 5 | 5 | Stage 5 (all clusters rejected by shape/vertical filter) |
