# Trolley-X evaluation plan (data points -> research questions -> references)

This plan lists the data to collect, and it links each data point to a research
question and to a reference paper. The follow logger records the raw streams. The
static capture tool records ground-truth accuracy stations.

## Research questions (this report)

- RQ1: How does UWB ranging fused with a Kalman filter improve the robustness and
  smoothness of hands-free following, compared with alternative positioning
  methods?
- RQ2: How reliably does a three-zone LiDAR layer detect people and obstacles and
  prevent collisions?
- RQ3: What are the performance differences (accuracy, latency, reliability,
  safety) between the validated simulation and the physical prototype?

## Data points

### RQ1 - UWB + Kalman following
| Data point | How to get it | Reference |
| --- | --- | --- |
| Raw range noise vs filtered noise | log `uwb_left_m`/`uwb_right_m` vs `target_range_m`; compare std | Puriyanto; He |
| Distance-hold error vs 1.0 m | log `dist_hold_err_m`; report mean, std, RMSE | proposal |
| Following smoothness / hunting | log `cmd_ang`/`req_ang`; count sign reversals per minute; angular jerk | He |
| Following latency | cross-correlate operator motion vs `cmd_lin`/odom | He |
| Filter consistency / ride-through | log `innovation_m` and `reject_count`; recovery after a dropout | EKF/UFIR; He |
| Kalman vs raw differential | run once `use_target:=true`, once `false`; same metrics | proposal (alternatives) |

### RQ2 - three-zone LiDAR safety
| Data point | How to get it | Reference |
| --- | --- | --- |
| LiDAR accuracy vs truth | `static_capture` at set distances: `lidar_*` columns | LiDAR review |
| Stopping distance vs speed | drive at set speeds toward a wall; log `cmd_lin` + `lidar_front_min_m` at STOP | VO-CBF |
| Zone-transition correctness | log `safety_state` vs `lidar_front_min_m`; % correct warn/slow/stop | Sousa |
| Minimum separation to a person | walk a person in; min `lidar_front_min_m` reached | Sousa; VO-CBF |
| Time-to-collision at intervention | closing speed and distance at the STOP transition | VO-CBF |
| Nuisance / false-stop rate | count STOPs with no real obstacle over a run | Sousa |

### RQ3 - sim-to-real / system
| Data point | How to get it | Reference |
| --- | --- | --- |
| End-to-end latency (UWB -> cmd) | timestamp gap `target` update -> `cmd_vel` change | proposal |
| Per-topic rate (Hz) | `ros2 topic hz <topic>` | proposal |
| CPU / memory on the Pi | `top -b -n1` / `tegrastats`-style sampler during a run | proposal |
| Node uptime / reliability | run length with no node crash | proposal |
| Power (V, A), energy per metre | power meter reading + odometry distance | Judhi note |
| Heading drift | compare odom `odom_theta_deg` against a known turn | proposal |

### UWB accuracy (Judhi: distance-dependent)
| Data point | How to get it | Reference |
| --- | --- | --- |
| RSSI vs distance (path-loss) | `static_capture`: `rssi_left_mean`/`rssi_right_mean` vs `truth_m` | Puriyanto |
| RSSI vs range error (NLOS flag) | correlate `rssi_*` with per-anchor error | He |
| Position accuracy vs distance | `static_capture`: `fused_range_err`, `fused_lat_std_m`; % within 1 m | He; proposal |
| Anchor-baseline effect on bearing | repeat a station with a wider `anchor_baseline`; compare bearing std | interfaces.md |

## Logged columns (follow_<timestamp>.csv)

t_s, uwb_left_m, uwb_right_m, rssi_left, rssi_right, target_x, target_y,
target_range_m, target_bearing_deg, tag_odom_x, tag_odom_y, odom_x, odom_y,
odom_theta_deg, req_lin, req_ang, cmd_lin, cmd_ang, safety_state,
lidar_front_min_m, innovation_m, reject_count, dist_hold_err_m

## Accuracy stations (accuracy_stations.csv)

truth_m, n_samples, left_*, right_*, uwb_*, fused_range_*, fused_lat_std_m,
rssi_left_mean, rssi_right_mean, lidar_* (mean/err/std for each).
