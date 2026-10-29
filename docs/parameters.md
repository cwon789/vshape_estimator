# ROS IEKF parameter convention

Shipped `config/vshape.yaml` follows the wall/reflector ROS names where the meanings
agree. Geometry/front-end parameters (`arm_length`, `opening_angle_deg`,
`association_distance`, `min_support`, `min_points`) retain geometric names.
Standalone C++ `Config` and replay `C` keys remain unchanged: they are separate APIs.

| Former V-shape ROS key | Current ROS key | Meaning / units |
|---|---|---|
| `odom_topic` | `iekf_odom_topic` | Body-twist Odometry topic |
| `q_linear` | `iekf_process_noise_lin` | **Covariance density** m²/s, converted to sqrt for core |
| `q_angular` | `iekf_process_noise_ang` | **Covariance density** rad²/s, converted to sqrt for core |
| `range_sigma` | `iekf_measurement_noise_pos` | Per-return radial standard deviation, m |
| `angle_sigma` | `iekf_measurement_noise_angle` | Per-ray angular standard deviation, rad |
| `model_sigma` | `iekf_arm_model_sigma` | Shared within-arm model standard deviation, m |
| `odom_timeout` | `iekf_odom_extrapolation_max_dt` | Maximum age for twist ZOH, s |
| `unknown_linear` | `iekf_odom_gap_linear_noise_density` | Gap diffusion density, m/sqrt(s) |
| `unknown_angular` | `iekf_odom_gap_angular_noise_density` | Gap diffusion density, rad/sqrt(s) |
| `gate_sigma` | `iekf_chi2_gate_sigma` | One-sided Gaussian equivalent of variable-dimension chi² probability |
| `huber_delta` | `iekf_huber_threshold` | Normalized Huber threshold; 0 disables |
| `max_iterations` | `iekf_max_iterations` | Iterated MAP update limit |
| `history_sec` | `iekf_state_history_duration` | Retained event history duration, s |
| `max_events` | `iekf_state_history_size` | Maximum retained scan+odometry events |
| `init_position_sigma` | `iekf_initial_position_sigma` | Initial marker pose position standard deviation, m |
| `init_yaw_sigma` | `iekf_initial_yaw_sigma` | Initial marker pose yaw standard deviation, rad |
| `measurement_timeout` | `iekf_no_measurement_timeout_sec` | Output diagnostic coasting timeout, s |

`iekf_process_noise_lin/ang` have the same covariance convention as
wall/reflector `setProcessNoise(q_lin,q_ang)`. The V-shape core previously exposed
square-root density through its ROS keys. Thus old `.02/.01` become `.0004/.0001`
in YAML, preserving the actual Q and all benchmark behavior.

Two intentional differences prevent misleading parameter reuse:

- Wall/reflector's `iekf_chi2_gate` is a fixed chi-square score for a fixed residual
  dimension. V-shape uses a changing number of point residuals, so
  `iekf_chi2_gate_sigma` specifies a probability and computes the score with actual
  degrees of freedom. Do not copy the wall's fixed score into this key.
- Reference `iekf_odom_gap_*_velocity_sigma` represents unknown held velocity.
  This implementation uses white pose diffusion during a gap; its density has
  different units and keeps `*_noise_density` in the name.

Not implemented options (adaptive consistency, auto time-offset, online bias or
extrinsic estimation) are not declared merely to imitate the reference YAML.
The node's parameters are startup-only. Use the current names in launch/YAML;
removed old ROS keys are not aliases. `use_sim_time` is set by the launch argument.
