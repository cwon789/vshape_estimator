# ros2-bot-estimator-lidar-vshape

V-shape 도킹 마커에 대한 로봇의 2D 상대 pose를 추정하는 ROS 2 패키지다.
**SE(2) iterated error-state EKF**가 body odometry와 LiDAR의 원래 점–팔 직선
잔차를 직접 결합한다. 검출기가 만든 pose를 독립 측정치로 다시 융합하지 않는다.
디렉터리 이름은 `ros2-bot-estimator-lidar-vshape`, ROS 패키지 이름은
`ros2_bot_estimator_lidar_vshape`다. ROS 2 Foxy / C++17 / 시스템 Eigen을 사용한다.

## 실행

```bash
cd /home/jay/catkin_ws/catkin_1022
source /opt/ros/foxy/setup.bash
colcon --log-base log_vshape build \
  --base-paths src/ros2-bot-estimator-lidar-vshape \
  --build-base build_vshape --install-base install_vshape \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install_vshape/setup.bash
ros2 launch ros2_bot_estimator_lidar_vshape vshape.launch.py use_sim_time:=true
```

실제 로봇에서 ROS wall clock을 쓰면 `use_sim_time:=false`로 실행한다.
기본 설정은 [config/vshape.yaml](config/vshape.yaml)이다. 다른 설정은
`config:=/absolute/path/to/vshape.yaml`로 전달한다. 파라미터는 시작할 때 검증하며
실행 중 변경하지 않는다. `use_sim_time`은 launch 인자로 지정한다.

simple sim의 AMR700D front LiDAR 설정을 기본으로 제공한다:

- scan: `/bot_sensor/lidar_front/laser_scan`, frame `lidar_front`
- odometry: `/bot_sensor/odometer/odometry`, child frame `base_footprint`
- `T_base_lidar`: `(0.362, -0.272, -0.785398163 rad)`
- MiR 참고 V: 한쪽 팔 `150 mm`, 내각 `120°`, 전체 폭 약 `260 mm`, 깊이 `75 mm`
- 물리 높이 `100 mm`, z 범위 `0.213–0.313 m`, pose 높이 `0.263 m`
- 최초 획득 시 예상 marker-in-base: `(2.0, 0.0, pi)`

simple sim에서 로봇을 `(0,0,0)`, VMarker를 `(2,0,pi)`에 배치하면 이 search prior에
해당한다. VMarker의 위치는 꼭짓점이고 내부 모델 +x가 양팔 사이의 열린 방향이다.
도킹 출력은 최초 획득 때 AMR에서 바깥으로 향하는 +x를 선택하므로, 이 정면 배치의
출력은 `x=1.925 m`, `yaw=0`이다. 꼭짓점에서 입구 면 중앙까지의 75 mm는
V 치수에서 자동 계산하므로 사용자가 도킹 오프셋에 따로 더할 필요가 없다. 타깃 치수와 설치 높이의 출처는 [MiR 사양](docs/vendor_spec.md)에 있다. sim의
ROS 연결과 `/clock`을 활성화하고 노드를 실행한다. 로봇 모델이나 LiDAR mount를
바꾸면 YAML의 frame/extrinsic도 함께 맞춰야 한다. 이 노드는 TF에서 extrinsic을
추측하지 않으며 frame이 다른 입력을 거절한다. 원래 wall/reflector의 proprietary
DLC/GPM 명령과는 독립적인 standard-message 노드다.

## 입출력과 상태

| 인터페이스 | 의미 |
|---|---|
| `/vshape/pose` | **단일** `PoseWithCovarianceStamped`, AMR base 기준 V 입구 면 중앙과 outward 도킹 yaw |
| `/vshape/deskewed_points` | 필터와 같은 `deskewScan`으로 계산한 `PointCloud2`, 첫 ray 시각의 base frame |
| `/vshape/diagnostics` | 모드, 입력 age, 거절 사유, NIS, 팔별 점 수, 정보 고유값, 처리 시간 |
| `/vshape/reset` | `std_srvs/Trigger`, 대상 변경/재획득을 위한 명시적 초기화 |

출력 covariance는 ROS `[x,y,z,roll,pitch,yaw]` 순서다. 내부 body right-tangent에서
base의 additive x/y/yaw 좌표로 변환한다. reflector와 같이 첫 획득 때
`heading · [x,y] > 0`인 pi branch를 고르고, 이후의 실제 AMR 회전은 유지한다.
출력 높이는 `marker_height`로 지정한 LiDAR 스캔 평면이며 z 추정값은 아니다.
미추정 z/roll/pitch의 큰 분산은 유지하되, 기본 RViz는 **XY 2σ 타원만** 그린다.
입구 면 중앙은 양 팔 끝점의 평균이며, 모델의 opening 축으로
`depth = arm_length * cos(opening_angle/2)`만큼 꼭짓점에서 이동한 위치다.
이 origin 이동의 yaw lever arm을 포함해 covariance도 변환한다. 내부 필터는
기하 계산을 위해 꼭짓점 좌표계를 그대로 사용한다.
초기화 전에는 pose를 출력하지 않는다. pose는 기본 30 Hz이며 측정이 끊겨도
최근 posterior에서 현재 시간으로 예측한다. 이전 두 pose topic은 제거했다.

움직이는 GUI 데모는 패키지 디렉터리에서 다음과 같이 실행한다:

```bash
bash tools/run_live_demo.sh
```

simple sim과 RViz가 함께 열리고, 작은 실제 치수 V 앞에서 전진·후진·회전을 반복한다.
아래 `AUTO DEMO`를 해제하거나 `Pause drive`로 멈출 수 있다. cyan은 보정 점군,
green은 V 도킹 pose와 XY covariance다. RViz의 Raw scan 항목은 비교용으로 기본 off다.

- `TRACKING`: 양팔 관측을 수용했다.
- `PARTIAL`: 한 팔만 수용했다. 팔 접선 방향의 정보는 추가하지 않는다.
- `PROPAGATION_ONLY`: V 측정 없이 fresh odometry로 예측한다.
- `ODOMETRY_STALE`: twist 유효기간 이후 mean을 유지하고 불확실성을 증가시킨다.
- `UNINITIALIZED`: 아직 ROI 안에서 모호하지 않은 전체 V를 찾지 못했다.

정보와 지연 이력은 한 single-threaded executor가 소유한다. SensorData QoS를 사용하며
입력 scan 크기와 history 수를 제한한다. 시계가 뒤로 가거나 ROS time source가 바뀌면
이력을 비우고 다시 초기화한다. 단순 forward jump는 초기화를 지우지 않는다.

## 모델과 방어 동작

`X = T_VB`, 즉 base 좌표를 marker 좌표로 옮기는 능동 변환을 상태로 둔다.
오차는 `X_true = X_hat Exp(delta)`, `delta=[rho_x,rho_y,yaw]`이다.

```text
모션:       X_next = X Exp([vx,vy,omega] dt)
전이:       Phi = Ad_Exp(-u dt)
측정:       0 = n_arm^T (X T_BL q_L) + noise
Jacobian:   H = n_arm^T R_VB [I, J q_B]
```

모션은 정확한 SE(2) exponential, Q는 continuous density 적분을 사용한다.
ROS 설정은 wall/reflector와 같은 `iekf_process_noise_lin/ang` 이름과 **Q 분산 밀도**
(m²/s, rad²/s)를 받는다. 현재 `0.0004 / 0.0001`은 코어 API의 square-root density
`q_linear=0.02 / q_angular=0.01`과 같다. 과거 std 값 `0.02`를 새 분산 파라미터에
그대로 넣으면 Q가 달라지므로 주의한다.

`iekf_measurement_noise_pos`는 return별 radial σ(m), `iekf_measurement_noise_angle`은
ray별 angular σ(rad)다. `iekf_arm_model_sigma`는 scan 안에서 같은 팔에 공유되는 선 모델
오차 σ(m)이며 고정 calibration 오차를 온라인 추정하지는 않는다. 나머지 이름 매핑과
wall/reflector와 다른 단위는 [파라미터 규약](docs/parameters.md)에 정리했다.

검출은 ROI 내 deterministic RANSAC + 두 직선의 공동 재할당을 사용한다. 유한 팔,
opening, 길이 방향 support, 연속적으로 이어진 긴 벽과 경쟁 후보를 검사한다.
당초 설계의 scan-order line segmentation 대신 이 검출 방식을 선택했으며, 실제
sim clutter/긴 V/원거리 collinear background 회귀 테스트로 검증했다.
검출 pose는 최초 획득과 association에만 사용한다. 초기화 scan은 다시 update하지 않는다.
초기 covariance는 marker-in-base right tangent의 보수적 prior를 inversion adjoint로
변환한다. 출력 covariance는 association이 맞다는 조건부 uncertainty다.

필터는 한 scan에 고정된 prior를 두고 MAP 반복 선형화한다. full innovation covariance로
chi-square gate를 적용하고, outlier에는 Huber 가중을 쓴다. Joseph covariance update와
retraction Jacobian을 적용하며, 미수렴/조건수 이상/비유한 posterior이면 mean과 covariance를
변경하지 않는다. `candidate_score`는 raw 후보 선택 점수, `support_fraction`은 선택한 점의
비율이다. 둘 다 보정된 확률이 아니며 covariance와 구분해서 diagnostics로 제공한다.

0.5초 이상 관측이 끊기면 predicted pose 주위의 제한된 ROI에서 전체 V를 다시 찾고,
모션을 반영한 후보가 3회 연속 일치한 뒤 원래 점 잔차로 보정한다. pose를 후보로 즉시
reset하지 않는다. 동일 형상·치수의 다른 V와의 완전한 모호성은 coarse target prior나
외부 target identity로 해결해야 한다.

scan stamp는 첫 ray의 시간이고 ray offset은 `time_increment`다. odometry는 sample
stamp부터 유효한 previous-sample ZOH다. deskew에는 센서 lever arm과 ray별 body motion을
반영한다. 시간 범위를 덮지 못하면 scan을 거절한다. 늦은 scan/odometry는 보관된
checkpoint로 돌아가 이후 raw scan과 odometry를 모두 재처리한다. 기존 sweep 내부의
새 odometry도 scan을 다시 계산한다. 첫 ray 바로 이전의 늦은 odometry도 초기 ZOH를
바꾸므로 필터와 보정 점군을 함께 갱신한다. 동일 scan을 두 번 융합하지 않는다.

기본 history는 1초/5000 events다. checkpoint로 닫힌 timestamp 구간(완료된 sweep의
support 포함)과 그 이전 입력은 거절한다. 메모리 제한에 걸린 미완료 sweep은 이력을
무한히 늘리는 대신 새 입력을 명시적으로 거절한다. `Tracker::estimate(t)`는 posterior
이후 t의 출력 예측이며 history의 posterior를 변경하지 않는다.

## 재현 가능한 검증

ROS 없는 production core를 빌드하고 모든 단위/계약 테스트를 실행한다:

```bash
cd /home/jay/catkin_ws/catkin_1022/src/ros2-bot-estimator-lidar-vshape
cmake -S . -B build -DVSHAPE_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 tools/validate_simple_sim.py --profile mir_v150 --seeds 150 --adverse-seeds 3 --output reports/mir_v150
# Legacy large synthetic target:
python3 tools/validate_simple_sim.py --profile legacy --seeds 30 --adverse-seeds 3 --output reports/simple_sim
./build/vshape_consistency > reports/consistency/summary.json
```

검증 runner는 기존 `../../simulator/simple-sim`의 **실제 Scanner / VMarker**를 import한다.
자체 ray caster나 정답 팔 label로 tracking하지 않는다. truth는 scene 생성과 외부 점수
계산에만 쓴다. Python NumPy/SciPy/Matplotlib/PyYAML과 로컬 simple sim이 필요하다.
실패 seed를 제외하지 않으며 기준 미달이면 exit code 1을 반환한다. 원래 simulator와
wall/reflector 소스는 수정하지 않았다.

[소형 MiR 결과](reports/mir_v150/report.md), [소형 JSON](reports/mir_v150/summary.json),
[기존 대형 synthetic 결과](reports/simple_sim/report.md), [기존 JSON](reports/simple_sim/summary.json),
[오차 그림](reports/simple_sim/errors.png), [ROS live 검증](reports/ros_validation.md),
[Gaussian consistency 결과](reports/consistency/summary.json)를 제공한다.
각 trial에 raw stdin scan, JSON 출력, 동일 시작 posterior의 odometry-only 입력/출력,
truth CSV, config/seed를 보관하며 provenance에 source/binary SHA256을 저장한다.

**기존 대형 synthetic 타깃** 검증: 20 Hz scan의 54개 trial(30 nominal seeds + 8종 adverse case 각 3 seeds)을
통과했다. nominal pooled RMSE는 **4.19 mm / 0.139°**, 같은 시작 posterior의
odometry-only는 **18.91 mm / 0.483°**였다. dropout·partial·clutter/outlier·지연 입력·
stale odometry·swept·무잡음 local recovery·overlong V 거절을 검사한다.
처리 시간 수치는 해당 report의 full scan latency와 filter update latency를 구분해서 본다.
일반 OS에서의 측정값이며 hard real-time 보장은 아니다.

이 대형 타깃의 결과를 소형 MiR 타깃의 정확도로 해석하면 안 된다. 소형 MiR 타깃은
150 nominal seeds + 24 adverse trials를 별도로 검사했다. 150/150 nominal seeds가
0.15초 이내 획득됐다. 요청한 **AMR 기준 개구면 중앙 포즈**의 **20 mm / 1° RMSE
기준은 nominal 140/150**이 통과했다. pooled RMSE는 **2.79 mm / 0.606°**이며,
실패 10건은 모두 0.7 m 조건의 yaw 기준 초과였다. adverse 24건은 각 시나리오의
검증 기준을 통과했다. odometry가 끊기는 경우까지 지속적인 위치 정확도를 보장하는
의미는 아니다. 기존 내부 상태(robot-in-apex-marker) 점수도 그대로 보존했으며,
내부 점수의 nominal 110/150, adverse 22/24와 공개 포즈의 점수를 구분한다.
실패를 숨기거나 기준을 낮추지 않았으며 benchmark는 exit code 1을 반환한다.

| 초기 LiDAR→V 거리 | 개구면 중앙 포즈 통과 / 시험 | 내부 상태 통과 / 시험 |
|---|---:|---:|
| 0.7 m | 15 / 25 | 15 / 25 |
| 1.0 m | 25 / 25 | 25 / 25 |
| 1.5 m | 25 / 25 | 24 / 25 |
| 2.0 m | 25 / 25 | 20 / 25 |
| 2.5 m | 25 / 25 | 12 / 25 |
| 3.0 m | 25 / 25 | 14 / 25 |

이는 고정 front mount, 10 mm 독립 range noise의 synthetic 실험 결과이지 보장된 운용
범위나 제조사의 완성 로봇 도킹 정확도가 아니다. 공개 포즈의 최대 nominal translation
RMSE는 3.72 mm, yaw RMSE는 1.33°였다. 내부 상태의 최대 translation RMSE는
40.9 mm였다. 같은 scan에서 odometry 오차를 없애도 일부 실패가
남았다. 단순한 visibility association 교체도 악화돼 적용하지 않았다. 근거와 대조
실험은 [추가 진단](reports/mir_v150_diagnostics)에 보존했다.

별도의 Gaussian fixed-association 실험은 200개의 독립 seed를 사용한다. 평균 NEES
**2.747**(95% 구간 2.670–3.349), NIS **47.208**(46.652–49.367), pose의 95% coverage
**96.5%**였다. 이 실험은 instantaneous scan의 한 번 update를 검증하며 robust association이나
swept scan 전체의 consistency를 증명하지 않는다.

sanitizer 재현:

```bash
cmake -S . -B /tmp/vshape-sanitized -DVSHAPE_BUILD_ROS=OFF \
  -DVSHAPE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG='-O1 -g0'
cmake --build /tmp/vshape-sanitized -j1
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/vshape-sanitized --output-on-failure
```

## 범위와 한계

정적인 알려진 치수의 V, 고정 LiDAR extrinsic, planar robot motion을 전제로 한다.
IMU bias·wheel scale·시각 offset·extrinsic·V 치수의 온라인 calibration, 전역 SLAM,
proprietary mission command 호환성은 포함하지 않는다. ROS odometry의 twist만 쓰며
odometry pose나 simulator truth pose를 측정치로 사용하지 않는다.

swept scan에서는 retained point 수를 반영한 joint covariance 상한의 diagonal envelope로
공유 deskew 오차가 점 수에 따라 사라지는 것을 막는다. 하지만 해당 scan이 사용한 후속
process noise와 출력 propagation 사이의 cross-correlation은 근사다. 실제 rolling scan
consistency, 지면 요철/roll/pitch, 실센서 calibration 오차와 현장 실패율은 추가 검증이
필요하다. 현재 결과는 보존된 synthetic 조건에서의 결과다.
