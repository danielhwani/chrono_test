# Chrono Simple Vehicle

PyChrono(Project Chrono core API, `chrono.vehicle` 모듈 없이 `ChBody`/`ChLink` 프리미티브만으로 직접 조립한) 기반의 간단한 차량 multibody dynamics 데모.

## 환경

- conda env `chrono` (Python 3.12), `projectchrono` 채널의 PyChrono 10.0.0 (core + irrlicht) 설치됨
- `pynput` 설치됨 (`drive_vehicle.py`의 전역 키보드 입력용)

```bash
conda activate chrono
cd /home/daniel/chrono_test
```

## 파일

| 파일 | 설명 |
|---|---|
| `simple_vehicle.py` | 차량/지면 조립(`make_vehicle`)과 자동(스텝 조향) 주행 데모. CSV 로그 생성. |
| `drive_vehicle.py` | 같은 차량을 화살표/WASD 키로 직접 조종하는 버전. |
| `slip_demo.py` | 한쪽 바퀴 저마찰 슬립 → 디퍼렌셜 개입 과정을 보여주는 6륜 전용 데모. |
| `plot_results.py` | `vehicle_log.csv`를 읽어 궤적/속도/조향-요/서스펜션 그래프(`vehicle_results.png`) 생성. |
| `compare_tire_models.py` | 같은 조향 시나리오를 rigid/empirical 두 타이어 모델로 각각 헤드리스 실행하고 겹쳐서 그래프(`tire_model_compare.png`)로 비교. |

## 모델 구조

- 각 바퀴 코너: 섀시 — (수직 `ChLinkLockPrismatic` + `ChLinkTSDA` 스프링-댐퍼) — 업라이트
- 조향축(전륜, 4륜/6륜 공통): 업라이트 — (`ChLinkMotorRotationAngle`, Z축 회전) — 너클 — (`ChLinkLockRevolute`, Y축 스핀) — 휠
  - 기본은 **좌우 바퀴에 동일한 각도**를 적용하는 평행 조향입니다.
  - `--ackermann` 옵션을 주면 `ackermann_wheel_angles_deg()`가 명령 각도를 좌우 실제 각도로 변환해서 적용합니다(안쪽 바퀴가 더 많이 꺾임, 두 바퀴의 조향축 연장선이 같은 회전중심에서 만나 타이어 스크럽이 없어짐). `make_vehicle`이나 기본 동작은 건드리지 않고, 조향 입력을 적용하는 방식만 바뀌는 순수 함수입니다.
- 구동축: 위와 동일하지만 조향축 대신 `ChLinkMotorRotationTorque`로 토크 인가
- 지면: 500m×500m 콘크리트 텍스처 박스, `ChCollisionSystem.Type_BULLET` 필요 (안 켜면 접촉 없이 그냥 뚫고 낙하함)
- 6륜(`--six-wheel`): 전축(F, 조향)+중간축(M)+후축(R, 둘 다 구동)인 3축 트럭 레이아웃. `make_vehicle`이 축 개수에 무관하게 동작하도록 일반화되어 있어 4륜/6륜이 같은 코드 경로를 씀.
- 좌우 디퍼렌셜: `apply_differential()` — 구동축별로 `ChLinkMotorRotationTorque.GetMotorAngleDt()`로 좌우 속도차를 읽어 헛도는 쪽 토크를 줄이고 반대쪽으로 몰아줌(진짜 기어 결합이 아니라 속도차 감지 기반 소프트웨어 재분배).
- 바퀴에는 회전 확인용 노란 마커(반지름 밖으로 살짝 튀어나온 박스)가 붙어 있어, 빨리 도는지 눈으로 구분 가능.
- 타이어 모델(`--tire-model {rigid, empirical}`): 기본은 `rigid`(Bullet의 Coulomb 마찰 접촉, 지금까지 써온 방식). `empirical`은 바퀴-지면 마찰을 0으로 낮춰서 Bullet은 수직 반력만 담당하게 하고, 대신 `apply_tire_forces()`가 매 스텝 슬립비/슬립각을 직접 계산해서 선형-then-포화(friction circle) 힘 법칙으로 종/횡방향 힘을 얹어줍니다. 개념적으로 TMEASY와 비슷하지만 직접 만든 단순화 모델입니다.
  - 진짜 `ChTMeasyTire`(Chrono::Vehicle 정식 클래스)는 `ChWheel.Initialize()`가 `ChChassis`(→ `ChVehicle`)를 요구해서, 지금처럼 `ChBody`/`ChLink`로 직접 조립한 차량에는 못 붙입니다 — Chrono::Vehicle 클래스 체계로 차량을 통째로 다시 지어야 합니다. 그래서 기존 구조를 유지하는 자체 슬립 기반 모델로 대신했습니다.
  - Fz(수직하중)는 서스펜션 스프링 힘 대신 `wheel.GetContactForce()`(엔진이 실제로 계산한 접지력)를 씁니다 — 스프링 힘은 구동 반작용 토크 때문에 인장 방향으로 틀어지는 경우가 있어 하중 추정치로 부정확했습니다.
- 지형(`--terrain {flat, bumps}`): 기본은 `flat`(기존 평평한 지면). `bumps`는 그 위에 반쯤 파묻힌 원통(지름 대비 낮게 튀어나오도록) 5개를 x=8m부터 6m 간격으로 일렬 배치해서 과속방지턱처럼 통과하게 만듭니다. 둘 다 `Fixed` 바디라 지면 박스와 겹쳐도 문제없음.

## 실행

```bash
# 자동 주행 데모 (헤드리스: CSV만 생성)
python simple_vehicle.py

# 3D 뷰로 보기
python simple_vehicle.py --irrlicht

# 조향 파라미터 조절 (목표각/시작시각/램프시간)
python simple_vehicle.py --irrlicht --steer-deg 25 --steer-start 1.5 --steer-ramp 0.8

# 6륜 트럭 레이아웃
python simple_vehicle.py --irrlicht --six-wheel

# 애커먼 조향 기하 (좌우 바퀴 각도를 따로 계산; --six-wheel과도 조합 가능)
python simple_vehicle.py --irrlicht --ackermann

# 슬립 기반 타이어력 모델 (다른 옵션들과 자유롭게 조합 가능)
python simple_vehicle.py --irrlicht --tire-model empirical

# rigid vs empirical 비교 그래프 (헤드리스, 같은 조향 시나리오로 둘 다 돌려서 겹쳐 그림)
python compare_tire_models.py --steer-deg 30 --steer-start 1.5 --steer-ramp 0.8 --time 6

# 요철 구간 통과 (조향 없이 직진으로; x=8m부터 시작하니 --time을 충분히 줘야 다 지나감)
python simple_vehicle.py --irrlicht --terrain bumps --steer-deg 0 --time 18

# 결과 그래프
python plot_results.py

# 키보드 조종 (↑↓←→ 또는 WASD, Space=정지, q/Esc=종료 — 어느 창에 포커스가 있든 동작)
python drive_vehicle.py
python drive_vehicle.py --six-wheel
python drive_vehicle.py --ackermann
python drive_vehicle.py --tire-model empirical
python drive_vehicle.py --terrain bumps

# 6륜 슬립 → 디퍼렌셜 개입 시각화
python slip_demo.py
python slip_demo.py --switch-time 4.0
```

## 알려진 이슈/한계

- **기본은 평행 조향**: `--ackermann`을 주지 않으면 좌우 앞바퀴가 항상 같은 각도로 꺾임. `--ackermann`을 켜면 실제 좌우 각도가 달라짐(CSV의 `steer_FL_deg`/`steer_FR_deg`, `plot_results.py`의 "Steer angle / chassis yaw" 그래프에서 확인 가능).
- **디퍼렌셜은 소프트웨어 방식**: 실제 기어 커플링(캐리어 각속도 = 좌우 평균)을 강제하지 않고, 속도차를 감지해서 토크를 재분배하는 제어 로직임.
- **회전 시 전복 가능**: 무게중심 높이(1.19m)가 트랙폭(1.5m) 대비 높은 편이라, 급선회 중 계속 가속하면 전복함 (물리적으로는 타당한 현상).
- Irrlicht 창은 `DISPLAY` 환경변수가 유효한 X11 세션이 있어야 뜸. `drive_vehicle.py`는 `pynput`으로 X서버 레벨 전역 키 입력을 사용하므로 터미널 포커스와 무관하게 동작함.

## 추후 아키텍처 계획 (진행 중)

지금은 `simple_vehicle.py` 하나에 차량 모델(조립)·제어(조향/디퍼렌셜/타이어력)·동역학 스텝핑이 다 얽혀 있음. 이걸 3단(모델/동역학/제어)으로 분리하는 게 목표:

- **모델**: URDF로 링크-조인트 토폴로지(+관성)만 분리 (아직 미착수). URDF는 순수 기구학 트리라 스프링/디퍼렌셜/타이어력 같은 로직은 못 담음 — 그건 결국 동역학 레이어에 남음.
- **동역학**: FMU로 래핑.
  1. **`pythonfmu`** (진행 중, `fmu/` 디렉터리): 순수 Python이라 Chrono/C++을 새로 빌드할 필요 없음. 실행 쪽에 Python 런타임이 있어야 하는 게 유일한 제약.
  2. Chrono 공식 `chrono_fmi` export 모듈 자체는 conda `pychrono` 바이너리에 없음. 다만 **"Chrono를 C++에서 쓰려면 소스 빌드가 필요하다"는 건 틀렸다는 게 나중에 밝혀짐** — conda `chrono` env 안에 Chrono의 C++ 헤더/라이브러리(`libChrono_core.so` 등)가 이미 들어 있어서, 그걸 직접 링크하는 손수 FMI2 C++ 래퍼로 우회 성공함. 자세한 내용은 아래 "완성: 네이티브 FMU 안에서 진짜 ChSystemNSC 호출" 섹션 참고.
- **제어**: `ros_control`(ROS1, EOL)이 아니라 **`ros2_control`**로 방향을 잡음 — 이 머신엔 ROS2 Humble이 이미 설치돼 있음(`/opt/ros/humble`). 컨트롤러는 **`ackermann_steering_controller`**로 확정(설치된 `steering_controllers_library`의 실제 파라미터 `front_wheels_names`/`rear_wheels_names`와 공식 문서를 직접 확인 — 처음엔 `bicycle_steering_controller`를 생각했다가, 우리 차량의 실제 FL/FR/RL/RR 구조를 그대로 못 살린다는 지적을 받고 바꿈). 구상: `command_interfaces`(전륜 조향각 2개, 후륜 트랙션 1개) → 커스텀 `hardware_interface::SystemInterface` 플러그인(`ChronoFmuSystemInterface`) → 같은 프로세스 안에서 `fmi2SetReal()`→`fmi2DoStep()`→`fmi2GetReal()` 직접 호출 → `vehicle_native.fmu`. `SystemInterface`는 그냥 `pluginlib`이 로드하는 C++ 클래스라, `fmu_driver.c`가 이미 하고 있는 `dlopen`+FMI2 직접 호출을 거의 그대로 재사용 가능. 4WD 처리 방식은 아래 "4WD (전축 구동) 추가" 섹션 참고 — 아직 미착수인 건 `ChronoFmuSystemInterface` 자체.

**진행 상황**: `fmu/` 아래에 pythonfmu 툴체인 자체를 검증하는 토이 FMU(`free_fall_fmu.py`, 자유낙하 적분기)를 먼저 만들어 빌드→로드→시뮬레이션이 실제로 동작하는지 확인 완료. (이후 이 틀에 실제 차량 동역학을 넣는 작업까지 완료 — "## 실제 차량을 FMU로 감싸기" 섹션 참고. 아래부터 여기까지는 전부 그 전 단계, 토이 바운싱볼 모델로 툴체인 자체를 검증하던 기록.)

```bash
conda activate chrono
pip install pythonfmu fmpy   # 이 env엔 아직 없어서 따로 설치 필요
cd fmu
pythonfmu build -f free_fall_fmu.py -d build   # build/FreeFall.fmu 생성
python validate_fmu.py                          # fmpy로 로드+시뮬레이션, 해석해와 비교
python plot_free_fall.py                        # h(t)/v(t) 그래프 (free_fall.png)
python render_free_fall.py                      # Irrlicht 실시간 렌더링 (차량 데모와 같은 스타일)
```
`render_free_fall.py`는 `fmpy.fmi2.FMU2Slave`로 FMU를 직접 스텝핑하면서, 그 결과값(`h`)으로 공의 위치만 움직이는 식 — Chrono는 여기서 물리 계산을 전혀 안 하고 순수 시각화 용도로만 씀. 공이 좌우로는 안 움직이고 수직으로만 빠르게 낙하해서, 차량 데모처럼 체이스캠을 쓰면 카메라가 지면 아래로 파고들어 오히려 어색해짐 — 그래서 고정 카메라로 프레이밍함.
`h`(높이)는 스텝 크기에 따른 1차 적분기 이산화 오차가 있어서(`output_interval=0.5`일 땐 오차 커짐, `1e-4`로 줄이면 0.001m 이내로 수렴), `v`(속도)는 스텝 크기와 무관하게 해석해와 항상 정확히 일치 — 둘 다 버그가 아니라 예상된 수치적 특성이며, `pythonfmu` co-simulation FMU는 `do_step`이 `output_interval` 간격으로 호출된다는 것도 이때 확인됨.

`free_fall_fmu.py`는 바닥을 그냥 뚫고 지나가서(의도된 "가장 단순한" 툴체인 검증용 모델), 바닥 충돌 + 반발계수를 더한 `bouncing_ball_fmu.py`(고전적인 FMI 레퍼런스 "BouncingBall" 모델과 동일한 형태)를 별도로 추가함:
```bash
pythonfmu build -f bouncing_ball_fmu.py -d build
python validate_bouncing_ball.py     # 바닥 비침투 + 바운스마다 높이가 e^2 비율로 줄어드는지 확인
python render_bouncing_ball.py                        # 실시간 렌더링 (e=0.7 기본값)
python render_bouncing_ball.py --restitution 0.9 --time 12
```
`e`(반발계수)는 FMU의 입력 변수라 모델 코드를 안 건드리고 `fmu.setReal(...)`로 실행할 때 바꿀 수 있음. 튕길 때마다 봉우리 높이가 `e²` 배로 줄어드는 게 물리적으로 맞는 동작이라, `validate_bouncing_ball.py`는 단일 해석해 대신 이 비율을 체크함(예: e=0.7 → e²=0.49, 실측 0.4899~0.4900로 거의 정확히 일치 확인). 공은 이제 바닥과 반지름만큼 띄워서 그리므로(중심이 아니라 표면이 바닥에 닿게) 시각적으로도 파묻히지 않음.

**실시간성 벤치마크** (`benchmark_realtime.py`): `render_*.py`는 `ChRealtimeStepTimer.Spin()`으로 일부러 실제 시간에 맞춰 속도를 늦추기 때문에, 전체 실행 시간을 재도 "진짜 계산 여력"은 안 보임(페이싱이 여유분을 그냥 재워버림). 그래서 페이싱을 빼고 (1) 순수 `do_step` 루프만, (2) 같은 루프에 Irrlicht 렌더링까지 얹은 것, 둘을 각각 최대 속도로 돌려서 실시간 배율(RTF = 시뮬레이션 시간/실제 걸린 시간)을 비교함:
```bash
python benchmark_realtime.py --sim-time 5
```
결과(BouncingBall.fmu 기준): **순수 동역학 394x**, **렌더링 포함 3.68x** — `do_step` 자체는 마이크로초 단위로 거의 공짜고, 렌더링(`Render()` 호출)이 비용의 대부분을 차지함. 즉 **동역학의 실시간 여력을 평가할 땐 렌더링을 반드시 빼고 측정해야 함** — 섞어서 재면 왜곡된 결론이 나옴. 지금은 모델이 가벼워서(자유낙하/바운스, 강체 1개) 394x라는 압도적인 수치가 나온 거고, 나중에 실제 차량(강체 여러 개+조인트+접촉)을 FMU로 감싸면 이 여유가 크게 줄어들 것이므로 그때 이 스크립트를 그대로 재사용해서 다시 잴 것.

**페이싱 자체가 실제 시간과 맞는지도 별도로 확인**(`--pacing on`): 위 RTF는 "페이싱 없이 최대 속도"였고, `render_*.py`가 실제로 쓰는 `Spin()` 페이싱을 켠 채로 목표 시뮬레이션 시간과 실제 걸린 시간이 맞아떨어지는지 따로 검증함:
```bash
python benchmark_realtime.py --pacing on --sim-time 5
```
결과: **헤드리스(렌더링 없음)는 +0.00% — 완벽히 동기화.** 하지만 **렌더링을 얹으면 +10~11% 느려짐**(목표 5.000s인데 실제 5.51~5.54s 걸림). `vis.Run()` 자체 비용은 무시할 수준(2500회 호출에 5ms)이라 원인이 아니었고, `Chrono`의 `ChRealtimeStepTimer.Spin()`이 렌더링처럼 스텝 사이에 끼어드는 지연을 제대로 보정 못 해서 누적되는 것으로 보임(정확한 내부 동작은 미확인, Chrono가 컴파일된 바이너리라 더 파고들려면 소스 레벨 분석 필요). **이건 이 FMU 벤치마크에 국한된 문제가 아니라, `simple_vehicle.py`/`drive_vehicle.py` 등 지금까지 만든 모든 실시간 렌더링 데모에 동일하게 적용되는 특성**임 — 지금까지 "실시간처럼 보인다"고만 확인했지 이렇게 정량적으로 재본 적은 없었음.

주의: `--render both`는 헤드리스/렌더링 두 측정을 **별도 프로세스로 격리**해서 돌림 — 한 프로세스에서 FMU/Irrlicht 세션을 연달아 두 번 만들면 정리(cleanup) 과정에서 `corrupted double-linked list`로 크래시하는 걸 확인해서 이렇게 고침(테스트 중 실제로 발생시켜 확인함).

**한눈에 그래프로 보기** (`plot_realtime_benchmark.py`): 시간에 따라 (시뮬레이션 시간, 실제 경과 시간)을 계속 기록해서, 헤드리스/렌더링 두 곡선을 "완벽 동기화(y=x)" 기준선과 함께 그림. 오른쪽 패널은 `wall - sim` 편차 자체를 시간축으로 그려서 얼마나 벌어지는지 누적 추세를 바로 보여줌:
```bash
python plot_realtime_benchmark.py --sim-time 5
```
결과(`realtime_pacing.png`): 헤드리스는 처음부터 끝까지 편차 0에 딱 붙어있고, 렌더링 포함은 시작부터 약 0.11s 앞서 벌어진 채 이후 거의 선형으로 계속 커져서 5초 뒤엔 약 0.48~0.58s 차이로 끝남 — 초반의 계단형 점프(예: t≈1.1s 부근)는 아마 첫 바운스 등 이벤트성 비용과 관련된 것으로 보임(원인 미확정).

**RT 커널 vs 일반 커널 비교** (`benchmark_rt_jitter.py` + `plot_rt_comparison.py`, 렌더링 제외): 이 머신은 PREEMPT_RT 커널(`5.15.0-1112-realtime`)로 부팅돼 있고, 일반 커널(`6.8.0-138-generic`)도 같이 설치돼 있음. `benchmark_realtime.py --pacing on`이 재는 "누적 편차"와 달리, 여긴 **루프 한 스텝 한 스텝의 주기 자체를 전부 기록**해서 퍼센타일/최악값(tail latency)을 봄 — PREEMPT_RT가 실제로 개선하는 건 평균이 아니라 이런 드문 큰 지연(스케줄링 스톨)이기 때문:
```bash
python benchmark_rt_jitter.py --sim-time 15   # 지금 부팅된 커널 기준, rt_results/<커널명>.json 저장
python plot_rt_comparison.py                   # rt_results/ 안의 모든 결과를 히스토그램으로 겹쳐 그림
```
**RT 커널 실측 결과** (스텝 목표 2000μs, n=7500): mean +0.5μs, p50 +0.0μs, p95 +1.2μs, p99 +3.2μs, p999 +66.9μs, **max +1693.4μs**(목표의 1.85배) — 2배 초과 0건. 즉 거의 대부분 목표에 딱 맞고, 아주 드물게 한 번씩 ms 단위로 튀는 정도.

**일반 커널과 비교하려면** (재부팅 필요 — 반드시 직접 하실 것, 이 세션은 재부팅되면 끊김):
1. `6.8.0-138-generic`으로 재부팅
2. `conda activate chrono && cd ~/chrono_test/fmu && python benchmark_rt_jitter.py --sim-time 15`
3. 다시 RT 커널로 재부팅해도 되고, 아니면 그 상태에서 `python plot_rt_comparison.py`만 실행 — `rt_results/`에 두 커널 결과가 다 쌓여있으면 자동으로 같이 그려짐(현재 코드는 이미 이 두 파일을 처리하도록 되어 있고, 지금은 RT 결과 하나만 있어서 그래프도 한 종류만 나옴)

**"RT"는 사실 두 개로 나뉨** — 위는 **커널** 레벨(재부팅 필요) 비교였고, 별개로 **프로세스 스케줄링 정책**(SCHED_OTHER ↔ SCHED_FIFO)은 재부팅 없이 즉시 켜고 끌 수 있음(`os.sched_setscheduler`, 이 계정은 `ulimit -r`=99라 root 없이도 가능):
```bash
python benchmark_rt_jitter.py --sim-time 15 --sched fifo --rt-priority 10
python plot_rt_comparison.py
```
**실측 결과 (같은 RT 커널, SCHED_OTHER vs SCHED_FIFO prio=10)** — 반직관적인 결과가 나옴:

| 지표 | SCHED_OTHER(기본) | SCHED_FIFO(prio 10) |
|---|---|---|
| p50 | +0.0μs | +0.0μs |
| p95 | +1.2μs | **+0.7μs (더 좋음)** |
| p99 | +3.2μs | +10.2μs |
| p999 | +66.9μs | **+47,843μs (47.8ms)** |
| max | +1,693μs (1.85배) | **+52,515μs (26배)** |
| 2배 초과 | 0건 | 14건 (0.187%) |

평상시(p50~p95)는 SCHED_FIFO가 오히려 살짝 더 좋아지는데, **최악의 경우(tail)는 30배 가까이 나빠짐**. 이 머신이 GNOME Shell·브라우저 등이 같이 떠 있는 일반 데스크톱이라(격리된 RT 전용 머신이 아님), 우선순위 10 정도로는 시스템의 다른 스레드와 충돌해서 드물게 큰 스톨이 생기는 것으로 보임(정확한 커널 레벨 원인은 미확인 — 우선순위 역전, 메모리 할당/GC 중 발생한 페이지 폴트가 더 급한 커널 스레드에 밀리는 경우 등을 의심 중). 실제 RT 배포에서 `isolcpus`/`taskset`으로 코어를 격리하고 커널 스레드 우선순위까지 같이 신경 쓰는 이유가 이런 것 — 프로세스만 SCHED_FIFO로 올린다고 공짜로 좋아지는 게 아님.

`cgroup v2`의 `cpuset.cpus.partition=isolated`로 재부팅 없이 코어를 격리해보려는 시도도 해봤으나(root 필요, 이 세션엔 비밀번호 없는 sudo가 없고 `tty_tickets` 때문에 사용자 터미널의 sudo 인증도 공유가 안 됨), 잘 안 돼서 보류함 — `rt_isolated`라는 빈 cgroup만 남아있을 수 있음(`sudo rmdir /sys/fs/cgroup/rt_isolated`로 정리 가능).

**후속 검증 완료 (60초, n=30000, Python·C++ 동일 조건)**: 표본을 늘려서 Python(`benchmark_rt_jitter.py`)과 C++(`bouncing_ball --paced 60 0.002 [fifo 10] --json-out ...`)을 SCHED_OTHER/SCHED_FIFO 각각 재측정함 — `bouncing_ball.cpp`도 이때 `--json-out`을 추가해서 `rt_results/*.json`에 파이썬과 같은 포맷으로 저장하고 `plot_rt_comparison.py`로 넷 다 겹쳐 그림(`rt_comparison.png`):

| 지표 | Python OTHER | C++ OTHER | Python FIFO10 | C++ FIFO10 |
|---|---|---|---|---|
| p99 | +2.8μs | +0.6μs | +2.7μs | +0.8μs |
| p999 | +58.2μs | +10.9μs | +48,055.6μs | +47,885.1μs |
| **max** | +11,090.4μs | +40.7μs | +53,231.7μs | +51,404.9μs |
| 2배 초과 | 1건(0.003%) | 0건 | 59건(0.197%) | 57건(0.190%) |

**결론: "C++이 확실히 우위"는 아니었음.** SCHED_OTHER끼리, SCHED_FIFO끼리 비교하면 Python과 C++이 거의 구분 안 될 정도로 비슷함(FIFO의 max는 둘 다 5.1만μs대, 발생 빈도도 0.19%대로 거의 일치) — 이전에 봤던 "언어에 따라 방향이 다르다"는 건 작은 표본(2500~7500) 노이즈였던 게 확인됨. `yield()` 기반 페이싱으로 고친 뒤에는 **페이싱 지터가 언어 문제가 아니라 순수 OS/스케줄러 레벨 현상**이라는 게 이번 큰 표본으로 재확인됨. (참고: 이번 라운드의 C++ OTHER는 우연히 큰 스톨을 안 만나서 max가 특히 깨끗하게 나왔음 — run-to-run 변동이 있다는 것도 그대로 보여주는 사례.)

C++이 여전히 명확히 이기는 영역은 **원시 계산 속도**뿐임(`--bench` 기준 800배 이상, 이건 페이싱과 무관하게 항상 성립). "실시간 페이싱 정확도"는 이제 두 언어가 동등하다고 보는 게 맞음.

## 실제 차량을 FMU로 감싸기 (fmu/chrono_vehicle_fmu.py)

지금까지의 모든 FMU/FMI 작업(pythonfmu, 네이티브 C/C++, Modelica 상호운용, 페이싱/지터 도구)은 전부 장난감 바운싱볼 모델로 툴체인 자체를 검증하는 단계였음. 이번이 그 인프라를 실제로 써먹는 첫 케이스 — `simple_vehicle.py`의 `make_vehicle()`을 pythonfmu로 감쌈(코드 재사용이 빠르고, `--irrlicht` 데모처럼 이미 검증된 함수들을 그대로 부르기만 하면 돼서 pythonfmu를 먼저 선택함 — 네이티브 C++ 경로는 서스펜션/디퍼렌셜/타이어력/조향까지 전부 새로 C++로 짜야 해서 작업량이 훨씬 큼).

**구조적 설정 vs 실시간 입력을 나눔**: 6륜/애커먼/타이어모델/지형처럼 차체 자체를 바꾸는 옵션은 FMI2 `parameter`(초기화 중에만 설정 가능, `Boolean`)로, 조향각/구동토크처럼 매 스텝 바뀌는 값은 `input`(`Real`)으로 노출:

```
parameter (Boolean): six_wheel, ackermann, empirical_tire, bumps_terrain
input     (Real):    steer_deg, drive_torque
output    (Real):    chassis_x/y/z, roll/pitch/yaw_deg, speed_mps, steer_FL/FR_deg
```

`simple_vehicle.py`의 `main()`이 CLI 옵션으로 하던 걸 그대로 매핑한 것 — 다만 `main()`의 `steer_angle_deg()` 램프와 고정 `DRIVE_TORQUE`는 그 스크립트 자체의 데모 편의였던 거라, FMU에서는 이 두 값을 매 스텝 마스터가 직접 넣어주는 진짜 `input`으로 바꿈(그래야 나중에 ros_control 같은 외부 제어기가 실제로 명령을 내릴 수 있음).

빌드 시 `simple_vehicle.py`를 "project file"로 같이 넘겨서 FMU 리소스 안에 번들함(pythonfmu가 지원하는 기능):
```bash
cd fmu
pythonfmu build -f chrono_vehicle_fmu.py -d build ../simple_vehicle.py   # build/ChronoVehicle.fmu
python validate_vehicle_fmu.py
```

**빌드 중 걸린 것 하나**: pythonfmu의 빌드 과정 자체가 FMU 클래스를 알아내려고 스크립트를 직접 import함 — 이때는 아직 패키징 전이라 `simple_vehicle.py`가 (FMU 리소스 안이 아니라) 저장소 루트에 그대로 있음. 반면 실제 FMU가 나중에 실행될 때는 리소스 폴더 안에 번들되어 옆에 있음. 두 시점의 경로가 달라서, `sys.path`에 스크립트 자신의 디렉터리와 그 상위 디렉터리를 둘 다 넣어야 양쪽 다 import가 됨.

**검증 (`validate_vehicle_fmu.py`)**: 같은 스텝-조향 시나리오를 (1) `make_vehicle()`을 직접 불러 도는 참조 루프와 (2) FMU를 fmpy로 구동한 루프, 두 가지로 돌려서 최종 섀시 위치/자세를 비교 — **완전히 일치**(`chassis_x/y/z`, `yaw_deg` 전부 diff `0.00e+00`). 결정론적 솔버라 당연한 결과지만, FMI2로 감싸는 과정 자체가 물리에 아무 영향을 안 줬다는 걸 직접 확인한 것.

`ackermann+six_wheel`, `empirical_tire+bumps_terrain` 파라미터 조합도 각각 따로 스모크 테스트 완료 — 전부 정상 동작(FL/FR 조향각이 애커먼답게 갈라짐, 요철 지형 위에서도 정상 주행).

네이티브 C++ 포팅은 아래 "네이티브 C++ 차량 (fmu/cpp/native_vehicle_fmu/)" 섹션에서 이어서 완료함.

## C++ 버전 (fmu/cpp/)

같은 바운싱볼 모델을 **의존성 없는 순수 C++**로도 포팅함 — Python 버전(`bouncing_ball_fmu.py`)의 물리 로직 자체가 애초에 PyChrono 없이 스칼라 수식(중력 적분 + 바닥 반사)뿐이었어서, C++ 이식도 Chrono 없이 가능함. FMU로 감싸서 구동 측까지 C++로 가는 것(실시간성 향상이 목적)의 첫 단계 — 렌더링(Irrlicht)은 아직 시도 안 해서 보류 중. (참고: "Chrono를 C++에서 쓰려면 소스 빌드가 필요하다"는 가정 자체는 아래에서 틀린 것으로 확인됨 — conda env에 `libChrono_irrlicht.so`도 이미 있어서 렌더링 쪽도 같은 방식으로 될 가능성이 있으나 미검증.)

```bash
cd fmu/cpp
g++ -O2 -o bouncing_ball bouncing_ball.cpp
./bouncing_ball --csv 8.0 0.002      # time,h,v를 stdout에 CSV로 출력
./bouncing_ball --bench 5.0 0.002    # 순수 계산 속도 벤치마크
```
**검증**: 바닥 비침투(min h=0.0), 봉우리 높이 감쇠 비율 0.485~0.488 (Python과 동일하게 e²=0.49에 근접) — 물리적으로 Python 버전과 일치.

**속도 비교** (5초 시뮬레이션, dt=2ms):

| 구현 | 스텝당 시간 | 실시간 배율(RTF) |
|---|---|---|
| Python (fmpy로 FMU 구동) | ~1,900~4,000 ns | 394x |
| 순수 C++ (의존성 없음) | **2.38 ns** | **840,336x** |

**800~1600배 차이.** Python 경로는 fmpy의 ctypes 마샬링 오버헤드 + pythonfmu로 빌드된 FMU가 내부적으로 Python 인터프리터를 다시 호출하는 이중 비용이 있는데, 순수 C++엔 둘 다 없음. 지금은 모델이 스칼라 연산 3개뿐이라 이 정도 배율까지 나온 거고, 나중에 실제 차량처럼 무거운 모델이면 절대 격차는 커지되 RTF 배율 차이는 좁혀질 것.

### 네이티브 FMU (fmu/cpp/native_fmu/) — Python 완전 배제

FMI2 C API(`fmi2Instantiate`, `fmi2DoStep`, `fmi2GetReal` 등 표준 함수 전부)를 직접 구현해서 진짜 네이티브 `.fmu`를 만듦. 공식 `fmi2Functions.h`는 레포에 없어서(벤더링 안 함) 스펙에 맞춰 필요한 타입/함수 시그니처만 손으로 선언함. 모델은 지금 단계에선 차량이 필요 없어서 이미 검증된 바운싱볼 그대로 재사용(입출력 변수도 `bouncing_ball_fmu.py`와 동일: `g`,`e`,`floor` 입력 / `h`,`v` 출력).

```bash
cd fmu/cpp/native_fmu
./build.sh                    # gcc로 .so 빌드 + modelDescription.xml과 묶어서 .fmu로 zip
python validate_native_fmu.py # fmpy로 로드+시뮬레이션, 바닥 비침투 + e^2 감쇠 확인
```

**검증**: 바닥 비침투, 봉우리 감쇠 비율 0.4898~0.4900 (e²=0.49와 거의 완벽히 일치 — Python 버전보다도 더 정확).

**Python이 정말 하나도 안 낀다는 증거**: `ldd binaries/linux64/bouncing_ball_native.so` → `libc.so.6`밖에 안 나옴(pythonfmu 빌드는 내부에 CPython 인터프리터를 통째로 품고 있었음). 파일 크기도 17KB vs pythonfmu `.fmu`의 660KB.

**같은 fmpy 드라이버로 구동해도 속도가 다름** (do_step만 비교):

| FMU | 스텝당 시간 | RTF |
|---|---|---|
| pythonfmu 빌드 (내부에 Python 인터프리터 내장) | 4,367.1 ns | 458x |
| 네이티브 C (이번에 만든 것) | **2,685.9 ns** | **744.6x** |

드라이버(fmpy/ctypes)는 양쪽 다 Python이라 그 오버헤드는 그대로 남아있는데도 약 1.6배 빨라짐 — 이게 "FMU 내부 구현이 Python이냐 아니냐"가 기여하는 몫이고, 순수 C++ `bouncing_ball --bench`의 800배와의 나머지 격차(대략 500배)는 드라이버 쪽(fmpy/ctypes) 오버헤드로 추정됨.

**네이티브 드라이버 (`fmu/cpp/native_fmu/driver/`) — 구동 측도 Python 배제**: `dlopen`/`dlsym`으로 `.so`를 직접 로드해서 `fmi2Instantiate`/`fmi2DoStep`/`fmi2GetReal` 등을 호출하는 C 프로그램. fmpy가 하던 역할(모델 로드, 스텝 구동, 값 읽기)을 그대로 하지만 프로세스 안에 Python 인터프리터가 아예 없음.

이 레포의 비슷하게 생긴 세 파일이 각각 다른 역할이라 헷갈리지 않게 정리:

| 파일 | 역할 |
|---|---|
| `fmu/cpp/bouncing_ball.cpp` (→ `bouncing_ball`) | FMI 자체를 안 씀. 순수 C++ 물리 계산만 — 위 "속도 비교" 표의 성능 상한선(ceiling) 역할 |
| `fmu/cpp/native_fmu/sources/bouncing_ball_native.c` (→ `.so`/`.fmu`) | FMI2 C API를 구현한 **모델 자체**. 혼자 실행 안 되고 반드시 드라이버가 로드해서 호출해야 함 |
| `fmu/cpp/native_fmu/driver/fmu_driver.c` (→ `fmu_driver`) | 그 `.so`를 `dlopen`으로 불러 구동하는 **드라이버**. `validate_native_fmu.py`/`benchmark_realtime.py`의 fmpy 역할을 C로 대체 |

```bash
cd fmu/cpp/native_fmu
./build.sh                          # 모델 .so + .fmu 빌드 (위와 동일)
cd driver
./build.sh                          # 드라이버 빌드 (gcc + -ldl)
./fmu_driver .. bench 10 0.002   # 페이싱 없이 순수 구동 속도 (첫 인자는 압축 푼 FMU 디렉터리 -- native_fmu/ 자체가 그 레이아웃)
./fmu_driver .. csv 3 0.002      # time,h,v CSV 출력 (물리 검증용)
```

**완전 네이티브 경로(모델도 C, 드라이버도 C)의 속도**:

| 경로 | 스텝당 시간 | RTF |
|---|---|---|
| pythonfmu 모델 + fmpy 드라이버 | 4,367.1 ns | 458x |
| 네이티브 C 모델 + fmpy 드라이버 (Python 드라이버 오버헤드 남음) | 2,685.9 ns | 744.6x |
| **네이티브 C 모델 + `fmu_driver`(C, dlopen)** | **3.67 ns** | **545,613x** |
| 순수 C++ (FMI 레이어 자체가 없음, 상한선) | 2.38 ns | 840,336x |

Python을 fmpy 드라이버에서 C `dlopen` 드라이버로 바꾸는 것만으로 2,685.9ns → 3.67ns, 약 730배 빨라짐 — 격차의 대부분이 "FMU 내부 구현이 Python이냐"가 아니라 "구동 측(driver)이 Python/ctypes냐"였다는 뜻. 완전 네이티브 경로는 이제 순수 C++ 상한선의 1.5배 이내(FMI 함수 포인터 호출 몇 개의 오버헤드)까지 근접.

**네이티브 FMU의 페이싱(paced) 버전 — Python·C++ 둘 다**: 위 `bench`/`csv`는 둘 다 플랫아웃(가능한 빨리 실행)이라 실제 페이싱(wall-clock과 맞추기)이 아님. 아래처럼 Python·C++ 양쪽에 페이싱 모드를 추가함. (`fmu_driver.c`는 새 파일이 아니라 기존 파일 그대로에 세 번째 모드만 얹은 것 — `bench`/`csv`는 손대지 않았고 지금도 그대로 동작함. 실행파일 이름도 여전히 `fmu_driver` 하나: `fmu_driver {bench|csv|paced} ...`.)

```bash
# Python: 기존 benchmark_rt_jitter.py가 --fmu로 임의의 FMU를 받게 이미 되어 있어서,
# 네이티브 FMU 경로를 그냥 가리키기만 하면 됨 (새 파일 불필요)
python fmu/benchmark_rt_jitter.py --fmu fmu/cpp/native_fmu/bouncing_ball_native.fmu --sim-time 60 --label native-py-60s

# C++: fmu_driver에 새로 추가한 paced 모드. bouncing_ball.cpp --paced와 동일한
# yield() 기반 대기 + 퍼센타일 출력 + rt_results/*.json과 같은 스키마의 --json-out
cd fmu/cpp/native_fmu/driver
./fmu_driver .. paced 60 0.002 --json-out ../../../rt_results/cpp-native-fmu-driver-60s.json
```

**결과 (60초, n=30000, SCHED_OTHER) — "네이티브 FMU + 네이티브 드라이버" vs 기존 "FMU 없는 순수 C++/Python 페이싱 루프" 비교**:

| 결과 파일 | p50 | p95 | p99 | p999 | max |
|---|---|---|---|---|---|
| `py-other-60s` (기존, FMU 없음) | 2000.0us | 2001.1us | 2002.8us | 2058.2us | 13090.4us |
| `native-py-60s` (네이티브 FMU + fmpy) | 2000.0us | 2001.0us | 2002.7us | 2020.6us | 4602.4us |
| `cpp-other-60s` (기존, FMU 없음) | 2000.0us | 2000.5us | 2000.6us | 2010.9us | 2040.7us |
| `cpp-native-fmu-driver-60s` (네이티브 FMU + `fmu_driver`) | 2000.0us | 2000.5us | 2000.7us | 2014.1us | 4714.6us |

p50~p99까지는 FMU를 끼우든 안 끼우든, 언어가 Python이든 C++이든 사실상 동일함 — `--paced`/`paced` 루프의 페이싱 정밀도는 OS 스케줄러 레벨 현상이지 FMI 레이어나 언어가 좌우하는 게 아니라는, 앞의 결론(`### C++ 페이싱` 아래 결론과 SCHED_FIFO 재검증 결과)과 일관됨. p999/max의 산발적인 큰 값(4.6~4.7ms)은 이 머신이 격리 안 된 일반 데스크톱이라 다른 프로세스에 밀리는 드문 스톨로, run마다 위치만 다를 뿐 양쪽 다 비슷한 빈도로 나타남. `rt_comparison.png`를 다시 그리면(`python fmu/plot_rt_comparison.py`) 6개 결과 세트가 모두 겹쳐 보임.

### 진짜 Modelica FMU를 `fmu_driver`로 구동 (fmu/modelica/)

지금까지 `fmu_driver`가 구동한 FMU는 전부 이 레포에서 직접 만든 것(pythonfmu, 네이티브 C)이었음. 이번엔 반대로 **우리 것이 아닌, OpenModelica(`omc`)가 생성한 실제 FMU**를 그 위에 그대로 얹어봄 — 이 머신엔 OpenModelica 1.26.1이 이미 설치되어 있음(`omc --version`).

`fmu/modelica/BouncingBallModelica.mo`에 다른 파일들과 동일한 물리(`g=-9.81, e=0.7, floor=0.0, h0=10.0, v0=0.0`)를 진짜 Modelica 하이브리드 모델로 작성함(`der(h)=v; der(v)=g; when h<=floor then reinit(...)`) — 우리 손으로 짠 explicit-Euler 적분기와 달리, OpenModelica의 기본 솔버가 바운스를 zero-crossing 이벤트로 정확히 잡아내는 게 차이점.

```bash
cd fmu/modelica
./build.sh    # omc로 .fmu 빌드 + extracted/ 에 압축 풀어둠 (fmu_driver가 바로 쓸 수 있게)
```

**`fmu_driver` 자체를 일반화함**: 지금까지는 우리 모델의 GUID/value-reference(`VR_H=3` 등)가 코드에 하드코딩돼 있어서 다른 FMU엔 그대로 못 썼음. 이번에 `fmu_driver.c`가 실행 시점에 `modelDescription.xml`을 직접 읽어서 `guid`/`modelIdentifier`/`h`·`v`의 value reference를 알아내도록 고침 — 그래서 첫 인자가 `.so` 경로가 아니라 **압축 푼 FMU 디렉터리**(`modelDescription.xml` + `binaries/linux64/*.so`가 있는 곳)로 바뀜:

```bash
cd fmu/cpp/native_fmu/driver
./build.sh
./fmu_driver ..                              paced 10 0.002   # 우리 네이티브 FMU (native_fmu/ 자체가 이미 이 레이아웃)
./fmu_driver ../../../modelica/extracted     paced 10 0.002   # 방금 만든 OpenModelica FMU -- 코드 수정 없이 그대로 됨
```

**디버깅 포인트 하나**: `omc`가 만든 FMU를 처음 돌렸더니 `fmi2Instantiate` 안에서 바로 세그폴트가 남. 원인은 우리 자체 FMU는 `fmi2CallbackFunctions*`가 `NULL`이어도 malloc으로 대체하도록 짜놨지만(편의상 봐준 것), OpenModelica가 생성한 FMU는 스펙대로 콜백이 항상 유효하다고 가정하고 인스턴스화 중에 바로 참조함 — `fmu_driver`가 `NULL`을 넘기고 있었던 게 문제. 실제 로거(`fmu_logger`)와 `calloc`/`free` 기반 `allocateMemory`/`freeMemory` 콜백을 채워 넣고, 리소스 위치도 `file://<절대경로>/resources` URI로 제대로 넘기도록 고치니 해결됨(fmpy는 원래 이걸 항상 제대로 넘겨주고 있어서 fmpy 쪽에서는 처음부터 문제없이 동작했음 — 그래서 세그폴트가 `fmu_driver`만의 문제라는 걸 fmpy 대조로 먼저 확인할 수 있었음).

**검증**: 바닥 비침투(min h=0.0), 봉우리 감쇠 비율 0.494~0.497(외부 통신 스텝이 2ms라 피크를 정확히 못 찍어서 우리 모델의 0.4898~0.4900보다 살짝 느슨하지만 e²=0.49에 근접). `paced` 모드 페이싱 지터도 우리 모델과 같은 수준(p50/p95/p99 ≈ 2000.0/2000.5/2000.6us).

**속도는 훨씬 느림**: `bench` 기준 이 Modelica FMU는 **~310 ns/step**(6,460x realtime) — 우리 네이티브 FMU의 2.4ns(826,000x)보다 100배 이상 느림. 물리는 스칼라 3개짜리로 똑같은데도 이런 차이가 나는 이유는, OpenModelica가 매 스텝 이벤트 감지(zero-crossing) + (지금은 안 쓰지만) 비선형 솔버 인프라까지 포함한 범용 시뮬레이션 런타임을 돌리기 때문 — 우리 손으로 짠 3줄짜리 `if (h<floor)` 체크와는 계산량 자체가 다름. 나중에 실제 차량처럼 무거운 모델이면 이 차이는 좁혀질 것으로 예상(고정비용 비중이 줄어드니까).

**일반 터미널에서 테스트하려면**:

```bash
cd ~/chrono_test

# 1) Modelica FMU 빌드 (OpenModelica omc 필요 -- 이미 설치돼 있음: omc --version)
cd fmu/modelica
./build.sh                   # BouncingBallModelica.fmu 빌드 + extracted/ 에 압축 풀기

# 2) C 드라이버 빌드 (아직 안 했으면 -- 우리 네이티브 FMU도 같이)
cd ../cpp/native_fmu
./build.sh
cd driver
./build.sh

# 3) 같은 드라이버로 둘 다 구동 -- 코드 수정 없이 디렉터리만 바꿔서
./fmu_driver .. bench 5 0.002                              # 우리 네이티브 FMU
./fmu_driver ../../../modelica/extracted bench 5 0.002     # OpenModelica FMU

# 4) 물리 검증 (CSV로 바운스 확인)
./fmu_driver ../../../modelica/extracted csv 8 0.002 2>/dev/null | head -5

# 5) 페이싱(실시간) 모드
./fmu_driver ../../../modelica/extracted paced 10 0.002
```

3번에서 두 FMU 모두 시작할 때 `model: <이름> guid=... h=vr.. v=vr..` 한 줄이 stderr로 찍힘 — 드라이버가 `modelDescription.xml`을 읽어서 실제로 다른 GUID/value-reference를 알아냈다는 증거. Python(fmpy) 쪽으로 대조 검증하려면(`conda activate chrono` 필요): `python fmu/benchmark_rt_jitter.py --fmu fmu/modelica/BouncingBallModelica.fmu --sim-time 10 --label modelica-py-test`.

### 반대 방향: Chrono로 만든 slave를 Modelica가 master로 불러올 수 있나? (fmu/chrono_bouncing_ball_fmu.py)

바로 위는 "Modelica가 만든 FMU를 우리 드라이버가 구동"이었는데, 반대로 **"Chrono 물리를 담은 FMU를 OpenModelica가 master로 불러와 구동"**이 되는지 확인해봄. 결론부터: **지금 방식(pythonfmu)으로는 안 됨** — 막힌 지점과 이유가 명확해서 기록.

**1) 먼저 진짜 Chrono 물리로 FMU를 만듦.** `fmu/chrono_bouncing_ball_fmu.py`는 지금까지의 bouncing ball들과 달리 `do_step()`이 물리를 손으로 적분하지 않고, `pythonfmu`의 `exit_initialization_mode()` 훅에서 진짜 `pychrono.ChSystemNSC`를 만들어(구체 + 바닥, Bullet 충돌, `ChContactMaterialNSC.SetRestitution(e)`) `do_step()`마다 `sys.DoStepDynamics(step_size)`만 호출하고 실제 시뮬레이션된 위치/속도를 읽어옴 — 즉 진짜 "Chrono가 뒤에서 돌아가는 slave".

```bash
cd fmu
pythonfmu build -f chrono_bouncing_ball_fmu.py -d build   # build/ChronoBouncingBall.fmu
```

fmpy로 구동해보면 물리 자체는 정상(바닥 비침투, 바운스마다 높이 감쇠) — 다만 손으로 짠 explicit-Euler 모델과 달리 실제 접촉 솔버가 관여하다 보니 감쇠 비율이 이상적인 e²=0.49에 깔끔히 들어맞지 않고 바운스마다 0.32~0.49 사이로 흔들림(솔버 반복 횟수/수렴 특성 때문으로 추정, 미조사) — 이상화된 수식이 아니라 진짜 물리 엔진이 접촉을 푸는 거라 자연스러운 차이.

**부수적으로 발견한 버그**: fmpy로 8초를 다 돌리고 `freeInstance()`를 호출하는 시점에 `corrupted double-linked list`로 크래시함(시뮬레이션 결과 자체는 끝까지 정상, 정리(cleanup) 단계에서만 죽음). pythonfmu가 내장하는 CPython 인터프리터를 종료하는 과정과 pychrono(SWIG로 감싼 C++ 객체, Bullet 충돌 시스템 등 전역 상태를 가짐)가 서로 안 맞는 것으로 추정 — 아직 원인 미조사, 물리 검증 자체엔 영향 없어서 이번엔 넘어감.

**2) OpenModelica로 이 FMU를 master로 불러오는 두 가지 경로를 시도함:**

- **`omc`의 `importFMU()`** (OMEdit GUI의 "Import FMU" 메뉴가 내부적으로 호출하는 바로 그 함수) — 시도하자마자 `Error: The FMU version is 2.0 and FMU type is CoSimulation. Unsupported FMU type. Only FMI 2.0 ModelExchange is supported.` `pythonfmu`는 Co-Simulation FMU만 만들 수 있는데, 이 경로는 **Model Exchange만** 받음. 즉 OMEdit의 기본 "FMU 불러오기" 메뉴로는 애초에 우리 FMU 종류 자체를 못 받음.
- **`OMSimulator`** (OpenModelica의 별도 co-simulation 전용 마스터 — `--mode=cs`로 CS FMU를 직접 로드 가능하고, OMEdit 안에도 "SSP" perspective로 통합돼 있음): `OMSimulator --mode=cs ... build/ChronoBouncingBall.fmu` → `undefined symbol: _Py_NoneStruct`로 `fmi2Instantiate()` 자체가 실패. `LD_PRELOAD`로 `libpython3.12.so`를 강제로 얹어서 심볼만 해결해봤더니 이번엔 바로 세그폴트 — 인터프리터가 초기화(`Py_Initialize`)조차 안 된 상태라 더 근본적인 문제였음.

**원인 정리**: `pythonfmu`가 컴파일하는 `.so`는 완전히 독립적인 라이브러리가 아니라, **자신을 dlopen하는 프로세스가 이미 Python 프로세스일 것**(그래서 `libpython` 심볼이 이미 전역 심볼 테이블에 있을 것)을 전제로 함. `fmpy`는 그 자체가 Python 프로세스라 문제없이 동작하고(우리가 이번 세션 내내 잘 썼던 이유), 우리 `fmu_driver`(C)도 결국 이 FMU엔 못 씀(같은 이유). `OMSimulator`/`OMEdit`도 순수 C++ 바이너리라 마찬가지로 안 됨.

**결론**: Chrono 물리를 담은 FMU를 Modelica GUI가 master로 불러오려면, 지금처럼 `pythonfmu`로 Python을 통해 감싸는 방식이 아니라 **C/C++에서 직접 Chrono API를 호출하는 네이티브 FMU**(`fmu/cpp/native_fmu/`가 손물리 대신 실제 `ChSystemNSC` 호출을 하도록 만든 버전)가 필요함. 즉 "Python으로 감싼 Chrono"는 Python 쪽 master(fmpy 등)까지만 통하고, "진짜 언어 무관 FMU"가 되려면 C++에서 Chrono를 직접 호출해야 함 — 이때는 "Chrono를 C++ 소스에서 새로 빌드해야 하나?"가 걱정이었는데, 바로 다음 섹션("완성: ...")에서 확인했듯 **불필요했음**(conda env에 이미 C++ 헤더/라이브러리가 있었음).

**검증**: 위 결론이 정말 "Python이 문제였다"인지, 아니면 OMSimulator/FMI 조합 자체가 우리 FMU와 안 맞는 건지 구분하기 위해, **이미 갖고 있던 Python-무관 FMU**(`fmu/cpp/native_fmu/bouncing_ball_native.fmu` — 손으로 짠 C 물리라 Chrono는 아니지만, `ldd`로 확인했듯 `libc.so.6`만 링크된 완전히 독립적인 `.so`)를 그대로 `OMSimulator`에 넣어봄:

```bash
cd fmu/cpp/native_fmu
OMSimulator --mode=cs --startTime=0 --stopTime=8 --stepSize=0.002 --resultFile=result.csv bouncing_ball_native.fmu
```

**바로 성공함** — 에러 없이 끝까지 돌고(`exit 0`), `result.csv`에 바닥 비침투 + 정상적인 바운스 감쇠가 그대로 찍힘. 즉 `OMSimulator`가 CS FMU 자체를 못 다루는 게 아니라(그런 거였으면 이것도 실패했어야 함), 딱 **pythonfmu가 만드는 `.so`의 "이미 Python 프로세스 안에서 dlopen될 것"이라는 전제 하나만** 걸림돌이었다는 게 대조 실험으로 확인됨. `fmu/cpp/native_fmu/`가 Chrono 대신 손물리를 쓰고 있다는 것만 빼면, "C/C++ 네이티브 FMU면 Modelica master가 코드 수정 없이 그대로 불러온다"는 걸 이미 실증한 셈 — 남은 건 그 안의 물리를 우리 손 코드에서 실제 `ChSystemNSC` 호출로 바꾸는 것뿐.

### 완성: 네이티브 FMU 안에서 진짜 ChSystemNSC 호출 (fmu/cpp/native_fmu/sources/bouncing_ball_native_chrono.cpp)

바로 위에서 예고한 마지막 조각. **기존 `bouncing_ball_native.c`(손물리)는 전혀 건드리지 않고**, 완전히 새 소스 파일 하나를 옆에 추가하는 방식으로 함 — 파일/모델 자체를 버전업하지 않고 애초에 별도 아티팩트로 만들어서, 기존 것과 새 것을 둘 다 계속 쓸 수 있게:

| | 기존 | 신규 |
|---|---|---|
| 소스 | `sources/bouncing_ball_native.c` | `sources/bouncing_ball_native_chrono.cpp` |
| 빌드 스크립트 | `./build.sh` | `./build_chrono.sh` |
| 산출물 레이아웃 | `native_fmu/` 자체 (`modelDescription.xml` + `binaries/linux64/bouncing_ball_native.so`) | `chrono_variant/` (`modelDescription.xml` + `binaries/linux64/bouncing_ball_native_chrono.so`) |
| `.fmu` | `bouncing_ball_native.fmu` | `bouncing_ball_native_chrono.fmu` |
| 물리 | 손으로 짠 3줄 Euler | 진짜 `chrono::ChSystemNSC` (구체+바닥, Bullet 충돌, NSC restitution) |

둘 다 같은 `fmu_driver`로, 디렉터리만 바꿔서 그대로 구동됨(`./fmu_driver .. ...` vs `./fmu_driver ../chrono_variant ...`) — 기존 것도 지금까지처럼 계속 동작.

**Chrono를 순수 C++에서 링크할 수 있었던 이유**: `pychrono`를 설치한 conda `chrono` 환경 안에 Python 바인딩(`site-packages/pychrono/*.so`)뿐 아니라 **Chrono 자체의 C++ 헤더(`envs/chrono/include/chrono/`)와 공유 라이브러리(`envs/chrono/lib/libChrono_core.so`)가 이미 통째로 들어 있었음** — 그래서 README 앞부분에서 "Chrono를 C++ 소스에서 새로 빌드해야 함"이라고 적었던 게 실제로는 필요 없었음, conda 패키지 안에 이미 있었던 것. `libChrono_core.so`는 Bullet까지 정적으로 포함하고 있어서 별도 collision 라이브러리 링크도 불필요.

```bash
cd fmu/cpp/native_fmu
CHRONO_ENV=~/miniconda3/envs/chrono ./build_chrono.sh   # 기본값도 이 경로라 보통은 인자 없이 됨
cd driver
./fmu_driver ../chrono_variant bench 5 0.002    # 순수 구동 속도
./fmu_driver ../chrono_variant csv 8 0.002      # 바운스 궤적
./fmu_driver ../chrono_variant paced 10 0.002   # 실시간 페이싱
```

**빌드 중 걸린 것들**(전부 include 경로 문제, 코드 문제 아님): Eigen(`envs/chrono/include/eigen3`)과 Chrono가 내장한 Bullet(`envs/chrono/include/chrono/collision/bullet` — 이 안의 헤더들이 `"LinearMath/..."`처럼 그 디렉터리 기준 상대경로로 서로를 include해서, 이 디렉터리 자체를 `-I`에 추가해야 함)이 안 잡혀서 두 번 더 `-I`를 추가함.

**속도**: `bench` 기준 **~2,242 ns/step**(892x realtime) — pythonfmu 버전(Python 오버헤드)보다야 당연히 빠르지만, 손물리 버전의 2.4ns보다는 훨씬 느림(실제 강체 동역학 + Bullet 충돌 검사를 매 스텝 도니까 당연함). 그래도 2ms 스텝 목표 대비 2.2μs는 여유가 1000배 가까이 남아서, `paced` 모드 지터는 다른 FMU들과 다를 바 없이 깨끗함(p50/p95/p99 ≈ 2000.0/2000.5/2000.6us).

**`OMSimulator`로 실제로 불러와서 돌림 — 이번엔 진짜 Chrono가 Modelica master의 slave로 동작함:**

```bash
cd fmu/cpp/native_fmu
OMSimulator --mode=cs --startTime=0 --stopTime=8 --stepSize=0.002 --resultFile=result.csv bouncing_ball_native_chrono.fmu
```

**처음엔 또 다른 이유로 막힘**: `undefined symbol: CXXABI_1.3.15`(`libstdc++.so.6`) — `OMSimulator`는 시스템 `libstdc++`(오래됨)에 링크된 바이너리인데, `libChrono_core.so`는 conda의 최신 `libstdc++`가 필요함. 문제는 `OMSimulator` 프로세스가 시작하면서 자기 자신의 의존성으로 시스템 `libstdc++.so.6`을 먼저 로드해버리면, 우리 `.so`가 나중에 같은 soname(`libstdc++.so.6`)을 요구해도 (우리 `.so`에 박아둔 RPATH와 무관하게) 리눅스 동적 로더가 "이미 로드된 것"을 그대로 재사용해버림 — 그래서 RPATH가 있어도 무시되고 구버전으로 해석됨. **`LD_PRELOAD`로 conda의 `libstdc++.so.6`을 먼저(=OMSimulator 자신의 의존성 해석보다 먼저) 얹어서 해결**:

```bash
LD_PRELOAD=~/miniconda3/envs/chrono/lib/libstdc++.so.6 \
  OMSimulator --mode=cs --startTime=0 --stopTime=8 --stepSize=0.002 --resultFile=result.csv bouncing_ball_native_chrono.fmu
```

**결과**: `exit 0`, `result.csv`에 정상적인 바운스 궤적(바닥 비침투, t≈6.8s 근처에 정지)이 그대로 찍힘. 즉 이번 세션에서 계속 파고든 질문 — "Chrono로 만든 slave를 Modelica가 master로 불러올 수 있나?" — 에 대한 답은 **"CLI(`OMSimulator`)로는 된다, 단 (a) Python을 거치지 않는 진짜 네이티브 FMU여야 하고 (b) conda와 시스템의 `libstdc++` 버전 차이를 `LD_PRELOAD`로 맞춰줘야 한다"**.

**OMEdit GUI로는 안 됨 (이 버전은)** — 실제로 화면에 띄워서 시도해봄. `SSP` 메뉴로 새 SSP 모델(`ChronoBB` / `Root`)을 만들고, `bouncing_ball_native_chrono.fmu`를 두 가지 다른 방법으로 넣어봤는데 **둘 다 같은 에러**로 막힘:
- Root(System)에 FMU를 직접 연결
- 다이어그램 캔버스 우클릭 → "Add Submodel"(SSP 용어로 FMU 하나 = Component/Submodel)

두 경우 다 `Messages` 패널에 `[NewComponent] FMU "bouncing_ball_native_chrono" doesn't support model exchange mode.` / `Only FMI 2.0 ModelExchange is supported.` — 이건 앞서 CLI에서 `omc`의 `importFMU()`를 직접 불렀을 때 나온 에러와 **글자 그대로 동일**함. 즉 System으로 넣든 Submodel로 넣든, OMEdit 1.26.1의 SSP GUI는 FMU를 추가할 때 내부적으로 항상 이 ME 전용 `importFMU()`/`NewComponent` 경로를 거치는 것으로 보임 — CS FMU를 실제로 실행하는 `OMSimulator` 엔진 자체는 멀쩡한데(바로 위에서 CLI로 검증), 그 엔진을 감싼 GUI 쪽의 FMU-추가 기능이 이 버전에서는 CS 전용 FMU를 못 받아들이는 것. 버전 한계/버그로 보이며, 더 시도해볼 만한 GUI 경로는 없어서 여기서 마무리 — **헤드리스 `OMSimulator` CLI가 현재 유일하게 검증된 경로**.

**정리 — "Modelica가 불러올 수 있는 네이티브 FMU"가 되기까지 실제로 뭘 바꿨나**: 세 겹으로 나뉨 — FMU 자체(모델), 그걸 불러오는 우리 드라이버, 그리고 Chrono를 링크한 버전만 추가로 필요했던 빌드/런타임 설정.

1. **`bouncing_ball_native.c` (손물리, 맨 처음 만든 것) — 사실 아무것도 안 바꿨음.** `libc.so.6`만 링크하는 순수 C로 처음부터 만들어져 있었고, `OMSimulator`에 그대로 넣었더니 수정 없이 바로 성공함 — 이게 "대조 실험"으로 pythonfmu 쪽 실패 원인이 Python이었다는 걸 증명하는 기준점이 됨(자세한 내용은 위 "검증" 문단).

2. **`fmu_driver.c` (우리 쪽 C 드라이버) — Modelica가 "만든" FMU를 우리가 불러오기 위해 바꾼 것** (Modelica가 우리 FMU를 불러오는 것과는 반대 방향이지만, 같은 "언어 무관 FMU 구동" 작업이라 같이 기록):
   - 우리 모델 하나에만 맞춰 하드코딩돼 있던 GUID/`VR_H=3` 등의 value reference를 제거하고, 실행 시점에 `modelDescription.xml`을 직접 읽어 `guid`/`modelIdentifier`/`h`·`v`의 value reference를 알아내도록 변경(`find_attr_value`, `read_whole_file` 함수 추가) — 그 결과 CLI 첫 인자가 `.so` 경로에서 "압축 푼 FMU 디렉터리"로 바뀜.
   - `fmi2Instantiate`에 넘기던 `NULL` 콜백을 실제 `logger`/`allocateMemory`/`freeMemory` 콜백으로 교체 — OpenModelica가 생성한 FMU는 콜백을 무조건 참조해서 `NULL`이면 인스턴스화 중 바로 세그폴트.
   - `fmuResourceLocation`을 빈 문자열 대신 `file://<절대경로>/resources` URI로 제대로 구성.

3. **`bouncing_ball_native_chrono.cpp` (신규, Chrono 물리) — 빌드/런타임 설정만 추가, FMI2 코드 구조는 그대로**:
   - 소스 자체는 `bouncing_ball_native.c`와 같은 FMI2 함수 시그니처를 그대로 유지, `fmi2DoStep` 내부 로직만 `sys->DoStepDynamics()` 호출로 교체 — FMI 쪽 코드를 특별히 더 손볼 필요는 없었음.
   - **빌드 시점**: `-I`에 Eigen(`envs/chrono/include/eigen3`)과 Chrono 내장 Bullet(`envs/chrono/include/chrono/collision/bullet` — 이 안의 헤더들이 자기 디렉터리 기준 상대경로로 서로를 include함) 두 개를 추가해야 컴파일됨. 링크는 `-lChrono_core` 하나로 충분(Bullet이 정적으로 포함돼 있어서).
   - **런타임(FMU 자체가 아니라 `OMSimulator`를 실행하는 환경)**: `OMSimulator`(시스템 `libstdc++`에 링크)와 `libChrono_core.so`(conda의 더 새 `libstdc++` 필요)가 같은 soname을 두고 충돌 — `LD_PRELOAD=<conda>/lib/libstdc++.so.6`로 conda 버전을 먼저 로드시켜야 함. **FMU `.so`나 코드를 바꾼 게 아니라, `OMSimulator`를 실행하는 커맨드 앞에 환경변수 하나를 붙인 것뿐.**

4. **OMEdit GUI(SSP 편집기)는 이 세 가지 중 뭘 해도 안 풀림** — 바로 위에서 기록한 대로, GUI 자체가 이 버전에서 CS FMU 추가를 못 받는 별개의 한계라 우리 쪽 FMU/드라이버를 더 고친다고 해결되는 게 아님.

### 네이티브 C++ 차량 (fmu/cpp/native_vehicle_fmu/)

`bouncing_ball_native_chrono.cpp`가 증명한 패턴("conda env의 Chrono C++ 헤더/라이브러리를 직접 링크하면 Python 없이도, Modelica master가 직접 불러올 수 있는 FMU가 된다")을 실제 차량에 그대로 적용함. `fmu/chrono_vehicle_fmu.py`(pythonfmu)와 인터페이스는 동일(`steer_deg`/`drive_torque` 입력, 섀시 위치/자세/속도/조향각 출력)하되, 내부적으로 C++에서 `chrono::ChSystemNSC`를 직접 조립.

**MVP 범위**: 4륜(전륜조향/후륜구동), rigid 타이어(Bullet Coulomb 접촉), flat 지형, 평행 조향만 먼저 포팅함 — `six_wheel`/`ackermann`/`empirical_tire`/`bumps_terrain`은 다음 단계로 미룸(이 프로젝트 내내 그래왔듯 작게 돌아가는 것부터 먼저). `simple_vehicle.py`의 `make_vehicle()`/`apply_differential()` 로직과 모델 상수(질량, 치수, 스프링/댐퍼 상수 등)를 C++로 그대로 옮겨 적음.

```bash
cd fmu/cpp/native_vehicle_fmu
CHRONO_ENV=~/miniconda3/envs/chrono ./build.sh   # 기본값도 이 경로라 보통은 인자 없이 됨
```

**API 이름이 Python 바인딩과 다른 부분들**: PyChrono는 SWIG로 감싼 편의 이름을 쓰지만(`GetPosDt()`, `wheel.GetContactForce()`), C++ 원본 API에서는 벡터 성분 접근이 `.x()`/`.y()`/`.z()`(함수 호출, 속성이 아님), 조인트 계층이 `ChLinkMateGeneric`(모터)과 `ChLinkMarkers`(락 조인트)로 나뉘어 있는 등 세부가 다름 — 전부 헤더를 직접 grep해서 정확한 시그니처를 확인하고 맞춰씀(추측으로 짜지 않음).

**검증 — pythonfmu 버전과 bit-exact 일치**: 같은 시나리오(t>1.0s부터 steer_deg=20, drive_torque=260, dt=0.005)를 두 FMU에 각각 흘려서 비교:

```bash
cd fmu/cpp/native_vehicle_fmu
LD_PRELOAD=~/miniconda3/envs/chrono/lib/libstdc++.so.6 \
  OMSimulator --mode=cs --startTime=0 --stopTime=6 --stepSize=0.002 --resultFile=result.csv vehicle_native.fmu
```

fmpy로 같은 dt(0.005)를 맞춰 6초를 돌려보면 `chassis_x/y`, `yaw_deg`, `speed_mps`가 pythonfmu 버전(`ChronoVehicle.fmu`)과 **소수점까지 완전히 일치**(예: t=6.0s에서 양쪽 다 x=2.045, yaw=-149.487, speed=5.314) — 같은 Chrono 솔버를 호출하는 거라 당연하지만, C++ 포팅 과정에서 상수/부호/축 하나 잘못 옮기지 않았다는 직접적인 증거. (참고: dt를 다르게 주면 당연히 갈라짐 — 비선형 차량 동역학이라 스텝 크기가 다르면 수치적분 오차가 다르게 누적되는 것뿐, 버그 아님.)

**`OMSimulator`로 실제 구동 — 진짜 차량 물리가 Modelica master의 slave로 동작함**: 위 명령 그대로 `exit 0`, `result.csv`에 직진 가속(steer_deg 기본값 0이라 입력 안 주면 직진, t=6s에 x≈21.4m, speed≈7.1m/s로 물리적으로 타당)이 정상적으로 찍힘. 바운싱볼에서 이미 증명된 "네이티브 C++ FMU는 Python 없이, 어떤 FMI master든(OMSimulator 포함) 코드 수정 없이 불러온다"는 게 실제 차량 규모(강체 10여 개, 조인트 20여 개, 접촉/서스펜션/디퍼렌셜)에서도 그대로 성립함을 확인.

**다음**: `six_wheel`/`ackermann`/`empirical_tire`/`bumps_terrain`을 C++ 쪽에도 추가(파라미터로), Irrlicht 렌더링(아직 미검증인 `libChrono_irrlicht.so` 경로), OMEdit GUI에서 여전히 안 되는지 재확인(바운싱볼 때와 같은 한계일 가능성 높음, 재확인은 안 함).

### `fmu_driver` 일반화 — 바운싱볼 전용에서 임의의 FMU로

지금까지 `fmu_driver`는 출력 변수 이름이 `h`/`v`로, 입력을 아예 못 넣는 걸로 하드코딩돼 있었음(바운싱볼 전용) — 차량 FMU는 출력이 `chassis_x`/`yaw_deg`/... 고 입력도 `steer_deg`/`drive_torque`가 있어서 그대로는 못 씀. 두 가지를 추가함(기존 호출은 전부 그대로 동작 — 옵션 안 주면 여전히 `h`,`v` 기본값):

```
--outputs name1,name2,...   매 스텝 읽어서 출력할 Real 변수들 (기본값: h,v)
--set name=value            Real 입력값 하나를 초기화 직후, 스텝 시작 전에 한 번 설정
                             (반복 가능 — 여러 입력을 각각 --set으로)
```

```bash
cd fmu/cpp/native_fmu/driver
./fmu_driver ../../native_vehicle_fmu csv 5 0.002 \
    --set steer_deg=15 --set drive_torque=260 \
    --outputs chassis_x,chassis_y,yaw_deg,speed_mps
```

**의도적으로 안 넣은 것**: 스텝마다 바뀌는 입력(조향 램프 같은 시나리오)은 지원 안 함 — `--set`은 처음 한 번 설정하고 끝까지 고정임. 그런 제어 로직(램프, PID, 시퀀싱)은 이 범용 드라이버가 아니라 "제어 레이어"(ros2_control 등)가 할 일이라고 선을 그음.

**회귀 확인**: 옵션 없이 기존처럼 `./fmu_driver .. bench/csv/paced ...`를 다시 돌려서 바운싱볼 FMU 결과가 전과 완전히 같은 수치로 나오는 것 확인(`h=3.278573 v=-5.615244` 등, 이전 세션 결과와 동일).

**부산물**: 차량 FMU의 첫 순수 벤치마크 수치도 나옴(`bench` 모드) — **~585μs/step, 약 3.4x realtime**. 강체 10여 개 + 조인트 20여 개 + 접촉/서스펜션이 있는 모델이라 바운싱볼(수 ns/step)과는 자릿수가 완전히 다르지만, 목표 스텝(2ms)보다는 여전히 3배 이상 빨라서 `paced` 모드도 별문제 없이 맞춰 돔(p50/p95 ≈ 2000.0/2000.6us, p999은 그래도 약간 더 벌어짐 — 여유가 줄어든 만큼 자연스러운 결과).

### 4WD (전축 구동) 추가 — ros2_control 방향 논의에서 나온 요구사항

ros2_control 설계를 얘기하다가(`ackermann_steering_controller`는 표준상 **후륜에만 구동 명령**을 내리게 돼 있음 — 공식 문서에 "traction, one for each fixed [rear] wheel"이라고 명시돼 있음, `control.ros.org` 확인) 4WD를 어떻게 넣을지 결정함: 컨트롤러는 표준 그대로 두고, `ChronoFmuSystemInterface`(아직 미착수)가 후륜 트랙션 요청 하나를 "전체 스로틀"로 해석해서 앞뒤 양쪽 축에 내부적으로 뿌려주기로 함(실제 차도 ECU는 스로틀 하나만 정하고 배분은 드라이브트레인 하드웨어가 함). 이걸 가능하게 하려면 **모델 자체가 4WD를 지원해야** 해서, ros2_control 작업(1번)보다 먼저 여기(0.5번)부터 함.

**조향과 구동이 같은 축에서 충돌하지 않는 이유**: 전축은 원래도 조향축(업라이트→너클 회전)이었는데, 구동 모터를 "너클→휠" 관절(스핀 축)에 얹으면 됨 — 이 관절 자체가 이미 너클의 현재 조향각을 따라 회전한 상태라서, 실제 CV(등속) 조인트가 달린 전륜구동축과 똑같은 방식으로 동작함. `make_vehicle()`/`vehicle_native.cpp` 코드 구조가 애초에 "조향축이든 아니든 그 축의 `spin_parent`에 구동 모터를 붙인다"는 식으로 이미 일반적으로 짜여 있어서, 이 조합 자체엔 핵심 로직 변경이 전혀 필요 없었음(축 리스트에서 `is_driven` 플래그 하나만 바꾸면 됨).

**독립적인 앞/뒤 토크 입력**(B안 — "표준을 따르면 살기 편해져"라 컨트롤러 표준을 지키면서도 FMU 자체는 유연하게 열어둠):
```
four_wheel_drive     구조적 파라미터(bool) — 초기화 시에만 설정, 기본 false(후륜 전용, 기존과 동일)
drive_torque         기존 그대로, 후륜 명목 토크
drive_torque_front   신규, 전륜 명목 토크 (four_wheel_drive=false면 안 쓰임)
```
- `simple_vehicle.py`: `make_vehicle(..., four_wheel_drive=False)` 파라미터 추가 — 기본값 False라 `simple_vehicle.py`/`drive_vehicle.py`/`slip_demo.py` 등 기존 호출 전부 무변화. `apply_differential()` 자체는 안 건드림(이미 어떤 축 그룹이 들어오든 일반적으로 처리하게 짜여 있었음) — 전축용/후축용으로 두 번 나눠 호출하도록 호출부만 바꿈.
- `chrono_vehicle_fmu.py`: `four_wheel_drive`(Boolean parameter) + `drive_torque_front`(Real input) 추가.
- `vehicle_native.cpp`: 이 파일은 FMU 전용이라 다른 호출자를 신경 쓸 필요는 없지만, **Python FMU와의 bit-exact 비교를 계속 유지하려고** 똑같이 `four_wheel_drive`(0.0/1.0, `build()` 시점에 한 번만 읽음) 구조적 토글을 넣음 — 값 12(`four_wheel_drive`), 11(`drive_torque_front`)로 뒤에 추가, 기존 0~10번 변수는 그대로.

**검증**: `validate_native_vehicle_fmu.py`를 확장해서 2WD/4WD 두 시나리오 다 Python FMU vs 네이티브 C++ FMU를 비교하도록 함 — **둘 다 0.00e+00**(2WD도 4WD도 소수점까지 완전히 일치), 4WD 추가가 C++ 포팅에서도 정확히 재현됐다는 뜻.

**뜻밖의 발견 (버그 아님, 정직하게 기록)**: `four_wheel_drive=True`인데 `drive_torque_front=0`으로 주면, `four_wheel_drive=False`(전륜 모터 자체가 없음)와 **완전히 같지는 않음**(6초 뒤 yaw 기준 약 0.03도 차이). 원인은 전륜에도 이미 있던 `ChLinkLockRevolute`(스핀 조인트)와 새로 얹은 `ChLinkMotorRotationTorque`(구동 모터)가 같은 두 바디 사이에 같은 프레임으로 겹쳐서, 토크가 정확히 0이라도 구속조건 개수가 늘어나 150회 고정 반복(BARZILAIBORWEIN) 솔버의 근사해가 아주 살짝 달라지기 때문으로 보임 — `four_wheel_drive=False`(전륜 모터가 아예 안 생성됨)일 때는 기존 동작과 완벽하게 일치하는 걸 이미 확인했으니, 이건 "구조 자체를 켜는 것"의 부작용이지 로직 버그는 아님.

### C++ 페이싱 — sleep_until의 함정과 해결

이 조사의 출발점은 Modelica 툴체인 경험: 거기선 C++로 생성한 실시간 시뮬레이션이 Python보다 지터가 확실히 작았어서, Chrono/`pythonfmu`도 당연히 같은 방향일 거라 예상하고 C++ 포팅을 시작함. 아래에서 보듯 처음엔 정반대 결과가 나와서 당황했지만, 결국 원인은 C++ 자체가 아니라 첫 구현이 고른 슬립 방식이었음 — Modelica가 생성하는 코드는 애초에 이 함정을 피하도록 짜여 있었을 것.

위 벤치마크는 전부 **페이싱 없이 최대 속도**로 돈 것("이것은 페이싱이 아니잖아"라는 지적을 받고, `--paced` 모드를 추가함). 처음엔 `sleep_until`(대부분 sleep) + 마지막 150μs busy-spin 방식으로 짰는데:

| 지표 | Python (fmpy + `ChRealtimeStepTimer.Spin()`) | C++ (`sleep_until` + busy-spin, 초기 버전) |
|---|---|---|
| p50 | +0.0μs | +0.0μs |
| p95 | +1.2μs | +49~571μs |
| p99 | +15.8μs | +605~647μs |
| max | +987.8μs | +5,900~9,400μs (6~9ms) |

**계산은 800배 이상 빠른데 페이싱 tail은 오히려 훨씬 나빴음.** 환경 노이즈인지 의심해서 파이썬↔C++을 번갈아 3라운드 재확인(파이썬은 매번 깨끗, C++은 매번 나쁨 — 재현됨), "혹시 내 C++ 코드에 버그가 있는 거 아니냐"는 지적을 받고 최악의 이상치들을 **직접 계측**해서 어느 구간에서 시간이 새는지 뜯어봄:

```
순위   총지연(us)   sleep_until이 늦은 정도(us)   busy-spin(us)
  0     11531.8            9712.4                    0.2
  1      7871.4             6078.2                    0.2
```

**busy-spin은 항상 0에 가까웠고, `sleep_until()` 자체가 목표 시각보다 최대 9.7ms 늦게 깨어난 것**이 범인이었음 — 코드 로직 버그가 아니라, SCHED_OTHER 스레드는 PREEMPT_RT 커널에서도 깨어나는 시각이 보장되지 않는다는 것(PREEMPT_RT가 주로 개선하는 건 SCHED_FIFO/RR 스레드의 지연). 그래서 **OS 타이머 기반 sleep을 아예 안 쓰고 `std::this_thread::yield()`만 반복하는 방식**(Chrono의 `Spin()`이 이럴 거라 추측)으로 바꿔봤더니:

| 지표 | Python `Spin()` | C++ (`sleep_until`, 초기) | **C++ (`yield()`, 최종)** |
|---|---|---|---|
| p999 | +66.9~89.6μs | +2,120~4,563μs | **+9.6~47.7μs** |
| max | +987.8~2,088μs | +5,900~9,400μs | **+29.8~56.0μs** |

**`yield()` 방식이 파이썬과 동급이거나 더 좋음.** 결국 C++이 "느려서" 나빴던 게 아니라 **타이밍 프리미티브 선택이 잘못됐던 것** — 지금 `bouncing_ball.cpp`의 `--paced`는 이 `yield()` 방식으로 되어 있음.

```bash
./bouncing_ball --paced 10.0 0.002
```

**교훈**: 원시 연산 속도와 실시간 페이싱 품질은 별개의 문제고, "느리게 도는 게 이상하면 버그를 의심하라"는 지적 덕분에 실제로 원인(잘못된 슬립 프리미티브)을 찾아 고쳤음 — 성급하게 "언어 차이"로 결론 내렸으면 놓쳤을 부분.

**C++도 진짜 RT 커널 위에서 도는지 자체 검증**: `--paced` 실행 시 프로그램이 시작할 때 `uname()`과 `/sys/kernel/realtime`을 직접 읽어서 자기 자신이 어떤 커널·스케줄링 정책으로 돌고 있는지 스스로 찍음(외부에서 추측하는 게 아니라 프로세스 본인이 확인):
```
kernel: Linux 5.15.0-1112-realtime  (PREEMPT_RT kernel: yes)  process sched: SCHED_OTHER prio=0
```
`fifo <priority>` 인자를 추가하면 C++에서도 `sched_setscheduler(SCHED_FIFO)`를 걸 수 있음(파이썬 `--sched fifo`와 동일한 개념):
```bash
./bouncing_ball --paced 5.0 0.002 fifo 10
```

트레이드오프: 이 분리는 ROS2/Simulink 등 외부 툴과 실제로 연동할 때 값어치가 있고, 계속 이 레포 안에서만 쓸 거면 지금 구조 대비 초기 비용이 큼.
