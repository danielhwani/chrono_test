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
  2. Chrono 네이티브 `chrono_fmi` 모듈은 conda `pychrono` 바이너리엔 없고 C++ 소스 빌드가 필요해서 보류.
- **제어**: ros_control 스타일 — 아직 미착수.

**진행 상황**: `fmu/` 아래에 pythonfmu 툴체인 자체를 검증하는 토이 FMU(`free_fall_fmu.py`, 자유낙하 적분기)를 먼저 만들어 빌드→로드→시뮬레이션이 실제로 동작하는지 확인 완료. 다음 단계는 이 틀에 실제 차량(`make_vehicle`) 동역학을 넣는 것.

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

## C++ 버전 (fmu/cpp/)

같은 바운싱볼 모델을 **의존성 없는 순수 C++**로도 포팅함 — Python 버전(`bouncing_ball_fmu.py`)의 물리 로직 자체가 애초에 PyChrono 없이 스칼라 수식(중력 적분 + 바닥 반사)뿐이었어서, C++ 이식도 Chrono 없이 가능함. FMU로 감싸서 구동 측까지 C++로 가는 것(실시간성 향상이 목적)의 첫 단계 — 렌더링은 Chrono/Irrlicht C++ 라이브러리를 별도로 빌드해야 해서(`chrono_fmi`와 같은 장벽) 현재는 보류.

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

**다음 단계(미착수)**: FMI2 C API를 직접 구현해서 진짜 네이티브 C++ FMU로 패키징하고, 구동 측도 C++(또는 fmpy가 아닌 C 드라이버)로 — Python이 전혀 안 끼는 완전한 경로를 만드는 것.

### C++ 페이싱 — 예상 밖의 결과

위 벤치마크는 전부 **페이싱 없이 최대 속도**로 돈 것("이것은 페이싱이 아니잖아"라는 지적을 받고, `--paced` 모드를 추가함). `benchmark_rt_jitter.py`와 똑같은 방식(스텝마다 절대시각까지 sleep 후 마지막 150μs는 busy-spin, 전체 주기 분포를 퍼센타일로 집계)으로 C++ 페이싱을 짜봤는데:

```bash
./bouncing_ball --paced 10.0 0.002
```

| 지표 | Python (fmpy + `ChRealtimeStepTimer.Spin()`) | C++ (직접 짠 sleep+spin) |
|---|---|---|
| p50 | +0.0μs | +0.0μs |
| p95 | +1.2μs | +49~571μs |
| p99 | +15.8μs | +605~647μs |
| max | +987.8μs | **+5,900~9,400μs (6~9ms)** |

**계산은 800배 이상 빠른데, 페이싱 정확도(특히 tail)는 오히려 C++이 훨씬 나쁩니다.** 처음엔 환경 노이즈를 의심해서 파이썬↔C++을 번갈아 3라운드 연속으로 재봤는데, 파이썬은 매번 깨끗하고(max 2.0~2.1ms) C++은 매번 나쁨(max 8.8~10.2ms)이 재현돼서 노이즈가 아니라고 확인함. `sleep_until` 대신 raw `clock_nanosleep` 직접 호출, busy-spin 마진 제거, 300μs 단위로 짧게 나눠 반복 sleep(Chrono가 이렇게 할 거라 추측하고 테스트) — **세 가지 다 시도했지만 전부 Chrono의 `Spin()`보다 나빴음**(청크 방식은 오히려 더 나빠짐, p99가 5~8ms까지 나옴). Chrono 소스가 없어서 `Spin()`이 정확히 뭘 다르게 하는지는 확인 못 함.

**교훈**: 원시 연산 속도(언어/컴파일 여부)와 실시간 페이싱 품질은 별개의 문제임. 잘 튜닝된 라이브러리의 페이싱 알고리즘이, 훨씬 빠른 언어로 대충 짠 페이싱보다 나을 수 있음 — "C++이니까 당연히 더 실시간에 유리하다"는 가정이 이번 테스트에선 틀렸음.

트레이드오프: 이 분리는 ROS2/Simulink 등 외부 툴과 실제로 연동할 때 값어치가 있고, 계속 이 레포 안에서만 쓸 거면 지금 구조 대비 초기 비용이 큼.
