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

# 결과 그래프
python plot_results.py

# 키보드 조종 (↑↓←→ 또는 WASD, Space=정지, q/Esc=종료 — 어느 창에 포커스가 있든 동작)
python drive_vehicle.py
python drive_vehicle.py --six-wheel
python drive_vehicle.py --ackermann
python drive_vehicle.py --tire-model empirical

# 6륜 슬립 → 디퍼렌셜 개입 시각화
python slip_demo.py
python slip_demo.py --switch-time 4.0
```

## 알려진 이슈/한계

- **기본은 평행 조향**: `--ackermann`을 주지 않으면 좌우 앞바퀴가 항상 같은 각도로 꺾임. `--ackermann`을 켜면 실제 좌우 각도가 달라짐(CSV의 `steer_FL_deg`/`steer_FR_deg`, `plot_results.py`의 "Steer angle / chassis yaw" 그래프에서 확인 가능).
- **디퍼렌셜은 소프트웨어 방식**: 실제 기어 커플링(캐리어 각속도 = 좌우 평균)을 강제하지 않고, 속도차를 감지해서 토크를 재분배하는 제어 로직임.
- **회전 시 전복 가능**: 무게중심 높이(1.19m)가 트랙폭(1.5m) 대비 높은 편이라, 급선회 중 계속 가속하면 전복함 (물리적으로는 타당한 현상).
- Irrlicht 창은 `DISPLAY` 환경변수가 유효한 X11 세션이 있어야 뜸. `drive_vehicle.py`는 `pynput`으로 X서버 레벨 전역 키 입력을 사용하므로 터미널 포커스와 무관하게 동작함.

## 다음에 이어서 할 만한 것

- 요철(bump) 지형 통과 테스트
