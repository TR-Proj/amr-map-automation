# AMR Map Automation (DXF → Isaac Sim → map2)

공장 CAD 도면(DXF)만으로 AMR 주행용 지도(map2)를 자동 생성하는 도구입니다.
도면을 Isaac Sim 가상 공장으로 바꾸고, 가상 AMR이 웨이포인트를 따라 주행하며 LiDAR로 스캔한 데이터에서 벽 선분과 관측 기반 공분산을 뽑아 map2 파일로 내보냅니다.

```
simple.dxf ──dxf_to_usd.py──▶ output/simple.usda (벽 + 웨이포인트)
                               + mobile.usd (로봇 씬)
                             ──▶ output/simple_scene.usd ──Isaac Sim 주행·스캔──▶ output/scan_log.npz
                                                                               │
                                     scan_to_map2.py / AMR Tools 확장 ◀────────┘
                                                 │
                                                 ▼
                                        output/scanned.map2
```

## 빠른 시작

```powershell
cd {PROJECT Dir}
git clone https://github.com/andong-sunbi/amr-map-automation.git

cd {ISAAC-SIM Dir}
.\python.bat -m pip install -r {PROJECT Dir}\amr-map-automation\requirements.txt

.\python.bat {PROJECT Dir}\amr-map-automation\dxf_to_usd.py --dxf simple.dxf --robot mobile.usd

.\python.bat {PROJECT Dir}\amr-map-automation\scan_to_map2.py --report
```

순서대로 저장소 받기 → ezdxf 설치(처음 한 번) → 도면으로 가상 공장을 만들고 주행·스캔(Isaac Sim이 열림) → Isaac Sim을 닫고 map2 생성입니다. 결과는 `amr-map-automation\output\scanned.map2`에 생깁니다.


## 폴더 구조

| 경로 | 설명 |
| --- | --- |
| `dxf_to_usd.py` | DXF/DWG → USD 변환, 로봇 씬 합치기, Isaac Sim 실행까지 한 번에 수행 |
| `scan_to_map2.py` | 스캔 로그(NPZ) + USD 웨이포인트 → map2 변환 (독립 실행) |
| `mobile.usd` | 로봇 씬 템플릿: 차동구동 AMR, LiDAR, OmniGraph script node 3개 |
| `script_node/` | OmniGraph script node 스크립트 (주행 / 궤적 기록 / LiDAR 스캔 기록) |
| `exts/amr.tools/` | Isaac Sim 확장: map2 내보내기 버튼, 주행 파라미터 실시간 튜닝 |
| `simple.dxf` | 예제 도면 (40 × 24 m, 장애물 2개) |
| `output/` | 생성물 폴더 (자동 생성, Git 제외) |

## 요구 사항

- NVIDIA Isaac Sim 4.5 (Windows에서 확인)
- Isaac Sim 내장 Python에 `ezdxf` 설치

```powershell
cd {ISAAC-SIM Dir}
.\python.bat -m pip install -r {PROJECT Dir}\amr-map-automation\requirements.txt
```

## 사용법

아래 명령은 모두 Isaac Sim 설치 폴더에서 실행합니다. `--dxf`, `--robot` 같은 상대 경로 인자는 현재 폴더가 아니라 **스크립트가 있는 폴더 기준**으로 해석됩니다.

| 스크립트 | 하는 일 | 주요 옵션 |
| --- | --- | --- |
| `dxf_to_usd.py` | 도면 → 가상 공장 생성, Isaac Sim 실행, 주행·스캔 | `--dxf`, `--robot`, `--layers` … |
| `scan_to_map2.py` | 저장된 스캔 로그 → map2 (Isaac Sim을 띄우지 않음) | `--scan`, `--usd`, `--out`, `--report` |

두 스크립트의 옵션은 서로 다릅니다. `scan_to_map2.py`에 `--dxf`를 주면 오류가 납니다.

### 1. 도면 → 가상 환경 → 주행·스캔

```powershell
.\python.bat {PROJECT Dir}\amr-map-automation\dxf_to_usd.py --dxf simple.dxf --robot mobile.usd
```

1. 도면에서 벽 선분을 추출해 `output/simple.usda`를 만듭니다 (이중선 벽은 중심선으로 합침).
2. `mobile.usd`를 복사해 맵을 붙인 `output/simple_scene.usd`를 만들고, 로봇을 시작 웨이포인트에 놓습니다. 맵은 항상 도면 좌표 그대로 배치되므로(로봇 씬에 남은 이동값은 제거), 만들어지는 map2도 도면 좌표계입니다.
3. Isaac Sim이 열리고 시뮬레이션이 자동으로 시작됩니다. AMR이 웨이포인트를 따라 주행하며 다음 파일을 500 프레임마다 저장합니다.
   - `output/scan_log.npz`: 스캔별 로봇 pose + LiDAR 점 (map2 변환 입력)
   - `output/amr_route.ply`: 주행 궤적 (CloudCompare 등으로 확인)

도면 레이어를 모르면 먼저 확인하세요.

```powershell
.\python.bat {PROJECT Dir}\amr-map-automation\dxf_to_usd.py --dxf my_factory.dxf --stats
.\python.bat {PROJECT Dir}\amr-map-automation\dxf_to_usd.py --dxf my_factory.dxf --layers WALL COLUMN --unit mm
```

자주 쓰는 옵션

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--layers` | `WALL` | 벽으로 쓸 레이어 (생략 시 `CONFIG` 값) |
| `--unit` | `mm` | 도면 단위 (`mm`, `cm`, `m`, `in`, `ft`) |
| `--collapse` | `0.25` | 이중선 벽을 중심선으로 합칠 최대 두께(m), `0`이면 끔 |
| `--wall-height` / `--wall-thickness` | `4.0` / `0.20` | 생성할 벽 크기(m) |
| `--start-wp` | `1` | 로봇 시작 웨이포인트 번호 |
| `--out-dir` | `output` | 생성물 폴더 |
| `--no-ext` | — | AMR Tools 확장 자동 활성화 끄기 |

웨이포인트(주행 노드)는 `dxf_to_usd.py` 상단 `CONFIG["waypoints"]`에서 `(id, 이름, x, y, heading(도), 연결 노드)` 형식으로 정의합니다. 도면이 바뀌면 이 값도 맞춰 주세요.

### 2. 스캔 로그 → map2

충분히 주행한 뒤 둘 중 하나로 내보냅니다. `output/`은 Git에 포함되지 않으므로, clone 직후에는 1단계(주행·스캔)를 먼저 해야 합니다.

- **Isaac Sim 안에서**: `AMR Tools` 창 → `Export map2` (메모리의 스캔 데이터를 바로 사용)
- **독립 실행**:

```powershell
.\python.bat {PROJECT Dir}\amr-map-automation\scan_to_map2.py --report
```

`--report`는 선분별 길이·공분산·관측 시점 수·입사각을 출력하고, 관측이 부족한 선분(`<- weak`)을 표시합니다. 이 구간은 AMR 위치 추정이 불안정할 수 있으니 재주행하거나 리플렉터 등 물리적 참조물 보강을 검토하세요.

### 출력 map2 구성

| 섹션 | 출처 |
| --- | --- |
| `Localization.Segments` | LiDAR 스캔에 RANSAC 선분 피팅, 선분별 관측 기반 공분산 |
| `Navigation.Nodes` | USD 웨이포인트 (`ant:nodeId`, `ant:linksTo`) |
| `Navigation.Home` | 첫 번째 노드 |

## 주행 방식과 파라미터

기본 주행은 **제자리 회전 → 직선 주행 → 노드 위 정지**입니다.

1. 다음 노드를 향해 제자리에서 회전 (전진 없음)
2. 시작점–노드를 잇는 직선을 따라 주행, 가속 후 노드 앞에서 감속
3. 노드 위 허용 오차 안에서 실제로 멈추면 도착 처리 (지나치면 천천히 후진해 맞춤)
4. 잠시 정지(settle) 후 다음 노드로 1번부터 반복

주행 중 진행 방향이 `re-align above` 이상 틀어지면 멈추고 다시 제자리 회전합니다. 도착할 때마다 콘솔에 `NAV: arrived node N miss=..cm`가 찍혀 정지 오차를 확인할 수 있습니다.

`AMR Tools` 창 → `AMR Params`에서 주행 중 실시간으로 바꿀 수 있습니다. `Reset to defaults`로 기본값 복원.

| 그룹 | 파라미터 | 기본값 | 설명 |
| --- | --- | --- | --- |
| — | turn in place | 켜짐 | 끄면 예전 방식(노드 전에 목표를 바꾸고 이동하며 회전, 모서리를 깎음) |
| Drive | max speed (m/s) | 1.5 | 직선 주행 최고 속도 |
| Drive | accel / brake (m/s²) | 1.0 | 가속도. 노드 앞 감속에도 사용 |
| Drive | line-follow gain | 1.5 | 직선 경로 이탈 보정 세기 |
| Turn in place | max turn rate (rad/s) | 0.8 | 제자리 회전 최고 각속도 |
| Turn in place | turn gain | 2.0 | 남은 각도 대비 회전 속도 |
| Turn in place | min turn rate (rad/s) | 0.15 | 회전 명령의 최솟값. 멈추면 자동으로 올라감 |
| Turn in place | align tolerance (deg) | 1.5 | 이 각도 안이면 회전을 끝내고 출발 |
| Turn in place | re-align above (deg) | 8.0 | 주행 중 이 각도 이상 틀어지면 멈추고 재정렬 |
| Arrival | position tolerance (m) | 0.05 | 도착으로 인정하는 위치 오차 |
| Arrival | settle time (s) | 0.3 | 노드에서 멈춘 뒤 대기 시간 |
| Arrival | smooth: switch at (m) | 1.2 | smooth 모드에서 다음 노드로 넘어가는 거리 |
| Safety | obstacle stop (m) | 1.5 | 전방 장애물이 이 거리 안이고 노드보다 가까우면 정지 대기 |
| Robot | wheel radius (m) | 0.5 | 바퀴 반지름 |
| Robot | wheel base (m) | 1.25 | 좌우 바퀴 간격 |

확장 없이 실행해도 `script_node/amr_driving_script_node.py`의 `DEFAULTS` 값으로 같은 방식으로 주행합니다.

## 참고

- `mobile.usd`의 script node 경로는 `./script_node/...`로 저장되어 있고, `dxf_to_usd.py`가 씬을 만들 때 이 저장소의 절대 경로로 다시 연결합니다. `mobile.usd`를 Isaac Sim에서 직접 열면 script node가 스크립트를 찾지 못하니, 항상 `dxf_to_usd.py`로 만든 `output/*_scene.usd`를 쓰세요.
- script node 출력 폴더는 `dxf_to_usd.py`가 넘겨준 `--out-dir`이며, 씬을 직접 열었을 때는 열린 USD 파일이 있는 폴더입니다.
- 로봇 prim 경로는 `/World/simplerobot`, LiDAR는 `/World/simplerobot/front_sensor/Lidar`로 고정되어 있습니다. 다른 로봇을 쓰면 `script_node/` 스크립트와 `--robot-path`를 함께 바꿔야 합니다.
- 가상 스캔의 로봇 pose는 물리 엔진의 ground truth를 사용합니다 (SLAM 아님).
