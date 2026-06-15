# SkyvoltRobot Fleet API Server (standalone)

플릿(다중 로봇) 관리용 REST API 서버를 **이 폴더만으로** 실행할 수 있게 모아둔 것입니다.
ROS 빌드(`colcon`)나 복잡한 `PYTHONPATH` 설정 없이 동작합니다.

## 폴더 구성

```
api-server/
├── server.py              # FastAPI 서버 본체 (엔드포인트 정의)
├── skyvolt_fleet/         # 서버가 의존하는 핵심 로직
│   ├── __init__.py
│   ├── scheduler.py       # 작업 → 로봇 배정 스케줄러
│   └── reservation.py     # 구간 예약 테이블 (충돌 회피)
├── requirements.txt       # 필요한 파이썬 라이브러리
├── run.sh                 # 실행 스크립트
└── README.md              # 이 파일
```

## 1) 처음 한 번만 — 라이브러리 설치

```bash
python3 -m pip install --user -r requirements.txt
```

## 2) 서버 실행

```bash
./run.sh                 # 기본 8080 포트
PORT=9000 ./run.sh       # 다른 포트로 띄우기
```

실행하면 터미널을 계속 차지합니다. **끄려면 `Ctrl + C`** 를 누르세요.

## 3) 동작 확인

브라우저에서 **운영자 대시보드**(읽기 전용, 실시간) 열기:

> http://localhost:8080/

트랙 위 로봇 위치·상태, 예약/대기 작업 수가 1초마다 갱신됩니다.

그 외:

```bash
curl http://localhost:8080/fleet        # 원시 상태 JSON
```

> http://localhost:8080/docs            # 자동 생성 대화형 API 문서

## 엔드포인트

| 메서드 | 경로 | 설명 |
|--------|------|------|
| `GET`  | `/` | **운영자 대시보드** (읽기 전용 웹 UI) |
| `GET`  | `/fleet` | 현재 로봇 위치/사용중 여부, 트랙 길이, 예약·대기 작업 수 |
| `POST` | `/jobs`  | 충전 작업 등록 (등록 즉시 스케줄러가 로봇 배정) |
| `GET`  | `/jobs/{job_id}` | 특정 작업 상태 조회 |

### 작업 등록 예시

```bash
curl -X POST http://localhost:8080/jobs \
  -H "Content-Type: application/json" \
  -d '{"target_arclength_m": 6.0, "priority": 1, "requester_id": "user-test"}'
```

응답 예:
```json
{"job_id": "...", "state": "assigned", "assigned_robot_id": "r0", "eta_s": ...}
```

## 참고: 원본과의 관계

이 폴더의 파일들은 메인 프로젝트에서 복사해 온 것입니다.

- `server.py`        ← `src/skyvolt_fleet/api/server.py`
- `skyvolt_fleet/*`  ← `src/skyvolt_fleet/skyvolt_fleet/{scheduler,reservation}.py`

원본 로직을 수정하면 이쪽 복사본도 함께 갱신해야 동기화됩니다.
