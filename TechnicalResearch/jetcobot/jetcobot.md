# Jetcobot

- [Environment](#environment)
- [Dependency](#dependency)

---

## Environment

현재 경로 `jetcobot/`에 파이썬 가상환경을 생성:

> `ls`를 해도 보이지 않도록 이름 앞에 `.`을 붙여준다

```sh
python3 -m venv .venv
```

가상환경 활성화:

> `source`대신에 `.` 사용
> 가상환경의 이름은 현재 경로의 이름과 같다

```sh
. .venv/bin/activate
```

`pip` 업데이트:

```sh
pip install -U pip
```

`requirements.txt`에 작성되어 있는 모듈 설치:

```sh
pip install -r requirements.txt
```

---

## Dependency

설치한 파이썬 모듈

- requests
