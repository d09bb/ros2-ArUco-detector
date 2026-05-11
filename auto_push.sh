#!/bin/bash

echo "==== aruco_detector GitHub 자동 업로드 시작 ===="

echo "[1] 현재 변경 상태 확인"
git status --short

echo "[2] 변경 파일 스테이징"
git add .

if git diff --cached --quiet; then
    echo "변경된 파일이 없습니다. 업로드를 종료합니다."
    exit 0
fi

echo -n "[3] 커밋 메시지 입력: "
read commit_message

if [ -z "$commit_message" ]; then
    commit_message="Auto update: $(date '+%Y-%m-%d %H:%M:%S')"
fi

echo "[4] 커밋 생성"
git commit -m "$commit_message"

echo "[5] GitHub로 push"
git push

echo "==== aruco_detector GitHub 업로드 완료 ===="
