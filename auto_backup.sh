#!/bin/bash

echo "==== aruco_detector 자동 백업 시작 ===="

git add .

if git diff --cached --quiet; then
    echo "변경된 파일이 없습니다. 백업하지 않습니다."
    exit 0
fi

commit_message="Auto backup: $(date '+%Y-%m-%d %H:%M:%S')"

git commit -m "$commit_message"
git push

echo "==== GitHub 자동 백업 완료 ===="
echo "$commit_message"
