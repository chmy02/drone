#!/bin/bash

echo "🛑 모든 20개 노드를 중지합니다..."

# Python 프로세스들 중지
echo "🐍 Python 노드들 중지 중..."
pkill -f "position_node_"
pkill -f "manual_node_"
pkill -f "override_node_"

# 잠시 대기
sleep 1

# 남은 프로세스 확인
echo "🔍 남은 프로세스 확인 중..."
ps aux | grep -E "(position_node_|manual_node_|override_node_)" | grep -v grep

if [ $? -eq 0 ]; then
    echo "⚠️  일부 프로세스가 아직 실행 중입니다. 강제 종료합니다..."
    pkill -9 -f "position_node_"
    pkill -9 -f "manual_node_"
    pkill -9 -f "override_node_"
else
    echo "✅ 모든 노드가 정상적으로 중지되었습니다."
fi

echo ""
echo "📊 현재 실행 중인 ROS2 노드들:"
ros2 node list

echo ""
echo "🎯 TXQ 경합 테스트가 완료되었습니다!" 