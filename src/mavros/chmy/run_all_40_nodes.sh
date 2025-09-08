#!/bin/bash

echo "🚀 40개 노드를 동시에 실행합니다..."
echo "TXQ 경합 테스트를 시작합니다!"

# 위치 명령 노드들 실행 (16개)
echo "📍 위치 명령 노드들 실행 중..."
for i in {1..16}; do
    python3 position_node_${i}.py &
    echo "Position Node ${i} started (PID: $!)"
    sleep 0.05
done

# 수동 조종 노드들 실행 (16개)
echo "🎮 수동 조종 노드들 실행 중..."
for i in {1..16}; do
    python3 manual_node_${i}.py &
    echo "Manual Node ${i} started (PID: $!)"
    sleep 0.05
done

# RC 오버라이드 노드들 실행 (8개)
echo "🎛️ RC 오버라이드 노드들 실행 중..."
for i in {1..8}; do
    python3 override_node_${i}.py &
    echo "Override Node ${i} started (PID: $!)"
    sleep 0.05
done

echo ""
echo "✅ 모든 40개 노드가 실행되었습니다!"
echo "📊 노드 목록 확인: ros2 node list"
echo "📈 TXQ 로그 확인: tail -f /home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/txq_timer_log.txt"
echo "🧵 스레드 로그 확인: tail -f /home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/thread_tracking_log.txt"
echo ""
echo "🛑 모든 노드 중지: ./stop_all_40_nodes.sh"
echo ""

# 실행 중인 노드들 확인
echo "🔄 실행 중인 노드들:"
ros2 node list

echo ""
echo "⏳ TXQ 경합 상황을 관찰하고 있습니다..."
echo "Ctrl+C로 중지할 수 있습니다."
echo ""

# 무한 대기
while true; do
    sleep 1
done
