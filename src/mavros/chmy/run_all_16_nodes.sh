#!/bin/bash

echo "🚀 20개 노드를 동시에 실행합니다..."
echo "TXQ 경합 테스트를 시작합니다!"

# 위치 명령 노드들 실행 (8개)
echo "📍 위치 명령 노드들 실행 중..."
python3 position_node_1.py &
echo "Position Node 1 started (PID: $!)"
sleep 0.1

python3 position_node_2.py &
echo "Position Node 2 started (PID: $!)"
sleep 0.1

python3 position_node_3.py &
echo "Position Node 3 started (PID: $!)"
sleep 0.1

python3 position_node_4.py &
echo "Position Node 4 started (PID: $!)"
sleep 0.1

python3 position_node_5.py &
echo "Position Node 5 started (PID: $!)"
sleep 0.1

python3 position_node_6.py &
echo "Position Node 6 started (PID: $!)"
sleep 0.1

python3 position_node_7.py &
echo "Position Node 7 started (PID: $!)"
sleep 0.1

python3 position_node_8.py &
echo "Position Node 8 started (PID: $!)"
sleep 0.1

# 수동 조종 노드들 실행 (8개)
echo "🎮 수동 조종 노드들 실행 중..."
python3 manual_node_1.py &
echo "Manual Node 1 started (PID: $!)"
sleep 0.1

python3 manual_node_2.py &
echo "Manual Node 2 started (PID: $!)"
sleep 0.1

python3 manual_node_3.py &
echo "Manual Node 3 started (PID: $!)"
sleep 0.1

python3 manual_node_4.py &
echo "Manual Node 4 started (PID: $!)"
sleep 0.1

python3 manual_node_5.py &
echo "Manual Node 5 started (PID: $!)"
sleep 0.1

python3 manual_node_6.py &
echo "Manual Node 6 started (PID: $!)"
sleep 0.1

python3 manual_node_7.py &
echo "Manual Node 7 started (PID: $!)"
sleep 0.1

python3 manual_node_8.py &
echo "Manual Node 8 started (PID: $!)"
sleep 0.1

# RC 오버라이드 노드들 실행 (4개)
echo "🎛️ RC 오버라이드 노드들 실행 중..."
python3 override_node_1.py &
echo "Override Node 1 started (PID: $!)"
sleep 0.1

python3 override_node_2.py &
echo "Override Node 2 started (PID: $!)"
sleep 0.1

python3 override_node_3.py &
echo "Override Node 3 started (PID: $!)"
sleep 0.1

python3 override_node_4.py &
echo "Override Node 4 started (PID: $!)"
sleep 0.1

echo ""
echo "✅ 모든 20개 노드가 실행되었습니다!"
echo "📊 노드 목록 확인: ros2 node list"
echo "📈 TXQ 로그 확인: tail -f /home/rtcl-chmy/mavros_ws/src/mavros/libmavconn/log/txq_timer_log.txt"
echo ""
echo "🛑 모든 노드 중지: ./stop_all_20_nodes.sh"
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