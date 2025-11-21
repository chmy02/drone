#!/usr/bin/env python3
"""
7개 토픽을 자동으로 순차 실행하는 스크립트
각 토픽을 1분씩 측정합니다.

Topic 1~7: 번호 재정렬 완료
"""

import subprocess
import sys
import time

# 7개 토픽 리스트 (번호 재정렬 완료)
SUCCESSFUL_TOPICS = [1, 2, 3, 4, 5, 6, 7]

def run_topic(topic_num, current, total):
    """단일 토픽 실험 실행"""
    print("\n" + "=" * 80)
    print(f"🔬 [{current}/{total}] Topic {topic_num} 측정 시작")
    print("=" * 80)
    
    start_time = time.time()
    
    try:
        result = subprocess.run(
            ['python3', '0_run_topic.py', str(topic_num)],
            cwd='/home/rtcl-chmy/mavros_ws/src/mavros/chmy/test',
            check=True
        )
        
        elapsed = time.time() - start_time
        print(f"✅ Topic {topic_num} 완료! (소요 시간: {elapsed:.1f}초)")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Topic {topic_num} 실패: {e}")
        return False
    except KeyboardInterrupt:
        print(f"\n⚠️ Topic {topic_num} 중단됨")
        return False


def main():
    total_topics = len(SUCCESSFUL_TOPICS)
    
    print("\n" + "=" * 80)
    print("🚀 자동 순차 측정 시작")
    print("=" * 80)
    print(f"📊 측정할 토픽: {SUCCESSFUL_TOPICS}")
    print(f"⏱️  각 토픽 측정 시간: 1분 (60초)")
    print(f"⏱️  토픽 간 간격: 2초")
    print(f"⏱️  예상 총 소요 시간: 약 {total_topics * 1.2:.0f}분")
    print("=" * 80)
    
    # 사용자 확인
    response = input("\n계속하시겠습니까? (y/n): ")
    if response.lower() != 'y':
        print("🛑 중단되었습니다.")
        return
    
    start_time = time.time()
    success_count = 0
    failed_topics = []
    
    for idx, topic_num in enumerate(SUCCESSFUL_TOPICS, 1):
        success = run_topic(topic_num, idx, total_topics)
        
        if success:
            success_count += 1
        else:
            failed_topics.append(topic_num)
        
        # 마지막 토픽이 아니면 잠시 대기
        if idx < total_topics:
            print(f"\n⏳ 다음 토픽까지 2초 대기...\n")
            time.sleep(2)
    
    # 최종 요약
    total_time = time.time() - start_time
    
    print("\n" + "=" * 80)
    print("🎉 자동 순차 측정 완료!")
    print("=" * 80)
    print(f"✅ 성공: {success_count}/{total_topics}개 토픽")
    
    if failed_topics:
        print(f"❌ 실패: Topic {failed_topics}")
    
    print(f"⏱️  총 소요 시간: {total_time / 60:.1f}분")
    print("=" * 80)
    
    print("\n💡 분석을 실행하려면:")
    print("   cd /home/rtcl-chmy/mavros_ws/src/mavros/chmy/test")
    print("   python3 2_analyze_multi_topic.py")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 중단되었습니다.")
        sys.exit(1)

