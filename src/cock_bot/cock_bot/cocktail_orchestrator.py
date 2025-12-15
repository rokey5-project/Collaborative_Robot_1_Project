# cocktail_orchestrator.py
import subprocess
import os
import signal
import sys

PACKAGE_PATH = os.path.dirname(os.path.abspath(__file__))

RECIPES = {
    "mojito": [
        'a_motion_node.py',
        'b_put_shaker.py',
        'c_close_cap.py',
        'd_get_shaker.py',
        'e_shaking.py',
        'f_put_shaker.py',
        'g_twist_open_cap.py',
        'h_pour_drink.py'
    ],
}

# 상태 관리
current_process = None
is_paused = False
current_step = 0

def stop_robot():
    """긴급 정지"""
    global current_process
    if current_process:
        current_process.terminate()
        current_process.wait()
        print("\n🛑 로봇 정지됨")

def signal_handler(sig, frame):
    """Ctrl+C 처리"""
    print("\n\n긴급 정지 신호 받음!")
    stop_robot()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def run_step(script):
    """스크립트 하나 실행"""
    global current_process
    
    script_path = os.path.join(PACKAGE_PATH, script)
    current_process = subprocess.Popen(
        ['python3', script_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    stdout, stderr = current_process.communicate()
    returncode = current_process.returncode
    current_process = None
    
    return returncode, stdout, stderr

def make_cocktail(recipe_name, start_from=0):
    """
    레시피 실행
    start_from: 에러 복구시 이 스텝부터 재시작
    """
    global current_step
    
    if recipe_name not in RECIPES:
        print(f"❌ 알 수 없는 레시피: {recipe_name}")
        return False, 0
    
    sequence = RECIPES[recipe_name]
    
    print(f"\n{'='*40}")
    print(f"  🍸 {recipe_name} 제조 시작")
    if start_from > 0:
        print(f"  ⚠️  스텝 {start_from + 1}부터 재시작")
    print(f"{'='*40}\n")
    
    for i, script in enumerate(sequence[start_from:], start_from + 1):
        current_step = i - 1
        print(f"[{i}/{len(sequence)}] {script} 실행 중...")
        
        returncode, stdout, stderr = run_step(script)
        
        if returncode != 0:
            print(f"\n❌ {script} 실패!")
            print(f"에러: {stderr}")
            return False, current_step
        
        print(f"✓ 완료\n")
    
    print(f"🍸 {recipe_name} 완성!\n")
    return True, len(sequence)

def main():
    recipe = sys.argv[1] if len(sys.argv) > 1 else "mojito"
    
    start_step = 0
    while True:
        success, last_step = make_cocktail(recipe, start_from=start_step)
        
        if success:
            break
        
        # 에러 발생시 선택지
        print("\n" + "="*40)
        print("  에러 복구 옵션")
        print("="*40)
        print(f"  1. 실패한 스텝({last_step + 1})부터 재시작")
        print(f"  2. 처음부터 다시")
        print(f"  3. 종료")
        print("="*40)
        
        choice = input("\n선택: ").strip()
        
        if choice == '1':
            start_step = last_step
        elif choice == '2':
            start_step = 0
        else:
            print("종료합니다.")
            break

if __name__ == '__main__':
    main()
