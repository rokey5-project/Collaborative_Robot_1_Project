# cock_bot/cocktail_orchestrator.py
import subprocess
import os

# 스크립트들이 있는 경로 (setup.py entry_points로 등록했으면 그냥 이름으로 호출 가능)
PACKAGE_PATH = os.path.dirname(os.path.abspath(__file__))

SEQUENCE = [
    'a_motion_node.py',
    'b_put_shaker.py',
    'c_close_cap.py',
    'd_get_shaker.py',
    'e_shaking.py',
    'f_put_shaker.py',
    'g_twist_open_cap.py',
    'h_pour_drink.py'
]

def make_cocktail(recipe_name="default"):
    print(f"\n{'='*40}")
    print(f"  {recipe_name} 제조 시작")
    print(f"{'='*40}\n")
    
    for i, script in enumerate(SEQUENCE, 1):
        print(f"[{i}/{len(SEQUENCE)}] {script} 실행 중...")
        
        script_path = os.path.join(PACKAGE_PATH, script)
        result = subprocess.run(
            ['python3', script_path],
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print(f"❌ {script} 실패!")
            print(f"에러: {result.stderr}")
            return False
        
        print(f"✓ {script} 완료\n")
    
    print(f"{'='*40}")
    print(f"  🍸 {recipe_name} 완성!")
    print(f"{'='*40}\n")
    return True

def main():
    make_cocktail("테스트 칵테일")

if __name__ == '__main__':
    main()