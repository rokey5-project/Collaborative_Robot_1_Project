# cocktail_orchestrator_topic.py
import rclpy
import threading
import time
import subprocess
import os
from std_msgs.msg import String

ROBOT_ID = "dsr01"
PACKAGE_PATH = os.path.dirname(os.path.abspath(__file__))

# 상태 관리
robot_state = "IDLE"  # IDLE, RUNNING, PAUSED, STOPPED
current_recipe = None
current_step = 0
current_process = None
state_lock = threading.Lock()
pause_event = threading.Event()
pause_event.set()

# 레시피 정의
RECIPES = {
    "mojito": [
        'a_motion_node.py',
        'b_put_shaker.py',
        'c_close_cap.py',
        'd_get_shaker.py',
        'e_shaking.py',
        'f_put_shaker.py',
        'g_twist_open_cap.py',
        'h_pour_drink.py',
    ],
    "highball": [
        'a_motion_node.py',
        'b_put_shaker.py',
        'h_pour_drink.py',
    ],
}

status_pub = None


def publish_status(status):
    global status_pub
    if status_pub:
        msg = String()
        msg.data = status
        status_pub.publish(msg)
        print(f"[Status] {status}")


def command_callback(msg):
    global robot_state, current_recipe
    command = msg.data.strip()
    
    print(f"\n========== [Command] 수신: {command} ==========")
    
    with state_lock:
        if command.startswith("order:"):
            recipe = command.split(":", 1)[1]
            if robot_state == "RUNNING":
                print("[Command] 이미 제조 중!")
                publish_status("busy")
                return
            if recipe in RECIPES:
                current_recipe = recipe
                robot_state = "RUNNING"
                pause_event.set()
                print(f"[Command] 주문 접수: {recipe}")
                publish_status(f"ordered:{recipe}")
            else:
                print(f"[Command] 알 수 없는 레시피: {recipe}")
                publish_status(f"error:unknown_recipe:{recipe}")
        
        elif command == "pause":
            if robot_state == "RUNNING":
                robot_state = "PAUSED"
                pause_event.clear()
                print("[Command] 일시정지 (현재 동작 완료 후)")
                publish_status("paused")
        
        elif command == "resume":
            if robot_state == "PAUSED":
                robot_state = "RUNNING"
                pause_event.set()
                print("[Command] 재개")
                publish_status("resumed")
        
        elif command == "stop":
            robot_state = "STOPPED"
            pause_event.set()
            if current_process:
                current_process.terminate()
            print("[Command] 긴급 정지")
            publish_status("stopped")


def sub_thread(sub_node):
    print("[SubThread] Subscriber 스레드 시작")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sub_node)
    try:
        executor.spin()
    except Exception as e:
        print(f"[SubThread] 에러: {e}")


def run_script(script):
    """모션 스크립트 실행"""
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
    
    if stdout:
        print(f"[Output] {stdout.strip()}")
    
    return returncode, stderr


def robot_task_thread():
    """로봇 작업 스레드"""
    global robot_state, current_recipe, current_step
    
    publish_status("ready")
    
    while True:
        with state_lock:
            state = robot_state
            recipe = current_recipe
        
        if state == "STOPPED":
            publish_status("stopped")
            break
        
        if state == "IDLE" or recipe is None:
            time.sleep(0.3)
            continue
        
        # 레시피 실행
        if state == "RUNNING" and recipe:
            steps = RECIPES[recipe]
            print(f"\n[Robot] 🍸 {recipe} 제조 시작 ({len(steps)} 스텝)")
            publish_status(f"making:{recipe}")
            
            success = True
            for i, script in enumerate(steps, 1):
                current_step = i
                
                # 일시정지 체크
                with state_lock:
                    state = robot_state
                
                if state == "PAUSED":
                    print("[Robot] ⏸️ 일시정지 중...")
                    pause_event.wait()
                
                with state_lock:
                    state = robot_state
                
                if state == "STOPPED":
                    print("[Robot] 정지됨")
                    success = False
                    break
                
                # 스크립트 실행
                print(f"\n[Robot] [{i}/{len(steps)}] {script}")
                publish_status(f"step:{i}:{len(steps)}:{script}")
                
                returncode, stderr = run_script(script)
                
                if returncode != 0:
                    print(f"[Robot] ❌ {script} 실패: {stderr}")
                    publish_status(f"error:{script}")
                    success = False
                    break
                
                print(f"[Robot] ✓ {script} 완료")
            
            # 완료 처리
            with state_lock:
                if success and robot_state == "RUNNING":
                    print(f"\n[Robot] 🍸 {recipe} 완성!")
                    publish_status(f"done:{recipe}")
                robot_state = "IDLE"
                current_recipe = None
                current_step = 0


def main(args=None):
    global status_pub, robot_state
    
    rclpy.init(args=args)
    
    # Subscriber 노드만 생성 (DSR_ROBOT2 안 씀)
    node = rclpy.create_node("cocktail_orchestrator", namespace=ROBOT_ID)
    
    # 토픽 설정
    node.create_subscription(
        String,
        f"/{ROBOT_ID}/cocktail/command",
        command_callback,
        10
    )
    
    status_pub = node.create_publisher(
        String,
        f"/{ROBOT_ID}/cocktail/status",
        10
    )
    
    # Subscriber 스레드
    sub_t = threading.Thread(target=sub_thread, args=(node,), daemon=True)
    sub_t.start()
    
    # 작업 스레드
    robot_t = threading.Thread(target=robot_task_thread, daemon=False)
    robot_t.start()
    
    print("\n" + "="*50)
    print("  🍸 칵테일 오케스트레이터 준비 완료")
    print("="*50)
    print(f"모션 스크립트 경로: {PACKAGE_PATH}")
    print(f"등록된 레시피: {list(RECIPES.keys())}")
    print("-"*50)
    print("명령어:")
    print(f"  ros2 topic pub --once /{ROBOT_ID}/cocktail/command std_msgs/String \"data: 'order:mojito'\"")
    print(f"  ros2 topic pub --once /{ROBOT_ID}/cocktail/command std_msgs/String \"data: 'pause'\"")
    print(f"  ros2 topic pub --once /{ROBOT_ID}/cocktail/command std_msgs/String \"data: 'resume'\"")
    print(f"  ros2 topic pub --once /{ROBOT_ID}/cocktail/command std_msgs/String \"data: 'stop'\"")
    print("="*50 + "\n")
    
    try:
        robot_t.join()
    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C")
        with state_lock:
            robot_state = "STOPPED"
        pause_event.set()
        robot_t.join()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
