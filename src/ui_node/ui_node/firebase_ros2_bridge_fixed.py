

import time
import subprocess
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import threading
import signal
import sys
import os
from datetime import datetime


# ========================================
# Firebase 설정
# ========================================
SERVICE_ACCOUNT_KEY_PATH = "./src/ui_node/ui_node/cocooker-firebase-adminsdk-fbsvc-7ce2458786.json"
DATABASE_URL = "https://cocooker-default-rtdb.asia-southeast1.firebasedatabase.app/"

# Firebase 초기화
try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DATABASE_URL
    })
    print("✅ Firebase 초기화 완료")
except ValueError:
    print("⚠️ Firebase 앱이 이미 초기화되었습니다.")
except Exception as e:
    print(f"❌ Firebase 초기화 실패: {e}")
    sys.exit(1)


# ========================================
# ROS2 프로세스 관리
# ========================================
class ROS2ProcessManager:
    def __init__(self):
        self.process = None
        self.is_running = False
        self.lock = threading.Lock()
        
    def start(self, package_name, executable_name):
        with self.lock:
            if self.is_running:
                print("⚠️ 이미 실행 중입니다.")
                return False
            
            try:
                env = os.environ.copy()
                
                cmd = [
                    '/bin/bash',
                    '-c',
                    f'source /opt/ros/humble/setup.bash && '
                    f'source ~/cobot_ws/install/setup.bash && '
                    f'ros2 run {package_name} {executable_name}'
                ]
                
                print(f"🚀 실행 명령: {' '.join(cmd)}")
                
                self.process = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                    text=True,
                    bufsize=1
                )
                
                self.is_running = True
                print(f"✅ 실행 완료 (PID: {self.process.pid})")
                
                threading.Thread(target=self._monitor_output, daemon=True).start()
                
                update_status("실행 중", "running")
                return True
                
            except FileNotFoundError as e:
                print(f"❌ 실행 파일을 찾을 수 없음: {e}")
                update_status("오류: 실행 파일 없음", "error")
                return False
            except Exception as e:
                print(f"❌ 실행 실패: {e}")
                update_status(f"오류: {str(e)}", "error")
                return False
    
    def _monitor_output(self):
        """🆕 ROS2 출력을 실시간으로 터미널에 그대로 출력"""
        if not self.process:
            return
        
        def read_stdout():
            try:
                for line in self.process.stdout:
                    # 🆕 ROS2 출력을 그대로 출력 (접두사 없이)
                    print(line.rstrip(), flush=True)
            except Exception as e:
                print(f"⚠️ stdout 읽기 오류: {e}")
        
        def read_stderr():
            try:
                for line in self.process.stderr:
                    # 🆕 ROS2 에러를 그대로 출력 (접두사 없이)
                    print(line.rstrip(), flush=True)
            except Exception as e:
                print(f"⚠️ stderr 읽기 오류: {e}")
        
        # 🆕 stdout과 stderr을 동시에 읽기 위해 별도 스레드 사용
        threading.Thread(target=read_stdout, daemon=True).start()
        threading.Thread(target=read_stderr, daemon=True).start()
        
    def stop(self):
        with self.lock:
            if not self.is_running or self.process is None:
                print("⚠️ 실행 중인 프로세스가 없습니다.")
                return False
            
            try:
                pgid = os.getpgid(self.process.pid)
                print(f"🛑 프로세스 그룹 종료 시도 (PGID: {pgid})")
                
                os.killpg(pgid, signal.SIGINT)
                
                try:
                    self.process.wait(timeout=5)
                    print("✅ 프로세스 정상 종료 완료")
                except subprocess.TimeoutExpired:
                    print("⚠️ 정상 종료 실패, 강제 종료(SIGKILL) 시도...")
                    os.killpg(pgid, signal.SIGKILL)
                    self.process.wait(timeout=2)
                    print("✅ 프로세스 강제 종료 완료")
                
                self.is_running = False
                self.process = None
                
                update_status("대기 중", "")
                update_current_object("중지 완료")
                return True
                
            except ProcessLookupError:
                print("⚠️ 프로세스가 이미 종료됨")
                self.is_running = False
                self.process = None
                return True
            except Exception as e:
                print(f"❌ 프로세스 종료 실패: {e}")
                update_status("오류: 종료 실패", "error")
                return False
    
    def emergency_stop(self):
        with self.lock:
            if not self.process:
                print("⚠️ 실행 중인 프로세스가 없습니다.")
                return False
            
            try:
                pgid = os.getpgid(self.process.pid)
                os.killpg(pgid, signal.SIGKILL)
                self.process = None
                self.is_running = False
                
                print("🚨 비상정지 실행")
                update_status("비상정지", "error")
                update_current_object("비상정지 완료")
                return True
            except Exception as e:
                print(f"❌ 비상정지 실패: {e}")
                return False


# ========================================
# Firebase 업데이트 함수
# ========================================
def update_status(status_text, indicator_class):
    """상태 업데이트"""
    try:
        db.reference('/status').set({
            'currentStatus': status_text,
            'indicatorClass': indicator_class,
            'timestamp': time.time()
        })
        print(f"📊 상태 업데이트: {status_text}")
    except Exception as e:
        print(f"❌ 상태 업데이트 실패: {e}")


def update_current_object(object_name):
    """현재 객체 업데이트"""
    try:
        db.reference('/currentObject').set({
            'name': object_name,
            'timestamp': time.time()
        })
        print(f"🎯 현재 객체: {object_name}")
    except Exception as e:
        print(f"❌ 객체 업데이트 실패: {e}")


# ========================================
# Firebase 명령 리스너 (수정 버전)
# ========================================
def command_listener(event):
    """Firebase에서 새로운 명령이 추가될 때 호출됨"""
    global process_manager
    
    print("\n" + "="*80)
    print(f"🔔 새 명령 수신! 시간: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
    print("="*80)
    
    # event.data는 새로 추가된 명령 자체
    command_data = event.data
    command_key = event.path.strip('/')  # Firebase 키
    
    print(f"📦 명령 키: {command_key}")
    print(f"📦 명령 데이터: {command_data}")
    print(f"📦 데이터 타입: {type(command_data)}")
    
    if not isinstance(command_data, dict):
        print(f"❌ 잘못된 명령 형식: {type(command_data)}")
        return
    
    try:
        action = command_data.get('action')
        timestamp_raw = command_data.get('timestamp', 0)
        
        print(f"\n📋 action: {action}")
        print(f"📋 timestamp: {timestamp_raw}")
        
        if not action:
            print("❌ action 필드가 없음")
            return
        
        # 타임스탬프 검증
        try:
            timestamp_ms = float(timestamp_raw)
        except (TypeError, ValueError) as e:
            print(f"❌ 타임스탬프 변환 실패: {e}")
            timestamp_ms = 0
        
        current_time_ms = time.time() * 1000
        time_diff_ms = current_time_ms - timestamp_ms
        
        print(f"\n⏱️ 현재 시간 (ms): {current_time_ms}")
        print(f"⏱️ 명령 시간 (ms): {timestamp_ms}")
        print(f"⏱️ 시간 차이 (초): {time_diff_ms/1000:.2f}")
        
        # 30초 이상 오래된 명령 무시
        if time_diff_ms > 30000:
            print(f"⏳ 오래된 명령 무시 (차이: {time_diff_ms/1000:.1f}초)")
            db.reference(f'/commands/{command_key}').delete()
            return
        
        # 명령 실행
        command_time = datetime.fromtimestamp(timestamp_ms / 1000).strftime('%Y-%m-%d %H:%M:%S')
        print(f"\n{'='*80}")
        print(f"🚀 명령 실행")
        print(f"   액션: {action}")
        print(f"   시간: {command_time}")
        print(f"   지연: {time_diff_ms/1000:.2f}초")
        print(f"{'='*80}\n")
        
        success = False
        if action == 'start':
            print("▶️ START 명령 실행")
            success = process_manager.start('kitchen_assistant', 'kitchen_voice')
        elif action == 'stop':
            print("⏹️ STOP 명령 실행")
            success = process_manager.stop()
        elif action == 'emergency_stop':
            print("🚨 EMERGENCY_STOP 명령 실행")
            success = process_manager.emergency_stop()
        else:
            print(f"❓ 알 수 없는 명령: {action}")
        
        # 명령 삭제 (중복 실행 방지)
        print(f"\n🗑️ 명령 삭제: {command_key}")
        db.reference(f'/commands/{command_key}').delete()
        
        if success:
            print(f"✅ 명령 '{action}' 실행 성공\n")
        else:
            print(f"❌ 명령 '{action}' 실행 실패\n")
        
    except Exception as e:
        print(f"\n❌ 리스너 오류!")
        print(f"오류: {e}")
        import traceback
        traceback.print_exc()
    
    print("="*80 + "\n")


# ========================================
# 메인 실행
# ========================================
def signal_handler(sig, frame):
    """Ctrl+C 처리"""
    print("\n\n👋 프로그램 종료 중...")
    if process_manager.is_running:
        process_manager.stop()
    sys.exit(0)


# 전역 변수
process_manager = ROS2ProcessManager()

def main():
    print("=" * 80)
    print("🤖 CoCooker Firebase Bridge")
    print("=" * 80)
    print("Ctrl+C로 종료하세요.")
    print("=" * 80)
    
    # Ctrl+C 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 초기 상태 설정
    print("\n📝 초기 상태 설정 중...")
    update_status("대기 중", "")
    update_current_object("-")
    print("✅ 초기 상태 설정 완료")
    
    # 기존 명령 모두 삭제 (깨끗한 시작)
    print("\n🧹 기존 명령 정리 중...")
    try:
        db.reference('/commands').delete()
        print("✅ 기존 명령 삭제 완료")
    except Exception as e:
        print(f"⚠️ 명령 삭제 실패 (무시): {e}")
    
    # Firebase 명령 리스너 시작 (child_added 이벤트 사용)
    print("\n🎧 Firebase 리스너 시작...")
    try:
        # child_added 이벤트: 새로운 명령이 추가될 때마다 호출됨
        db.reference('/commands').listen(command_listener)
        print("✅ Firebase 리스너 등록 완료 (child_added 이벤트)")
    except Exception as e:
        print(f"❌ 리스너 등록 실패: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\n✅ 명령 대기 중...")
    
    # 무한 루프
    try:
        while True:
            # 프로세스 상태 모니터링
            if process_manager.is_running and process_manager.process:
                poll = process_manager.process.poll()
                if poll is not None:
                    with process_manager.lock:
                        process_manager.is_running = False
                        process_manager.process = None
                    
                    print(f"\nℹ️ 프로세스 종료됨 (코드: {poll})")
                    
                    if poll == 0:
                        update_status("완료", "")
                        update_current_object("작업 완료")
                    else:
                        update_status(f"오류 종료 (코드: {poll})", "error")
                        update_current_object("오류 발생")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n👋 프로그램 종료")
        if process_manager.is_running:
            process_manager.stop()


if __name__ == "__main__":
    main()
