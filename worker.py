import threading
import time



class MotorWorker(threading.Thread):
    def __init__(self, drv, base_schedule, cycle_period, tick=0.1):
        super().__init__(daemon=True)
        self.drv = drv
        self.base_schedule = base_schedule
        self.cycle_period  = cycle_period
        self.tick = tick
        self.stop_evt = threading.Event()

        self.queue = []
        self.cycle_idx = 0
        self.lock = threading.Lock()     # GUI ↔ 워커 공유 보호
        self.stat = {}                   # 최신 drv.poll 결과
        self.t0 = 0
        self.looping = False

    def start_loop(self):
        now = time.perf_counter() - self.t0
        self.queue = [c.shifted(now) for c in self.base_schedule]
        self.cycle_idx = 0
        self.looping = True 
        print(f"MotorWorker: start_loop, {len(self.queue)} commands queued, cycle_period={self.cycle_period:.2f}s, tick={self.tick:.2f}s")

    def stop_loop(self):
        self.queue.clear()
        self.cycle_idx = 0            
        self.looping = False
        print("MotorWorker: stop_loop, queue cleared")


    def stop(self):
        self.stop_evt.set()

    # 스레드 메인루프
    def run(self):
        self.t0 = time.perf_counter()
        while not self.stop_evt.is_set():
            now_rel = time.perf_counter() - self.t0

            # 스케줄 처리
            while self.looping and self.queue and now_rel >= self.queue[0].t:
                cmd = self.queue.pop(0)
                if cmd.kind == "MOVE":
                    with self.lock:
                        self.drv.move(cmd.deg, cmd.vel, cmd.acc, cmd.dwell)
                    print(f"MotorWorker: MOVE command at {cmd.t:.2f}s, deg={cmd.deg}, vel={cmd.vel}, acc={cmd.acc}, dwell={cmd.dwell}")
                elif cmd.kind == "RESTART":
                    shift = now_rel
                    self.queue = [c.shifted(shift) for c in self.base_schedule]
                    print("MotorWorker: RESTART command received, rescheduling commands")

            # 폴링
            with self.lock:
                st = self.drv.poll()
                st["time"] = now_rel
                self.stat = st
                
            # with self.lock:
            #     self.stat = st           # GUI 가 읽어갈 수 있게 저장

            time.sleep(self.tick)
