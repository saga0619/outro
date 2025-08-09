from dataclasses import dataclass, replace
from typing import List

# ── MOVE 스케줄 파서 ───────────────────────────────────────────
@dataclass
class Command:
    t: float                 # 실행 시각(접속 기준 상대시간)
    kind: str                # "MOVE" | "RESTART" | "HOME" | "ESTOP"
    deg: float = None  # 회전각(도 단위, MOVE 명령에만 사용)
    vel: int = None  # 회전각(도 단위, MOVE 명령에만 사용)
    acc: int = None  # 회전각(도 단위, MOVE 명령에만 사용)
    dwell: int = None  # 회전각(도 단위, MOVE 명령에만 사용)
    def shifted(self, delta: float):      # 시각만 +delta 해 복제
        return replace(self, t=self.t + delta)

def parse_schedule(txt: str) -> List[Command]:
    cmds = []
    for ln in txt.strip().splitlines():
        ln = ln.split("#", 1)[0].strip()     # 주석 제거
        if not ln:
            continue
        parts = [p.strip() for p in ln.split(",")]
        t         = float(parts[0])
        cmd_type  = parts[1].upper() if len(parts) > 1 else "MOVE"

        if cmd_type in ("RESTART", "RESET"):
            cmds.append(Command(t, "RESTART"))
        elif cmd_type == "MOVE":
            _, type, deg, vel, acc, dwell = parts    # parts[1] == "MOVE"
            cmds.append(Command(t, "MOVE", float(deg), int(vel), int(acc), int(dwell)))
        else:
            raise ValueError(f"알 수 없는 명령: {cmd_type}")
    cmds.sort(key=lambda c: c.t)
    return cmds