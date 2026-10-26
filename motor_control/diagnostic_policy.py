"""Writer-thread routing: polling cannot evict the action/fault journal."""
from collections import deque


class EvidencePolicy:
    def __init__(self):
        self.recent = deque(maxlen=4096)
        self.until_ms = -1
        self.health = {}

    def route(self, record):
        now = record["elapsed_ms"]
        while self.recent and self.recent[0]["elapsed_ms"] < now-20_000:
            self.recent.popleft()
        data, event = record["data"], record["event"]
        severe = record["level"] in ("ERROR", "CRITICAL", "WARNING")
        transition = False
        if event == "x42s.telemetry":
            role = data.get("role")
            state = (data.get("online"), data.get("enabled"), data.get("fault"))
            transition = self.health.get(role) != state
            self.health[role] = state
            severe = transition and (state[0] is False or bool(state[2]))
        text = " ".join(str(data.get(k, "")) for k in ("command", "line", "reply", "raw"))
        important = (record["channel"] == "gui" or severe or transition
                     or event.startswith(("gait.", "safety.", "firmware.", "worker.", "bench."))
                     or any(word in text for word in (",MOVE,", ",STOP", ",EN,", "ESTOP", "OFFLINE")))
        routed = []
        if important:
            routed.append(("events", record))
        if severe:
            if now > self.until_ms:
                routed.extend(("flight", old) for old in self.recent)
            self.until_ms = now+10_000
        if now <= self.until_ms:
            routed.append(("flight", record))
        self.recent.append(record)
        return routed
