"""Small, monotonic-clock records; sensor stamps are identities, never clocks."""
from pathlib import Path
import time
import uuid


def clock_id():
    # CLOCK_MONOTONIC values are comparable only within the same Linux boot.
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return ''


class CycleRecorder:
    def __init__(self, emit, clock=time.monotonic_ns):
        self.emit, self.clock = emit, clock
        self.run_id, self.clock_id = str(uuid.uuid4()), clock_id()
        self.sequence = 0

    def record(self, frame, event, **fields):
        self.sequence += 1
        row = dict(schema=1, source='semantic_mapper', run_id=self.run_id,
                   clock_id=self.clock_id, sequence=self.sequence, event=event,
                   event_ns=self.clock(), node_id=frame.node_id,
                   node_stamp=frame.node_stamp, session_id=frame.session_id)
        row.update(fields)
        self.emit(row)
        return row
