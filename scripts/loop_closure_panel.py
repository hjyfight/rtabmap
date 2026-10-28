"""Optional, non-modal loop status window; no map or SLAM parameters are changed."""

from collections import deque
import datetime
import time


class LoopStatus:
    """Keep accepted events visible across subsequent non-loop frames."""

    def __init__(self, clock=time.monotonic, hold_seconds=8.0):
        self.clock = clock
        self.hold_seconds = hold_seconds
        self.highlight_until = 0.0
        self.last_update = None
        self.state = 'WAITING_FOR_SLAM'
        self.label = '等待 SLAM 数据'
        self.nodes = 0
        self.counts = {'GLOBAL_LOOP': 0, 'PROXIMITY_ICP': 0}
        self.last_success = None
        self.recent = deque(maxlen=128)

    def update(self, event):
        kind = event.get('type')
        if kind in ('DETECTION_STATUS', 'DETECTION_HEARTBEAT'):
            self.state = event['state']
            self.label = event['state_label']
            self.nodes = event.get('nodes', self.nodes)
            if kind == 'DETECTION_STATUS' or event.get('slam_ok'):
                self.last_update = self.clock()
            if (kind == 'DETECTION_STATUS' and self.state == 'COLLECTING_HISTORY'
                    and event.get('rehearsal_merged_id')):
                self.label = '相邻帧合并中；等待移动并积累历史关键帧'
        if kind not in self.counts or event.get('target_id', 0) <= 0:
            return False
        key = (kind, event['ref_id'], event['target_id'])
        if key in self.recent:
            return False
        self.recent.append(key)
        self.counts[kind] += 1
        self.last_success = dict(event)
        self.last_update = self.clock()
        self.highlight_until = self.clock() + self.hold_seconds
        return True

    def display(self):
        now = self.clock()
        stale = self.last_update is not None and now - self.last_update >= 7.5
        if stale or self.state == 'SLAM_TIMEOUT':
            title, color = 'SLAM 数据已中断', '#a83c32'
        elif self.last_success and now < self.highlight_until:
            title, color = '回环成功', '#16834b'
        else:
            title, color = '回环检测中', '#254560'
        last = '本次监控尚未收到成功回环'
        if self.last_success:
            event = self.last_success
            kind = '全局回环' if event['type'] == 'GLOBAL_LOOP' else '空间邻近回环'
            stamp = datetime.datetime.fromtimestamp(event['time']).strftime('%H:%M:%S')
            last = f'最近成功：{stamp}  {kind}\n#{event["ref_id"]} ↔ #{event["target_id"]}'
        return title, color, last


def create_panel():
    """Lazy Qt import keeps terminal-only diagnostics usable without a desktop."""
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import QApplication, QCheckBox, QLabel, QVBoxLayout, QWidget

    app = QApplication.instance() or QApplication(['loop_closure_status'])
    # Closing the window must leave the diagnostic subscriptions running.
    app.setQuitOnLastWindowClosed(False)

    class Panel(QWidget):
        def __init__(self):
            super().__init__()
            self.status = LoopStatus()
            self.setWindowTitle('RTAB-Map 回环状态')
            self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
            self.setAttribute(Qt.WA_ShowWithoutActivating)
            layout = QVBoxLayout(self)
            self.banner = QLabel()
            self.banner.setAlignment(Qt.AlignCenter)
            self.count = QLabel()
            self.phase = QLabel()
            self.phase.setWordWrap(True)
            self.last = QLabel()
            self.sound = QCheckBox('成功时提示音（需系统支持）')
            self.sound.setChecked(True)
            for widget in (self.banner, self.count, self.phase, self.last, self.sound):
                layout.addWidget(widget)
            self.resize(420, 230)
            screen = app.primaryScreen()
            if screen:
                area = screen.availableGeometry()
                self.move(area.right() - self.width() - 20, area.top() + 40)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(200)
            self.refresh()

        def update_event(self, event):
            accepted = self.status.update(event)
            if accepted and self.sound.isChecked():
                QApplication.beep()
            self.refresh()

        def refresh(self):
            title, color, last = self.status.display()
            self.banner.setText(title)
            self.banner.setStyleSheet(
                f'background:{color};color:white;font-size:26px;font-weight:bold;padding:12px;')
            counts = self.status.counts
            self.count.setText(f'本次监控成功 {sum(counts.values())} 次  |  '
                               f'全局 {counts["GLOBAL_LOOP"]}  /  空间 {counts["PROXIMITY_ICP"]}')
            self.phase.setText(f'{self.status.label}\n当前图节点：{self.status.nodes}')
            self.last.setText(last)

    panel = Panel()
    panel.show()
    return app, panel


def spin_with_panel(node, app):
    """Bound ROS work per Qt tick so sensor traffic cannot freeze the window."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from PyQt5.QtCore import QTimer

    def pump():
        try:
            for _ in range(8):
                if not rclpy.ok():
                    app.quit()
                    return
                rclpy.spin_once(node, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            app.quit()

    timer = QTimer()
    timer.timeout.connect(pump)
    timer.start(20)
    app.exec_()
