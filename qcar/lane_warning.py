"""Debounced, measurement-only lane-boundary proximity warning."""


class LaneBoundaryWarning:
    """Warn when a confidently detected yellow boundary approaches center.

    This class has no actuator interface. Thresholds are image-space values
    derived from repeated .151 operator-stop data and remain experimental.
    """

    def __init__(
        self,
        enter_offset=-0.32,
        exit_offset=-0.40,
        minimum_confidence=0.60,
        enter_frames=6,
        exit_frames=6,
        latch_after_entry=True,
    ):
        self.enter_offset = float(enter_offset)
        self.exit_offset = float(exit_offset)
        self.minimum_confidence = float(minimum_confidence)
        self.enter_frames = int(enter_frames)
        self.exit_frames = int(exit_frames)
        self.latch_after_entry = bool(latch_after_entry)
        self.warning = False
        self.enter_streak = 0
        self.exit_streak = 0

    def update(self, detected, confidence, offset):
        reliable = bool(detected) and float(confidence) >= self.minimum_confidence
        near = reliable and float(offset) >= self.enter_offset
        clear = (not reliable) or float(offset) <= self.exit_offset

        if not self.warning:
            self.enter_streak = self.enter_streak + 1 if near else 0
            self.exit_streak = 0
            if self.enter_streak >= self.enter_frames:
                self.warning = True
                self.enter_streak = 0
        else:
            if self.latch_after_entry:
                return True
            self.exit_streak = self.exit_streak + 1 if clear else 0
            self.enter_streak = 0
            if self.exit_streak >= self.exit_frames:
                self.warning = False
                self.exit_streak = 0
        return self.warning
