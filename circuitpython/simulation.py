class SimulatedTesterBackend:
    def __init__(self, sample_interval_s=0.1):
        self.sample_interval_s = sample_interval_s
        self.state = "idle"
        self.startup_message = "CircuitPython simulation ready."
        self._speed_mm_per_min = 0.0
        self._absolute_position_mm = 0.0
        self._zero_reference_mm = 0.0
        self._tare_reference_n = 0.0
        self._last_motion_time = 0.0
        self._last_sample_time = 0.0
        self._run_started_at = None

    def handle_command(self, command, now):
        self._advance_motion(now)
        cmd = str(command.get("cmd", ""))

        if self.state == "estop" and cmd in {"tare_force", "zero_displacement", "jog", "start_test"}:
            return [self._error("estop_active", "Reset the board to clear E-Stop.")]

        if cmd == "tare_force":
            self._tare_reference_n = self._raw_force_for_displacement(self._current_displacement_mm())
            return [self._status("idle", "Force tared."), self._sample(now)]

        if cmd == "zero_displacement":
            self._zero_reference_mm = self._absolute_position_mm
            return [self._status("idle", "Displacement zeroed."), self._sample(now)]

        if cmd == "jog":
            if self.state == "running":
                return [self._error("busy", "Cannot jog while the test is running.")]

            direction = str(command.get("direction", "forward"))
            if direction not in {"forward", "reverse"}:
                return [self._error("invalid_direction", "Direction must be 'forward' or 'reverse'.")]

            distance_mm = float(command.get("distance_mm", 0.0))
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if distance_mm <= 0.0:
                return [self._error("invalid_distance", "Jog distance must be greater than zero.")]
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Jog speed must be greater than zero.")]

            delta = distance_mm if direction == "forward" else -distance_mm
            self._absolute_position_mm += delta
            return [self._status("idle", "Jog complete."), self._sample(now)]

        if cmd == "start_test":
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Test speed must be greater than zero.")]

            self.state = "running"
            self._speed_mm_per_min = speed_mm_per_min
            self._run_started_at = now
            self._last_motion_time = now
            self._last_sample_time = 0.0
            return [self._status("running", "Pull started at %.3f mm/min." % speed_mm_per_min)]

        if cmd == "stop":
            if self.state == "running":
                self.state = "idle"
                self._speed_mm_per_min = 0.0
                return [self._sample(now), self._status("idle", "Test stopped.")]
            return [self._status(self.state, "Stop ignored.")]

        if cmd == "estop":
            self.state = "estop"
            self._speed_mm_per_min = 0.0
            return [self._sample(now), self._status("estop", "Emergency stop triggered.")]

        return [self._error("unknown_command", "Unsupported command: %r" % cmd)]

    def poll(self, now):
        self._advance_motion(now)
        if self.state != "running":
            return []
        if now - self._last_sample_time < self.sample_interval_s:
            return []
        self._last_sample_time = now
        return [self._sample(now)]

    def _advance_motion(self, now):
        if self._last_motion_time == 0.0:
            self._last_motion_time = now
            return

        if self.state == "running":
            delta_s = max(0.0, now - self._last_motion_time)
            self._absolute_position_mm += self._speed_mm_per_min * delta_s / 60.0

        self._last_motion_time = now

    def _current_displacement_mm(self):
        return self._absolute_position_mm - self._zero_reference_mm

    def _raw_force_for_displacement(self, displacement_mm):
        extension = max(0.0, displacement_mm)
        if extension < 3.0:
            return extension * 15.0
        if extension < 6.0:
            return 45.0 + (extension - 3.0) * 8.0
        return max(0.0, 69.0 - (extension - 6.0) * 10.0)

    def _measured_force_n(self):
        measured = self._raw_force_for_displacement(self._current_displacement_mm()) - self._tare_reference_n
        return max(0.0, measured)

    def _sample(self, now):
        if self._run_started_at is None:
            timestamp_s = 0.0
        else:
            timestamp_s = max(0.0, now - self._run_started_at)

        return {
            "type": "sample",
            "timestamp_s": round(timestamp_s, 3),
            "force_n": round(self._measured_force_n(), 3),
            "displacement_mm": round(self._current_displacement_mm(), 3),
            "state": self.state,
        }

    def _status(self, state, message):
        self.state = state
        return {"type": "status", "state": state, "message": message}

    def _error(self, code, message):
        return {"type": "error", "code": code, "message": message}
