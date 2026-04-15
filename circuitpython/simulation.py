class SimulatedTesterBackend:
    def __init__(self, sample_interval_s=0.1):
        self.sample_interval_s = sample_interval_s
        self.state = "idle"
        self.startup_message = "CircuitPython simulation ready."
        self._speed_mm_per_min = 0.0
        self._motion_direction = 1
        self._absolute_position_mm = 4.0
        self._zero_reference_mm = 0.0
        self._tare_reference_n = 0.0
        self._last_motion_time = 0.0
        self._last_sample_time = 0.0
        self._run_started_at = None
        self._jog_remaining_mm = 0.0
        self._homed = False
        self._home_phase = None
        self._pending_run_speed_mm_per_min = None
        self._pending_status_message = None

    def handle_command(self, command, now):
        self._advance_motion(now)
        cmd = str(command.get("cmd", ""))

        if self.state == "estop" and cmd in {"tare_force", "zero_displacement", "jog", "home", "start_test"}:
            return [self._error("estop_active", "Reset the board to clear E-Stop.")]

        if cmd == "home":
            if self.state in {"homing", "jogging", "running"}:
                return [self._error("busy", "Cannot home while motion is active.")]
            self._begin_homing(now, pending_run_speed_mm_per_min=None)
            return [self._status("homing", "Homing started.")]

        if cmd == "tare_force":
            if self.state != "idle":
                return [self._error("busy", "Force can only be tared while idle.")]
            self._tare_reference_n = self._raw_force_for_displacement(self._current_displacement_mm())
            return [self._status("idle", "Force tared."), self._sample(now)]

        if cmd == "zero_displacement":
            if self.state != "idle":
                return [self._error("busy", "Displacement can only be zeroed while idle.")]
            self._zero_reference_mm = self._absolute_position_mm
            return [self._status("idle", "Displacement zeroed."), self._sample(now)]

        if cmd == "jog":
            if self.state in {"homing", "jogging", "running"}:
                return [self._error("busy", "Cannot jog while motion is active.")]

            direction = str(command.get("direction", "forward"))
            if direction not in {"forward", "reverse"}:
                return [self._error("invalid_direction", "Direction must be 'forward' or 'reverse'.")]

            distance_mm = float(command.get("distance_mm", 0.0))
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if distance_mm <= 0.0:
                return [self._error("invalid_distance", "Jog distance must be greater than zero.")]
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Jog speed must be greater than zero.")]

            self._begin_jog(
                direction=1 if direction == "forward" else -1,
                distance_mm=distance_mm,
                speed_mm_per_min=speed_mm_per_min,
            )
            return [self._status("jogging", "Jog started.")]

        if cmd == "start_test":
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Test speed must be greater than zero.")]
            if self.state in {"homing", "jogging", "running"}:
                return [self._error("busy", "Cannot start a test while motion is active.")]

            if not self._homed:
                self._begin_homing(now, pending_run_speed_mm_per_min=speed_mm_per_min)
                return [self._status("homing", "Homing before test.")]

            self._start_running(now, speed_mm_per_min)
            return [self._status("running", "Pull started at %.3f mm/min." % speed_mm_per_min)]

        if cmd == "stop":
            if self.state in {"homing", "jogging", "running"}:
                self._stop_motion()
                return [self._sample(now), self._status("idle", "Motion stopped.")]
            return [self._status(self.state, "Stop ignored.")]

        if cmd == "estop":
            self.state = "estop"
            self._stop_motion()
            return [self._sample(now), self._status("estop", "Emergency stop triggered.")]

        return [self._error("unknown_command", "Unsupported command: %r" % cmd)]

    def poll(self, now):
        self._advance_motion(now)
        messages = []
        sample_emitted = False
        if now - self._last_sample_time >= self.sample_interval_s:
            self._last_sample_time = now
            messages.append(self._sample(now))
            sample_emitted = True
        if self._pending_status_message is not None:
            if not sample_emitted:
                messages.append(self._sample(now))
            messages.append(self._status("idle", self._pending_status_message))
            self._pending_status_message = None
        return messages

    def _begin_homing(self, now, pending_run_speed_mm_per_min):
        self._stop_motion()
        self.state = "homing"
        self._home_phase = "seek_fast"
        self._speed_mm_per_min = 240.0
        self._motion_direction = -1
        self._pending_run_speed_mm_per_min = pending_run_speed_mm_per_min
        self._last_motion_time = now

    def _begin_jog(self, direction, distance_mm, speed_mm_per_min):
        self._stop_motion()
        self.state = "jogging"
        self._motion_direction = direction
        self._speed_mm_per_min = speed_mm_per_min
        self._jog_remaining_mm = distance_mm

    def _start_running(self, now, speed_mm_per_min):
        self._stop_motion()
        self.state = "running"
        self._motion_direction = 1
        self._speed_mm_per_min = speed_mm_per_min
        self._run_started_at = now
        self._last_motion_time = now

    def _stop_motion(self):
        self._speed_mm_per_min = 0.0
        self._jog_remaining_mm = 0.0
        self._home_phase = None
        self._pending_run_speed_mm_per_min = None

    def _advance_motion(self, now):
        if self._last_motion_time == 0.0:
            self._last_motion_time = now
            return

        delta_s = max(0.0, now - self._last_motion_time)
        self._last_motion_time = now
        if delta_s <= 0.0:
            return

        if self.state == "running":
            self._absolute_position_mm += self._speed_mm_per_min * delta_s / 60.0
            return

        if self.state == "jogging":
            requested_delta = self._speed_mm_per_min * delta_s / 60.0
            actual_delta = min(self._jog_remaining_mm, requested_delta)
            self._absolute_position_mm += actual_delta * self._motion_direction
            self._jog_remaining_mm -= actual_delta
            if self._jog_remaining_mm <= 1e-6:
                self.state = "idle"
                self._stop_motion()
                self._pending_status_message = "Jog complete."
            return

        if self.state != "homing":
            return

        if self._home_phase == "seek_fast":
            delta = self._speed_mm_per_min * delta_s / 60.0
            self._absolute_position_mm = max(0.0, self._absolute_position_mm - delta)
            if self._absolute_position_mm <= 0.0:
                self._home_phase = "backoff"
                self._speed_mm_per_min = 60.0
                self._motion_direction = 1
        elif self._home_phase == "backoff":
            delta = self._speed_mm_per_min * delta_s / 60.0
            self._absolute_position_mm += delta
            if self._absolute_position_mm >= 1.0:
                self._home_phase = "seek_slow"
                self._motion_direction = -1
        elif self._home_phase == "seek_slow":
            delta = self._speed_mm_per_min * delta_s / 60.0
            self._absolute_position_mm = max(0.0, self._absolute_position_mm - delta)
            if self._absolute_position_mm <= 0.0:
                self._absolute_position_mm = 0.0
                self._homed = True
                self._zero_reference_mm = self._absolute_position_mm
                pending_speed = self._pending_run_speed_mm_per_min
                self._stop_motion()
                if pending_speed is not None:
                    self._pending_run_speed_mm_per_min = None
                    self._start_running(now, pending_speed)
                else:
                    self.state = "idle"
                    self._pending_status_message = "Homing complete."

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
