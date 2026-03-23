class HardwareTesterBackend:
    def __init__(self):
        self.state = "idle"
        self.startup_message = (
            "Hardware backend placeholder active."
        )

    def handle_command(self, command, now):
        _ = command
        _ = now
        return [
            {
                "type": "error",
                "code": "not_implemented",
                "message": "Hardware control is not implemented yet.",
            }
        ]

    def poll(self, now):
        _ = now
        return []
