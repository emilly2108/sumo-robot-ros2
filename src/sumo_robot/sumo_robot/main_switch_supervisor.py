#!/usr/bin/env python3
"""Fail-safe main switch controller for the systemd ROS service."""

import logging
import os
import signal
import subprocess
import time

from gpiozero import Button

SERVICE = "sumo-robot.service"
SYSTEMCTL = "/usr/bin/systemctl"
POLL = 0.05
DEBOUNCE = 0.15
running = True


def request_exit(signum, frame):
    global running
    running = False


def service_command(action, timeout=25):
    return subprocess.run(
        [SYSTEMCTL, action, SERVICE], check=False, timeout=timeout,
        capture_output=True, text=True,
    )


def active():
    return service_command("is-active", 5).returncode == 0


def switch_state(start_input, stop_input):
    start = start_input.is_pressed
    stop = stop_input.is_pressed
    if start and not stop:
        return "START"
    if stop and not start:
        return "STOP"
    return "INVALID"


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)
    start_pin = int(os.environ["SUMO_START_GPIO"])
    stop_pin = int(os.environ["SUMO_STOP_GPIO"])
    if start_pin == stop_pin or {start_pin, stop_pin} & {23, 27}:
        raise ValueError("main switch pins must differ and must not use READY/FIGHT GPIO 23/27")
    start_input = Button(start_pin, pull_up=True)
    stop_input = Button(stop_pin, pull_up=True)
    state = "REQUIRE_STOP"
    stable = None
    changed_at = time.monotonic()
    logging.info("supervisor started; waiting for confirmed STOP; ROS active=%s", active())
    if active():
        result = service_command("stop")
        if result.returncode or active():
            logging.error("startup STOP failed: %s", result.stderr.strip())
            state = "FAULT_LATCHED"
    try:
        while running:
            raw = switch_state(start_input, stop_input)
            now = time.monotonic()
            if raw != stable:
                stable, changed_at = raw, now
            if now - changed_at < DEBOUNCE:
                time.sleep(POLL)
                continue
            if raw == "INVALID":
                if state == "RUNNING":
                    logging.error("invalid switch contacts; requesting STOP")
                    state = "STOPPING"
                    result = service_command("stop")
                    state = "REQUIRE_STOP" if result.returncode == 0 and not active() else "FAULT_LATCHED"
                time.sleep(POLL)
                continue
            if raw == "STOP":
                if state in ("RUNNING", "STARTING", "STOPPING") or active():
                    result = service_command("stop")
                    if result.returncode or active():
                        logging.error("STOP failed: %s", result.stderr.strip())
                        state = "FAULT_LATCHED"
                    else:
                        logging.info("ROS stopped")
                        state = "ARMED"
                else:
                    state = "ARMED"
            elif raw == "START" and state == "ARMED":
                state = "STARTING"
                result = service_command("start", 20)
                if result.returncode or not active():
                    logging.error("START failed: %s", result.stderr.strip())
                    state = "FAULT_LATCHED"
                else:
                    logging.info("ROS started")
                    state = "RUNNING"
            elif state == "RUNNING" and not active():
                logging.error("ROS exited unexpectedly; require STOP before restart")
                state = "FAULT_LATCHED"
            time.sleep(POLL)
    finally:
        start_input.close()
        stop_input.close()
        result = service_command("stop")
        if result.returncode:
            logging.error("shutdown STOP failed: %s", result.stderr.strip())


if __name__ == "__main__":
    main()
