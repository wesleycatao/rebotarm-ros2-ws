"""Digital I/O bridge for the Raspberry Pi GPIO.

Runs on the Raspberry Pi (where the GPIO hardware lives) and exposes the
digital inputs **and** outputs as a ROS topic so the GUI works regardless of
where it runs (Pi or a laptop on the same ROS network).

Each I/O point is identified by a stable logical *id* (``IN1``..``IN5`` for
the opto-isolated inputs, ``OUT1``..``OUT5`` for the relay outputs) and carries
a user *tag*, an *enabled* flag, the current *value* and an *activation_count*
(rising edges for inputs, energize commands for outputs).

Topics
------
``/<namespace>/dio/state``  (``rebotarm_msgs/DigitalIOState``)
    Snapshot of all configured I/O points at ``publish_rate`` Hz.

Services
--------
``/<namespace>/dio/set_output``  (``rebotarm_msgs/SetDigitalOutput``)
    Drive a relay output (``OUT*``) to a given level.
``/<namespace>/dio/set_config``  (``rebotarm_msgs/SetDigitalIOConfig``)
    Update the tag / enabled flag of any I/O point.

The node uses ``gpiozero`` when available on real Pi hardware. Otherwise it
falls back to a *dry-run* mode that keeps the topic alive and simulates the
first input (``IN1``) toggling every few seconds so ``WAIT_FOR_SIGNAL`` can be
exercised on a development laptop.

Parameters
----------
``input_pins`` : list of int
    BCM pin numbers for the 5 inputs (default ``[17, 18, 27, 22, 23]``).
``output_pins`` : list of int
    BCM pin numbers for the 5 outputs (default ``[5, 6, 12, 16, 26]``).
``pull_up`` : bool
    Use the internal pull-up resistor for the inputs (default ``True``).
``publish_rate`` : float
    Publish rate in Hz (default ``20.0``).
``namespace`` : str
    ROS namespace for the topics/services (default ``rebotarm``).
``config_path`` : str
    Path to the JSON file persisting tags / enabled flags.

Usage
-----
    ros2 run rebotarmcontroller DioBridge
"""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import DigitalIO, DigitalIOState
from rebotarm_msgs.srv import SetDigitalIOConfig, SetDigitalOutput

DEFAULT_INPUT_PINS = [17, 18, 27, 22, 23]
DEFAULT_OUTPUT_PINS = [5, 6, 12, 16, 26]
INPUT_IDS = [f"IN{i}" for i in range(1, 6)]
OUTPUT_IDS = [f"OUT{i}" for i in range(1, 6)]

# Direction constants (mirror DigitalIO.msg)
_DIR_INPUT = DigitalIO.DIR_INPUT
_DIR_OUTPUT = DigitalIO.DIR_OUTPUT


def _default_config_path() -> str:
    base = os.environ.get("XDG_CONFIG_HOME")
    root = Path(base) if base else Path.home() / ".config"
    return str(root / "rebotarm_dio" / "io_config.json")


class DioBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_dio_bridge")

        self.declare_parameter("input_pins", DEFAULT_INPUT_PINS)
        self.declare_parameter("output_pins", DEFAULT_OUTPUT_PINS)
        self.declare_parameter("pull_up", True)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("namespace", "rebotarm")
        self.declare_parameter("config_path", _default_config_path())

        in_pins = [int(p) for p in self.get_parameter("input_pins").value]
        out_pins = [int(p) for p in self.get_parameter("output_pins").value]
        self._pull_up = bool(self.get_parameter("pull_up").value)
        ns = str(self.get_parameter("namespace").value).strip().strip("/")
        rate = float(self.get_parameter("publish_rate").value)
        self._config_path = str(self.get_parameter("config_path").value)

        # Pad/truncate to the fixed 5+5 set so ids stay stable.
        in_pins = (in_pins + DEFAULT_INPUT_PINS)[:5]
        out_pins = (out_pins + DEFAULT_OUTPUT_PINS)[:5]
        self._inputs: Dict[str, int] = dict(zip(INPUT_IDS, in_pins))
        self._outputs: Dict[str, int] = dict(zip(OUTPUT_IDS, out_pins))

        # Per-point runtime state.
        self._tags: Dict[str, str] = {}
        self._enabled: Dict[str, bool] = {}
        self._value: Dict[str, bool] = {iid: False for iid in INPUT_IDS + OUTPUT_IDS}
        self._count: Dict[str, int] = {iid: 0 for iid in INPUT_IDS + OUTPUT_IDS}
        self._lock = threading.Lock()
        self._load_config()

        self._pub = self.create_publisher(DigitalIOState, f"/{ns}/dio/state", 10)
        self.create_service(SetDigitalOutput, f"/{ns}/dio/set_output", self._set_output_cb)
        self.create_service(SetDigitalIOConfig, f"/{ns}/dio/set_config", self._set_config_cb)

        # Hardware binding.
        self._in_dev: Dict[str, object] = {}
        self._out_dev: Dict[str, object] = {}
        self._dry_run = False
        self._bind_hardware()

        period = 1.0 / rate if rate > 0 else 0.05
        self._sim_t = 0.0
        self.create_timer(period, self._tick)

    # ------------------------------------------------------------------
    # config persistence
    # ------------------------------------------------------------------
    def _load_config(self) -> None:
        try:
            data = json.loads(Path(self._config_path).read_text(encoding="utf-8"))
        except Exception:
            data = {}
        for iid in INPUT_IDS + OUTPUT_IDS:
            entry = data.get(iid, {}) or {}
            self._tags[iid] = str(entry.get("tag", ""))
            self._enabled[iid] = bool(entry.get("enabled", True))

    def _save_config(self) -> None:
        data = {
            iid: {"tag": self._tags.get(iid, ""), "enabled": bool(self._enabled.get(iid, True))}
            for iid in INPUT_IDS + OUTPUT_IDS
        }
        try:
            Path(self._config_path).parent.mkdir(parents=True, exist_ok=True)
            Path(self._config_path).write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception as exc:
            self.get_logger().warning(f"não foi possível salvar config DIO: {exc}")

    # ------------------------------------------------------------------
    # hardware
    # ------------------------------------------------------------------
    def _bind_hardware(self) -> None:
        try:
            from gpiozero import DigitalInputDevice, DigitalOutputDevice  # type: ignore

            for iid, pin in self._inputs.items():
                dev = DigitalInputDevice(pin, pull_up=self._pull_up)
                dev.when_activated = lambda _d=None, _i=iid: self._on_input_rise(_i)
                self._in_dev[iid] = dev
            for iid, pin in self._outputs.items():
                self._out_dev[iid] = DigitalOutputDevice(pin)
            self.get_logger().info(
                f"GPIO bound — entradas {list(self._inputs.values())}, "
                f"saídas {list(self._outputs.values())} (pull_up={self._pull_up})"
            )
        except Exception as exc:
            self._dry_run = True
            self.get_logger().warning(
                f"gpiozero indisponível ou sem hardware GPIO ({exc}); "
                "rodando em modo dry-run."
            )

    def _on_input_rise(self, iid: str) -> None:
        with self._lock:
            self._count[iid] = self._count.get(iid, 0) + 1

    def _read_input(self, iid: str) -> bool:
        dev = self._in_dev.get(iid)
        if dev is None:
            return False
        try:
            return bool(dev.value)
        except Exception:
            return False

    def _write_output(self, iid: str, value: bool) -> None:
        dev = self._out_dev.get(iid)
        if dev is not None:
            try:
                dev.value = bool(value)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def _set_output_cb(self, req, resp) -> None:
        iid = str(req.id).strip().upper()
        with self._lock:
            if iid not in self._outputs:
                resp.success = False
                resp.message = f"saída desconhecida: {req.id}"
                return resp
            if not self._enabled.get(iid, True):
                resp.success = False
                resp.message = f"{iid} desabilitada"
                return resp
            value = bool(req.value)
            self._value[iid] = value
            if value:
                self._count[iid] = self._count.get(iid, 0) + 1
        self._write_output(iid, value)
        resp.success = True
        resp.message = f"{iid}={'ON' if value else 'OFF'}"
        return resp

    def _set_config_cb(self, req, resp) -> None:
        iid = str(req.id).strip().upper()
        with self._lock:
            if iid not in (INPUT_IDS + OUTPUT_IDS):
                resp.success = False
                resp.message = f"I/O desconhecido: {req.id}"
                return resp
            self._tags[iid] = str(req.tag)
            self._enabled[iid] = bool(req.enabled)
            # Turning a relay off when disabled is safer.
            if not req.enabled and iid in self._outputs:
                self._value[iid] = False
                self._write_output(iid, False)
        self._save_config()
        resp.success = True
        resp.message = f"{iid} atualizado"
        return resp

    # ------------------------------------------------------------------
    # publish
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            # Snapshot mutable state under the lock.
            tags = dict(self._tags)
            enabled = dict(self._enabled)
            counts = dict(self._count)
            out_values = {iid: self._value[iid] for iid in self._outputs}

        # Dry-run: toggle IN1 every ~3 s so rising-edge waits can be tested.
        if self._dry_run:
            self._sim_t += 1.0 / 20.0
            new = (int(self._sim_t / 3.0) % 2) == 0
            with self._lock:
                prev = self._value["IN1"]
                self._value["IN1"] = new
                if new and not prev:
                    self._count["IN1"] = self._count.get("IN1", 0) + 1
                counts["IN1"] = self._count["IN1"]

        msg = DigitalIOState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for iid in INPUT_IDS:
            msg.ios.append(self._make_io(
                iid, _DIR_INPUT, self._inputs[iid],
                self._read_input(iid) if enabled.get(iid, True) else False,
                tags.get(iid, ""), enabled.get(iid, True), counts.get(iid, 0),
            ))
        for iid in OUTPUT_IDS:
            val = bool(out_values.get(iid, False)) if enabled.get(iid, True) else False
            msg.ios.append(self._make_io(
                iid, _DIR_OUTPUT, self._outputs[iid], val,
                tags.get(iid, ""), enabled.get(iid, True), counts.get(iid, 0),
            ))
        self._pub.publish(msg)

    @staticmethod
    def _make_io(iid, direction, pin, value, tag, enabled, count) -> DigitalIO:
        io = DigitalIO()
        io.direction = direction
        io.id = iid
        io.tag = tag
        io.pin = int(pin)
        io.enabled = bool(enabled)
        io.value = bool(value)
        io.activation_count = int(count)
        return io


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        # Ensure relays are de-energized on exit.
        try:
            for iid in OUTPUT_IDS:
                node._write_output(iid, False)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
