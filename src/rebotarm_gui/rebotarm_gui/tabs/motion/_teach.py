"""MotionTab teach mixin — extracted from motion_tab.py."""

# ruff: noqa: F401,F403,F405
from __future__ import annotations

from ._common import *  # noqa: F403
from ._common import (  # noqa: F401
    _config_dir,
    _dict_to_pose,
    _INVALID_GENERIC_STEP_IDS,
    _pose_msg_to_dict,
    _preload_path,
)


class MotionTabTeachMixin:
    """Teach concern methods for MotionTab."""

    def _set_freedrive_checked(self, checked: bool) -> None:
        self._btn_freedrive.blockSignals(True)
        self._btn_freedrive.setChecked(checked)
        self._btn_freedrive.blockSignals(False)


    def _on_freedrive_toggled(self, checked: bool) -> None:
        if self._running:
            self._set_freedrive_checked(False)
            self._lbl_status.setText("Freedrive indisponível durante a execução.")
            return
        if not self._robot_ready:
            self._set_freedrive_checked(False)
            self._lbl_status.setText("Freedrive indisponível: braço sem prontidão operacional.")
            return
        if checked:
            self._lbl_status.setText("Ativando freedrive...")
            self._bridge.gravity_start()
        else:
            self._bridge.gravity_stop()


    def _on_teach_service_result(self, label: str, success: bool, message: str) -> None:
        if label not in ("gravity_start", "gravity_stop"):
            return
        if success:
            if label == "gravity_start":
                self._lbl_status.setText("Freedrive ativo — mova o braço com a mão.")
            else:
                self._lbl_status.setText("Freedrive desativado.")
            return
        # service failed: undo the toggle to reflect the real arm state
        self._set_freedrive_checked(label == "gravity_stop")
        self._lbl_status.setText(f"Falha no freedrive: {message}")


    def _on_safety_step_result(
        self, label: str, success: bool, message: str
    ) -> None:
        pending = self._pending_safety_dispatch
        if pending is None or label != pending[2]:
            return
        self._pending_safety_dispatch = None
        if not self._running:
            return
        if not success:
            self._finish(
                ok=False,
                message=f"Falha ao ativar perfil de segurança: {message}",
            )
            return
        op, total, _ = pending
        self._dispatch_async_opcode(op, total)


    def _teach_current_point(self) -> None:
        if self._running:
            return
        pose_msg = self._bridge.tcp_pose_fk() or self._bridge.last_reached_pose
        pose_dict = _pose_msg_to_dict(pose_msg)
        if pose_dict is None:
            QMessageBox.information(
                self,
                "Pose indisponível",
                "Não foi possível obter a pose atual do TCP.\n"
                "Verifique se o braço está conectado e publicando joint_states.",
            )
            return
        n = sum(1 for s in self._program.steps if s.label.startswith("Ponto ")) + 1
        step = MoveStep(pose=pose_dict, duration=2.0, label=f"Ponto {n}")
        # same mutation pattern as the editor's add-step: undo snapshot,
        # in-place append, refresh, changed
        self._push_undo_snapshot()
        self._program.steps.append(step)
        self._editor.set_steps(self._program.steps)
        self._on_program_changed()
        self._lbl_status.setText(
            f"Ponto {n} ensinado: ({pose_dict['x']:.3f}, {pose_dict['y']:.3f}, "
            f"{pose_dict['z']:.3f})"
        )


