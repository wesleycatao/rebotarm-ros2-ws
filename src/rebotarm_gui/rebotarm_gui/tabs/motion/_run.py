"""MotionTab run mixin — extracted from motion_tab.py."""

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


class MotionTabRunMixin:
    """Run concern methods for MotionTab."""

    def _run(self) -> None:
        self._start_run(0)


    def _run_from_selected(self) -> None:
        start_index = self._editor.selected_index()
        if start_index < 0:
            QMessageBox.information(
                self,
                "Nenhuma etapa selecionada",
                "Selecione uma etapa na lista antes de executar a partir dela.",
            )
            return
        self._start_run(start_index)


    def _execution_program(self, start_index: int) -> Program:
        """Return the top-level suffix selected for this execution."""
        if not (0 <= start_index < len(self._program.steps)):
            raise IndexError("etapa inicial fora do programa")
        parameters = dict(self._program.parameters)
        if start_index > 0:
            # Runtime-only marker: the full released recipe remains the
            # integrity authority, while envelope translation may legitimately
            # omit vision-relative steps that precede this guided resume.
            parameters["_execution_subset_start_index"] = int(start_index)
        return Program(
            name=self._program.name,
            version=self._program.version,
            speed_scale=float(self._speed_spinbox.value()),
            revision=self._program.revision,
            recipe_status=self._program.recipe_status,
            released_by=self._program.released_by,
            released_at_utc=self._program.released_at_utc,
            release_hash=self._program.release_hash,
            parameters=parameters,
            steps=list(self._program.steps[start_index:]),
        )


    def _compile_execution(self, start_index: int):
        """Compile a suffix and map its highlights back to the full editor."""
        program = self._execution_program(start_index)
        ops, raw_meta = program.flatten_with_meta()
        meta = dict(raw_meta)
        meta["pc_to_root_index"] = {
            int(pc): int(root_index) + start_index
            for pc, root_index in raw_meta.get("pc_to_root_index", {}).items()
        }
        return program, ops, meta, build_label_index(ops)


    def _start_run(self, start_index: int) -> None:
        if self._running:
            return
        self._sync_editors_to_program()
        if self._production_mode and not self._dry_run:
            released, reason = self._program.verify_release()
            if not released:
                QMessageBox.critical(
                    self,
                    "Modo Produção",
                    f"Execução bloqueada: {reason}.",
                )
                return
            calibration_ok, calibration_reason, calibration_reference = (
                self._production_prerequisite_status()
            )
            if not calibration_ok:
                QMessageBox.critical(
                    self,
                    "Modo Produção",
                    "Execução bloqueada pela calibração da célula:\n\n"
                    f"{calibration_reason}",
                )
                return
            released_calibration = self._program.parameters.get(
                "_calibration_profile", {}
            )
            if not isinstance(released_calibration, dict):
                released_calibration = {}
            released_hash = str(
                released_calibration.get("integrity_hash", "")
            )
            active_hash = str(calibration_reference.get("integrity_hash", ""))
            if not released_hash or released_hash != active_hash:
                QMessageBox.critical(
                    self,
                    "Modo Produção",
                    "Execução bloqueada: a receita não foi liberada com o "
                    "perfil de calibração atualmente ativo. Saia do Modo "
                    "Produção, valide e libere uma nova revisão.",
                )
                return
        if (
            not self._dry_run
            and self._collision_planning_required
            and not self._bridge.collision_planner_ready()
        ):
            QMessageBox.critical(
                self,
                "Planejamento seguro indisponível",
                "Esta estação exige o MoveIt /move_action disponível. "
                "Nenhum movimento foi enviado.",
            )
            return
        if not self._program.steps:
            QMessageBox.information(self, "Programa vazio", "Adicione etapas antes de executar.")
            return
        try:
            run_program, ops, meta, labels = self._compile_execution(start_index)
        except Exception as exc:
            QMessageBox.critical(self, "Programa inválido", str(exc))
            return
        if not any(op and op[0] in ACTION_OPS for op in ops):
            QMessageBox.information(self, "Programa vazio", "O programa não gera nenhum movimento.")
            return

        if not self._preflight_gate(run_program):
            return

        ok, reason = self._check_execution_preconditions()
        if not ok:
            QMessageBox.warning(self, "Não é possível executar", reason)
            return

        n_steps = run_program.estimated_steps()
        n_cmds = run_program.estimated_commands()
        speed = self._speed_spinbox.value()
        global_override = self._bridge.speed_override
        dynamic_ceiling = self._dynamic_override_ceiling()
        physical_override = global_override * dynamic_ceiling
        effective_speed = speed * physical_override
        state = self._bridge.state_machine or "desconhecido"
        mode_text = []
        if self._dry_run:
            mode_text.append("modo SIMULAÇÃO")
        if self._step_by_step:
            mode_text.append("modo PASSO A PASSO")
        if start_index > 0:
            mode_text.append(f"a partir da etapa {start_index + 1}")
        mode_line = f"<br>• Modo: {', '.join(mode_text)}" if mode_text else ""
        scene_line = ""
        if (
            self._cell_scene_store is not None
            and self._cell_scene_store.scene.configured
        ):
            scene_line = (
                "<br>• Cena da célula: "
                f"<b>{len(self._cell_scene_store.scene.enabled_obstacles)} volume(s), "
                f"sincronizada; "
                f"{len(self._cell_scene_store.external_geometries)} externa(s) "
                "monitorada(s)</b>"
            )
        reply = QMessageBox.question(
            self,
            "Confirmar execução",
            f"<b>{self._program.name}</b><br>"
            f"• {n_steps} etapa(s)<br>"
            f"• {n_cmds} comando(s) de movimento/gripper/I/O<br>"
            f"• Escala do programa: {speed:.2f}x<br>"
            f"• Controle global: {global_override * 100:.0f}%<br>"
            f"• Teto dinâmico da receita: {dynamic_ceiling * 100:.0f}% físico<br>"
            f"• Velocidade efetiva: {effective_speed:.2f}x<br>"
            f"• Estado do braço: <b>{state}</b>"
            f"{scene_line}"
            f"{mode_line}<br><br>"
            "Deseja executar o programa?",
        )
        if reply != QMessageBox.Yes:
            return

        if not self._begin_audit_run(run_program, start_index):
            QMessageBox.critical(
                self,
                "Auditoria indisponível",
                "A execução não foi iniciada porque o registro de auditoria não pôde "
                f"ser salvo em:\n{self._audit_log.path}\n\n{self._audit_log.last_error}",
            )
            return

        self._vm_ops = ops
        self._vm_labels = labels
        self._vm_meta = meta
        self._vm_action_count = meta.get("action_count", n_cmds)
        self._vm_action_seen = 0
        self._vm_pc = 0
        self._vm_loop_counters = {}
        self._vm_seq = 0
        self._vm_pending_tag = None
        self._vm_pending_gripper = False
        self._step_waiting = False
        self._step_advance_pending = False
        self._speed_scale = float(self._speed_spinbox.value())
        self._running = True
        self._paused = False
        self._pause_requested = False
        self._recovery_required = False
        self._recovery_reason = ""
        self._bridge.set_recipe_execution_scope(not self._dry_run)
        self._record_joints = self._chk_record.isChecked()
        self._joint_history.clear()
        self._joint_history_markers.clear()
        self._joint_recording_active = self._record_joints
        self._joint_recording_start = time.monotonic()
        if self._dry_run and self._record_joints:
            self._dry_run_record_timer.start(self._dry_run_record_interval_ms)
        self._btn_run.setEnabled(False)
        self._btn_run_selected.setEnabled(False)
        self._btn_preview.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_pause.setEnabled(True)
        self._btn_resume.setEnabled(False)
        self._btn_advance.setEnabled(False)
        self._btn_freedrive.setEnabled(False)
        self._btn_teach.setEnabled(False)
        self._editor.set_edit_enabled(False)
        self._flow_widget.set_edit_enabled(False)
        self._progress.setValue(0)
        self._progress.setVisible(True)
        self._chk_dry_run.setEnabled(False)
        self._chk_step.setEnabled(False)
        self._act_record_joints.setEnabled(False)
        self._act_preload.setEnabled(False)
        self._update_undo_buttons()
        self._vm_step()


    def _vm_step(self) -> None:
        """Dispatch synchronous control opcodes until an async one is hit."""
        if not self._running or self._paused:
            return
        ops = self._vm_ops
        total = len(ops)
        iters = 0
        while 0 <= self._vm_pc < len(ops):
            iters += 1
            if iters > VM_ITER_CAP:
                self._finish(ok=False, message="Limite de iterações excedido (possível loop infinito).")
                return
            op = ops[self._vm_pc]
            tag = op[0] if op else ""

            if tag == OP_LABEL:
                self._vm_pc += 1
                continue
            if tag == OP_GOTO:
                target = self._vm_labels.get(op[1])
                if target is None:
                    self._finish(ok=False, message=f"Rótulo desconhecido: {op[1]}")
                    return
                self._vm_pc = target
                continue
            if tag == OP_BRANCH:
                cond, name, polarity = op[1], op[2], op[3]
                try:
                    res = evaluate(cond, self._bridge)
                except Exception as exc:
                    self._finish(ok=False, message=f"Erro ao avaliar condição: {exc}")
                    return
                if bool(res) == bool(polarity):
                    target = self._vm_labels.get(name)
                    if target is None:
                        self._finish(ok=False, message=f"Rótulo desconhecido: {name}")
                        return
                    self._vm_pc = target
                else:
                    self._vm_pc += 1
                continue
            if tag == OP_LOOP_INIT:
                self._vm_loop_counters[op[1]] = 0
                self._vm_pc += 1
                continue
            if tag == OP_LOOP_BACK:
                loop_id, count, name = op[1], op[2], op[3]
                self._vm_loop_counters[loop_id] = self._vm_loop_counters.get(loop_id, 0) + 1
                c = self._vm_loop_counters[loop_id]
                if count <= 0 or c < count:
                    target = self._vm_labels.get(name, self._vm_pc)
                    self._vm_pc = target
                else:
                    self._vm_pc += 1
                continue

            # digital output: fire-and-forget, advance immediately
            if tag == OP_OUTPUT:
                io_id, action, step_id = op[1], op[2], op[3]
                self._record_step_marker(step_id)
                self._vm_action_seen += 1
                self._update_progress_bar()
                self._highlight_current_step()
                if not self._audit_begin_step(op):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar início da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                self._dispatch_output(io_id, action)
                if not self._audit_finish_step(
                    "completed" if self._dry_run else "dispatched"
                ):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar fim da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                self._vm_pc += 1
                continue

            # async opcodes: highlight, progress, maybe step-by-step pause, then dispatch
            if tag in (OP_MOVE, OP_GRIPPER, OP_WAIT, OP_WAIT_SM, OP_WAIT_SIG):
                self._record_step_marker(op[-1])
                self._vm_action_seen += 1
                self._update_progress_bar()
                self._highlight_current_step()
                if self._step_by_step and not self._step_advance_pending:
                    self._pause_for_step(tag)
                    return
                self._step_advance_pending = False
                self._step_waiting = False
                if not self._audit_begin_step(op):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar início da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                if self._dry_run:
                    self._dispatch_async_opcode(op, total)
                    return
                step_id = str(op[-1])
                label = f"safety_set_step({step_id or 'default'})"
                self._pending_safety_dispatch = (op, total, label)
                self._lbl_status.setText(
                    f"Ativando perfil de segurança da etapa {step_id} ..."
                )
                if not self._bridge.safety_set_step(step_id):
                    if self._running and self._pending_safety_dispatch is not None:
                        self._pending_safety_dispatch = None
                        self._finish(
                            ok=False,
                            message="Serviço do perfil de segurança indisponível.",
                        )
                return

            # unknown opcode: skip defensively
            self._vm_pc += 1

        # reached the end
        self._finish(ok=True, message="Programa concluído.")


    def _highlight_current_step(self) -> None:
        root_index = self._vm_meta.get("pc_to_root_index", {}).get(self._vm_pc)
        self._editor.highlight_row(root_index)
        self._flow_widget.set_active_root_index(root_index)


    def _update_progress_bar(self) -> None:
        total = max(1, self._vm_action_count)
        pct = int(100 * self._vm_action_seen / total)
        self._progress.setValue(min(pct, 100))


    def _pause_for_step(self, tag: str) -> None:
        self._step_waiting = True
        self._step_advance_pending = False
        self._btn_advance.setEnabled(True)
        kind_label = {
            OP_MOVE: "movimento",
            OP_GRIPPER: "gripper",
            OP_WAIT: "WAIT",
            OP_WAIT_SM: "WAIT estado",
            OP_WAIT_SIG: "WAIT sinal",
        }.get(tag, tag)
        self._lbl_status.setText(f"Pausado no {kind_label}. Clique em Avançar.")


    def _vm_advance(self) -> None:
        if not self._audit_finish_step("success"):
            self._finish(
                ok=False,
                message=f"Falha ao registrar fim da etapa: {self._audit_log.last_error}",
                result="failure",
            )
            return
        self._vm_pc += 1
        self._vm_step()


    def _dispatch_async_opcode(self, op, total: int) -> None:
        tag = op[0]
        if tag == OP_MOVE:
            self._dispatch_move(op, total)
        elif tag == OP_GRIPPER:
            self._dispatch_gripper(op, total)
        elif tag == OP_WAIT:
            self._dispatch_wait_time(op, total)
        elif tag == OP_WAIT_SM:
            self._dispatch_wait_sm(op, total)
        elif tag == OP_WAIT_SIG:
            self._dispatch_wait_sig(op, total)


    def _dispatch_move(self, op, total: int) -> None:
        pose_dict, raw_dt = op[1], op[2]
        # A released recipe maps 100% of the operator control to its certified
        # joint-demand ceiling.  The bridge still applies the global override,
        # therefore increasing duration by 1/ceiling composes both scales
        # without altering the Cartesian path.
        dynamic_ceiling = self._dynamic_override_ceiling()
        dt = scale_duration(raw_dt, self._speed_scale) / dynamic_ceiling
        self._vm_seq += 1
        tag = f"wp{self._vm_seq}"
        self._vm_pending_tag = tag
        self._lbl_status.setText(f"Movendo (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            effective_dt = self._bridge.effective_motion_duration(dt)
            delay_ms = int(min(max(0.05, effective_dt), 1.0) * 1000)
            self._lbl_status.setText(f"[SIMULAÇÃO] Movendo (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(delay_ms, self._vm_advance)
            return
        use_collision_planner = bool(
            self._collision_planning_required and not self._dry_run
        )
        if use_collision_planner:
            sent = self._bridge.move_to_pose_planned_tagged(
                tag,
                _dict_to_pose(pose_dict),
                dt,
                program_speed_scale=self._speed_scale,
                recipe_speed_ceiling=dynamic_ceiling,
            )
        else:
            sent = self._bridge.move_to_pose_tagged(
                tag, _dict_to_pose(pose_dict), dt
            )
        if not sent:
            # The bridge normally emits the tagged failure synchronously. Keep
            # this fallback for bridges/mocks that only return ``False``.
            if self._running and not self._paused:
                message = "Action server indisponível."
                if self._dry_run:
                    self._finish(ok=False, message=message)
                else:
                    self._enter_recovery(message)


    def _dispatch_gripper(self, op, total: int) -> None:
        position_mm, max_effort, _timeout = (
            float(op[1]),
            float(op[2]),
            float(op[3]),
        )
        self._vm_pending_gripper = True
        self._wait_kind = "gripper"
        self._wait_deadline = (time.monotonic() + _timeout) if _timeout > 0 else None
        self._lbl_status.setText(f"Gripper {position_mm:.1f} mm (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] Gripper {position_mm:.1f} mm (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(200, self._vm_advance)
            return
        if not self._bridge.gripper_command(position_mm, max_effort):
            if self._running and not self._paused:
                self._vm_pending_gripper = False
                message = "Action server do gripper indisponível."
                if self._dry_run:
                    self._finish(ok=False, message=message)
                else:
                    self._enter_recovery(message)
        elif self._wait_deadline is not None:
            self._wait_timer.start(POLL_INTERVAL_MS)


    def _dispatch_wait_time(self, op, total: int) -> None:
        secs = float(op[1])
        self._wait_kind = "time"
        self._wait_deadline = time.monotonic() + max(0.0, secs)
        self._lbl_status.setText(f"WAIT {secs:.2f}s (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] WAIT {secs:.2f}s (passo {self._vm_pc + 1}/{total})")
            delay_ms = int(min(max(0.0, secs), 1.0) * 1000)
            QTimer.singleShot(delay_ms, self._vm_advance)
            return
        self._wait_timer.start(int(max(0.0, secs) * 1000))


    def _dispatch_wait_sm(self, op, total: int) -> None:
        state, timeout = op[1], float(op[2])
        self._wait_kind = "sm"
        self._wait_target_state = state
        self._wait_deadline = (time.monotonic() + timeout) if timeout > 0 else None
        self._lbl_status.setText(f"WAIT estado = {state} (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] WAIT estado = {state} (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(300, self._vm_advance)
            return
        self._wait_timer.start(POLL_INTERVAL_MS)


    def _dispatch_output(self, io_id: str, action: str) -> None:
        action_label = {"on": "LIGAR", "off": "DESLIGAR", "toggle": "ALTERNAR"}.get(action, action.upper())
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] OUTPUT {action_label} {io_id}")
            return
        if action == OUTPUT_TOGGLE:
            ios = self._bridge.dio_ios_snapshot()
            io = ios.get(io_id)
            cur = bool(getattr(io, "value", False)) if io is not None else False
            self._bridge.set_dio_output(io_id, not cur)
        elif action == OUTPUT_OFF:
            self._bridge.set_dio_output(io_id, False)
        else:
            self._bridge.set_dio_output(io_id, True)


    def _dispatch_wait_sig(self, op, total: int) -> None:
        io_id, value, edge, timeout = (
            str(op[1]), int(op[2]), str(op[3]), float(op[4])
        )
        self._wait_kind = "sig"
        self._wait_io_id = io_id
        self._wait_value = value
        self._wait_edge = edge
        self._wait_edge_last = self._bridge.signal_by_id(io_id)
        self._wait_deadline = (time.monotonic() + timeout) if timeout > 0 else None
        edge_txt = {"level": "nível", "rising": "subida", "falling": "descida"}.get(edge, edge)
        self._lbl_status.setText(
            f"WAIT {io_id} (borda {edge_txt}) (passo {self._vm_pc + 1}/{total})"
        )
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(
                f"[SIMULAÇÃO] WAIT {io_id} (borda {edge_txt}) (passo {self._vm_pc + 1}/{total})"
            )
            QTimer.singleShot(300, self._vm_advance)
            return
        self._wait_timer.start(POLL_INTERVAL_MS)


    def _on_wait_poll(self) -> None:
        if not self._running:
            return

        if self._wait_kind == "time":
            # single-shot timer reached: handled here for uniformity
            self._wait_timer.stop()
            self._vm_advance()
            return

        if self._wait_kind == "sm":
            if self._bridge.state_machine == self._wait_target_state:
                self._wait_timer.stop()
                self._vm_advance()
                return
        elif self._wait_kind == "sig":
            cur = self._bridge.signal_by_id(self._wait_io_id)
            matched = False
            if self._wait_edge == EDGE_LEVEL:
                matched = cur == self._wait_value
            elif self._wait_edge == EDGE_RISING:
                matched = self._wait_edge_last == 0 and cur == 1
            elif self._wait_edge == EDGE_FALLING:
                matched = self._wait_edge_last == 1 and cur == 0
            self._wait_edge_last = cur
            if matched:
                self._wait_timer.stop()
                self._vm_advance()
                return

        # not yet satisfied: check timeout
        if self._wait_deadline is not None and time.monotonic() >= self._wait_deadline:
            self._wait_timer.stop()
            label = {
                "sm": f"WAIT_FOR_STATE_MACHINE ({self._wait_target_state})",
                "sig": f"WAIT_FOR_SIGNAL ({self._wait_io_id})",
                "gripper": "GRIPPER",
            }.get(self._wait_kind, "WAIT")
            message = f"Timeout em {label}."
            if self._dry_run:
                self._finish(ok=False, message=message)
            else:
                self._enter_recovery(message)


    def _save_recovery_checkpoint(self, reason: str) -> None:
        op = self._vm_ops[self._vm_pc] if 0 <= self._vm_pc < len(self._vm_ops) else ()
        step_id = str(op[-1]) if op else ""
        root_index = int(
            self._vm_meta.get("pc_to_root_index", {}).get(self._vm_pc, 0)
        )
        run = self._audit_run or {}
        checkpoint = RecoveryCheckpoint(
            program_path=str(getattr(self, "_current_path", "") or ""),
            program_hash=str(
                self._program.release_hash or self._program.compute_release_hash()
            ),
            root_step_index=root_index,
            opcode_index=int(self._vm_pc),
            step_id=step_id,
            reason=str(reason),
            run_id=str(run.get("run_id", "")),
        )
        try:
            self._recovery_store.save(checkpoint)
            self._persisted_checkpoint = checkpoint
        except OSError as exc:
            self._lbl_status.setText(f"Falha ao persistir checkpoint: {exc}")


    def _pause(
        self,
        _checked: bool = False,
        *,
        reason: str = "Pausa solicitada pelo operador.",
        recovery: bool = False,
    ) -> None:
        """Pause at the current checkpoint, cancelling any in-flight command."""
        if not self._running or self._paused:
            return
        self._paused = True
        self._pause_requested = True
        self._recovery_required = bool(recovery)
        self._recovery_reason = str(reason)
        if self._wait_timer.isActive():
            remaining_ms = self._wait_timer.remainingTime()
            self._paused_wait_remaining = max(0.0, remaining_ms / 1000.0)
            self._wait_timer.stop()
        else:
            self._paused_wait_remaining = None
        if self._audit_active_step is not None:
            if recovery and self._audit_run is not None:
                active = dict(self._audit_active_step)
                self._append_audit(
                    "failure",
                    run_id=self._audit_run["run_id"],
                    **self._audit_common_fields(),
                    step_sequence=active.get("step_sequence", ""),
                    step_index=active.get("step_index", ""),
                    step_id=active.get("step_id", ""),
                    step_type=active.get("step_type", ""),
                    parameters=active.get("parameters", {}),
                    result="failure",
                    message=str(reason),
                )
                self._audit_finish_step("failure", str(reason))
            else:
                self._audit_finish_step("paused", str(reason))
        if not self._dry_run:
            self._bridge.cancel_active_actions(
                {"move_to_pose", "move_group", "gripper_command"}
            )
        self._vm_pending_tag = None
        self._vm_pending_gripper = False
        self._pending_safety_dispatch = None
        self._save_recovery_checkpoint(reason)
        self._btn_pause.setEnabled(False)
        self._btn_resume.setEnabled(self._robot_ready or self._dry_run)
        self._btn_stop.setEnabled(True)
        self._lbl_status.setText(
            ("RECUPERAÇÃO NECESSÁRIA: " if recovery else "PAUSADO: ") + str(reason)
        )
        self._lbl_progress.setText("RECUPERAR" if recovery else "PAUSADO")


    def _resume(self) -> None:
        if not self._running:
            checkpoint = self._persisted_checkpoint or self._recovery_store.load()
            if checkpoint is None:
                return
            path = Path(checkpoint.program_path)
            if not path.is_file():
                QMessageBox.critical(
                    self,
                    "Recuperação indisponível",
                    "O arquivo original da receita não foi encontrado.",
                )
                return
            try:
                candidate = Program.from_json(path.read_text(encoding="utf-8"))
            except Exception as exc:
                QMessageBox.critical(self, "Recuperação indisponível", str(exc))
                return
            digest = candidate.release_hash or candidate.compute_release_hash()
            if digest != checkpoint.program_hash:
                QMessageBox.critical(
                    self,
                    "Recuperação bloqueada",
                    "A receita foi alterada desde o checkpoint; nenhuma retomada foi feita.",
                )
                return
            if (
                QMessageBox.question(
                    self,
                    "Recuperação guiada",
                    f"Retomar a receita {candidate.name} na etapa "
                    f"{checkpoint.root_step_index + 1}?\n\n"
                    "A etapa será planejada novamente a partir da posição atual.",
                    QMessageBox.Yes | QMessageBox.Cancel,
                    QMessageBox.Cancel,
                )
                != QMessageBox.Yes
            ):
                return
            self.load_program(candidate, "Receita restaurada do checkpoint.")
            self._start_run(checkpoint.root_step_index)
            return
        if not self._paused:
            return
        if not self._dry_run and not self._robot_ready:
            QMessageBox.warning(
                self,
                "Recuperação bloqueada",
                "Aguarde heartbeat, braço habilitado e ausência de falha.",
            )
            return
        if (
            not self._dry_run
            and self._collision_planning_required
            and not self._bridge.collision_planner_ready()
        ):
            QMessageBox.critical(
                self,
                "Recuperação bloqueada",
                "O planejador de colisão não está disponível; a etapa não será retomada.",
            )
            return
        prompt = (
            "A condição de falha foi removida e a célula está livre?\n\n"
            "A etapa interrompida será planejada novamente a partir da posição atual."
            if self._recovery_required
            else "Confirma retomar a etapa interrompida a partir da posição atual?"
        )
        if (
            QMessageBox.question(
                self,
                "Recuperação guiada" if self._recovery_required else "Retomar programa",
                prompt,
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            != QMessageBox.Yes
        ):
            return
        self._paused = False
        self._pause_requested = False
        self._recovery_required = False
        self._btn_pause.setEnabled(True)
        self._btn_resume.setEnabled(False)
        self._lbl_progress.setText("")
        if self._paused_wait_remaining is not None and self._wait_kind:
            remaining = self._paused_wait_remaining
            self._paused_wait_remaining = None
            op = (
                self._vm_ops[self._vm_pc]
                if 0 <= self._vm_pc < len(self._vm_ops)
                else ()
            )
            if op and not self._audit_begin_step(op):
                self._enter_recovery("Falha ao registrar retomada da etapa.")
                return
            self._wait_deadline = time.monotonic() + remaining
            self._wait_timer.start(
                int(remaining * 1000) if self._wait_kind == "time" else POLL_INTERVAL_MS
            )
        else:
            self._vm_step()


    def _enter_recovery(self, reason: str) -> None:
        self._pause(reason=reason, recovery=True)


    def _stop(
        self,
        _checked: bool = False,
        *,
        reason: str = "Execução interrompida.",
        source: str = "operator",
    ) -> None:
        if not self._running:
            return
        self._audit_cancel(source, reason)
        self._running = False
        if self._wait_timer.isActive():
            self._wait_timer.stop()
        if not self._dry_run:
            self._bridge.cancel_active_actions(
                {"move_to_pose", "move_group", "gripper_command"}
            )
        self._finish(ok=False, message=reason, result="canceled")


    def _finish(self, ok: bool, message: str, result: str | None = None) -> None:
        audit_result = result or ("success" if ok else "failure")
        audit_ok = self._audit_finish_run(audit_result, message)
        self._running = False
        self._paused = False
        self._pause_requested = False
        self._recovery_required = False
        self._bridge.set_recipe_execution_scope(False)
        self._vm_pending_tag = None
        self._vm_pending_gripper = False
        self._pending_safety_dispatch = None
        self._wait_timer.stop()
        self._step_waiting = False
        self._step_advance_pending = False
        self._joint_recording_active = False
        self._dry_run_record_timer.stop()
        self._btn_run.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._btn_pause.setEnabled(False)
        self._btn_resume.setEnabled(False)
        self._btn_advance.setEnabled(False)
        self._editor.set_edit_enabled(True)
        self._editor.clear_highlight()
        self._flow_widget.set_edit_enabled(True)
        self._flow_widget.set_active_root_index(None)
        self._progress.setVisible(False)
        self._progress.setValue(0)
        self._chk_dry_run.setEnabled(True)
        self._chk_step.setEnabled(True)
        self._act_record_joints.setEnabled(True)
        self._act_preload.setEnabled(True)
        self._update_undo_buttons()
        if audit_ok:
            self._lbl_status.setText(message)
            self._lbl_progress.setText("OK" if ok else "PARADO")
        else:
            self._lbl_status.setText(
                f"{message} ALERTA: registro de auditoria incompleto "
                f"({self._audit_log.last_error})."
            )
            self._lbl_progress.setText("AUDITORIA!")
        self._btn_plot_joints.setEnabled(bool(self._joint_history))
        self._act_plot_joints.setEnabled(bool(self._joint_history))
        if ok or audit_result == "canceled":
            try:
                self._recovery_store.clear()
                self._persisted_checkpoint = None
            except OSError:
                pass
        self._refresh_recipe_state()
        self.set_robot_ready(self._robot_ready)


    def _update_run_availability(self) -> None:
        if self._running:
            self._btn_run.setEnabled(False)
            self._btn_run_selected.setEnabled(False)
            self._btn_preview.setEnabled(False)
            return
        dry_run_available = bool(self._dry_run and self._bridge.is_connected)
        available = bool(self._robot_ready or dry_run_available)
        self._btn_run.setEnabled(available)
        self._btn_run_selected.setEnabled(
            available and self._editor.selected_index() >= 0
        )
        self._btn_preview.setEnabled(bool(self._program.steps))


    def set_robot_ready(self, ready: bool) -> None:
        """Update motion/teach controls from the shared readiness policy."""
        self._robot_ready = bool(ready)
        self._btn_freedrive.setEnabled(self._robot_ready and not self._running)
        self._btn_teach.setEnabled(self._robot_ready and not self._running)
        self._btn_resume.setEnabled(
            bool(
                (self._running and self._paused and (self._robot_ready or self._dry_run))
                or (
                    not self._running
                    and self._persisted_checkpoint is not None
                    and self._robot_ready
                )
            )
        )
        self._update_run_availability()


    def handle_robot_unavailable(self, message: str) -> None:
        """Fail closed when a real execution loses its controller heartbeat."""
        self.set_robot_ready(False)
        self._set_freedrive_checked(False)
        if self._running and not self._dry_run:
            self._enter_recovery(str(message))


    def stop_for_software_stop(self) -> None:
        """Stop an active program and identify the global software-stop source."""
        self._stop(
            reason="Execução interrompida pela parada de software.",
            source="software_stop",
        )


    @staticmethod
    def _default_operator() -> str:
        return os.environ.get("USER", "").strip() or "não identificado"


    def _audit_common_fields(self) -> dict[str, Any]:
        run = self._audit_run or {}
        return {
            "operator": run.get("operator", "não identificado"),
            "program": run.get("program", self._program.name),
            "program_version": run.get("program_version", self._program.version),
            "program_hash": run.get("program_hash", ""),
            "mode": run.get("mode", "simulation" if self._dry_run else "real"),
            "start_index": run.get("start_index", 0),
        }


    def _begin_audit_run(self, program: Program, start_index: int) -> bool:
        """Create the run context and durably record it before motion starts."""
        try:
            program_data = program.to_dict()
            canonical = json.dumps(
                program_data,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._audit_log.last_error = f"programa não serializável: {exc}"
            return False
        operator = str(self._operator_provider() or "").strip() or "não identificado"
        started_at = utc_now_iso()
        run_id = new_run_id()
        self._audit_run = {
            "run_id": run_id,
            "operator": operator,
            "program": program.name,
            "program_version": program.version,
            "program_hash": hashlib.sha256(canonical).hexdigest(),
            "mode": "simulation" if self._dry_run else "real",
            "start_index": int(start_index),
            "started_at_utc": started_at,
            "started_monotonic": time.monotonic(),
            "write_failed": False,
        }
        self._audit_active_step = None
        self._audit_step_sequence = 0
        override_status = getattr(
            self._bridge, "dynamic_speed_override_status", {}
        )
        applied_override = float(
            override_status.get("applied", self._bridge.speed_override)
        )
        ok = self._append_audit(
            "run_start",
            run_id=run_id,
            **self._audit_common_fields(),
            started_at_utc=started_at,
            parameters={
                "speed_scale": float(self._speed_spinbox.value()),
                "global_speed_override": float(self._bridge.speed_override),
                "controller_speed_override_applied": applied_override,
                "dynamic_speed_override_supported": bool(
                    getattr(
                        self._bridge,
                        "dynamic_speed_override_supported",
                        False,
                    )
                ),
                "effective_speed_scale": (
                    float(self._speed_spinbox.value()) * float(self._bridge.speed_override)
                ),
                "step_by_step": bool(self._step_by_step),
                "record_joints": bool(self._chk_record.isChecked()),
                "estimated_steps": program.estimated_steps(),
                "estimated_commands": program.estimated_commands(),
            },
            result="started",
        )
        if not ok:
            self._audit_run = None
        return ok


    def _append_audit(self, event: str, **fields: Any) -> bool:
        ok = self._audit_log.append(event, **fields)
        if not ok and self._audit_run is not None:
            self._audit_run["write_failed"] = True
        return ok


    def _audit_step_details(self, op: tuple) -> tuple[str, str, dict[str, Any]]:
        tag = str(op[0]) if op else "unknown"
        if tag == OP_MOVE:
            raw_duration = float(op[2])
            program_duration = scale_duration(raw_duration, self._speed_scale)
            return tag, str(op[3]), {
                "pose": dict(op[1]),
                "duration_s": raw_duration,
                "scaled_duration_s": program_duration,
                "global_speed_override": float(self._bridge.speed_override),
                "effective_duration_s": self._bridge.effective_motion_duration(
                    program_duration
                ),
            }
        if tag == OP_GRIPPER:
            return tag, str(op[4]), {
                "position_mm": float(op[1]),
                "max_effort": float(op[2]),
                "timeout_s": float(op[3]),
            }
        if tag == OP_WAIT:
            return tag, str(op[2]), {"duration_s": float(op[1])}
        if tag == OP_WAIT_SM:
            return tag, str(op[3]), {"state": str(op[1]), "timeout_s": float(op[2])}
        if tag == OP_WAIT_SIG:
            return tag, str(op[5]), {
                "io_id": str(op[1]),
                "value": int(op[2]),
                "edge": str(op[3]),
                "timeout_s": float(op[4]),
            }
        if tag == OP_OUTPUT:
            return tag, str(op[3]), {"io_id": str(op[1]), "action": str(op[2])}
        return tag, "", {"opcode": list(op[1:])}


    def _audit_begin_step(self, op: tuple) -> bool:
        if self._audit_run is None:
            return False
        step_type, step_id, parameters = self._audit_step_details(op)
        self._audit_step_sequence += 1
        started_at = utc_now_iso()
        self._audit_active_step = {
            "step_sequence": self._audit_step_sequence,
            "step_index": self._vm_meta.get("pc_to_root_index", {}).get(self._vm_pc),
            "step_id": step_id,
            "step_type": step_type,
            "parameters": parameters,
            "started_at_utc": started_at,
            "started_monotonic": time.monotonic(),
        }
        fields = dict(self._audit_active_step)
        fields.pop("started_monotonic", None)
        return self._append_audit(
            "step_start",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            **fields,
            result="started",
        )


    def _audit_finish_step(self, result: str, message: str = "") -> bool:
        if self._audit_run is None or self._audit_active_step is None:
            return True
        active = self._audit_active_step
        self._audit_active_step = None
        finished_at = utc_now_iso()
        fields = {
            key: value for key, value in active.items() if key != "started_monotonic"
        }
        fields.update(
            finished_at_utc=finished_at,
            duration_ms=round(
                max(0.0, time.monotonic() - active["started_monotonic"]) * 1000.0,
                3,
            ),
            result=result,
            message=str(message),
        )
        return self._append_audit(
            "step_end",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            **fields,
        )


    def _audit_cancel(self, source: str, message: str) -> None:
        if self._audit_run is None:
            return
        active = self._audit_active_step or {}
        self._append_audit(
            "cancel_requested",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            step_sequence=active.get("step_sequence", ""),
            step_index=active.get("step_index", ""),
            step_id=active.get("step_id", ""),
            step_type=active.get("step_type", ""),
            parameters=active.get("parameters", {}),
            result="requested",
            message=message,
            cancel_source=source,
        )


    def _audit_finish_run(self, result: str, message: str) -> bool:
        if self._audit_run is None:
            return True
        run = self._audit_run
        ok = not bool(run.get("write_failed"))
        if result == "failure":
            active = self._audit_active_step or {}
            ok = self._append_audit(
                "failure",
                run_id=run["run_id"],
                **self._audit_common_fields(),
                step_sequence=active.get("step_sequence", ""),
                step_index=active.get("step_index", ""),
                step_id=active.get("step_id", ""),
                step_type=active.get("step_type", ""),
                parameters=active.get("parameters", {}),
                result="failure",
                message=message,
            ) and ok
        ok = self._audit_finish_step(result, message) and ok
        ok = self._append_audit(
            "run_end",
            run_id=run["run_id"],
            **self._audit_common_fields(),
            started_at_utc=run["started_at_utc"],
            finished_at_utc=utc_now_iso(),
            duration_ms=round(
                max(0.0, time.monotonic() - run["started_monotonic"]) * 1000.0,
                3,
            ),
            result=result,
            message=message,
        ) and ok
        self._audit_run = None
        self._audit_active_step = None
        return ok


    def _export_audit_csv(self) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suggested = self._audit_log.path.parent / f"execution_audit_{stamp}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "Exportar auditoria CSV", str(suggested), "CSV (*.csv)"
        )
        if not path:
            return
        try:
            count = self._audit_log.export_csv(path)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Erro ao exportar auditoria", str(exc))
            return
        self._lbl_status.setText(f"Auditoria exportada: {count} evento(s).")
        QMessageBox.information(
            self,
            "Auditoria exportada",
            f"{count} evento(s) salvos em:\n{path}",
        )


    def _on_dry_run_record_tick(self) -> None:
        """Generate dummy joint samples during dry-run so the plot works offline."""
        if not self._running or not self._dry_run or not self._joint_recording_active:
            return
        if self._step_waiting:
            return
        self._joint_history.append(self._make_dry_run_joint_sample())


    def _record_step_marker(self, step_id: str) -> None:
        """Store the start time of a program step for later plotting."""
        if not self._joint_recording_active:
            return
        t = time.monotonic() - self._joint_recording_start
        # Avoid duplicate markers at the exact same timestamp.
        if self._joint_history_markers and self._joint_history_markers[-1][0] == t:
            return
        self._joint_history_markers.append((t, step_id))


    def _make_dry_run_joint_sample(self) -> tuple[float, list[str], list[float]]:
        """Create a plausible synthetic joint sample for dry-run plotting."""
        now = time.monotonic()
        t = now - self._joint_recording_start
        names = [f"joint{i}" for i in range(1, 7)]
        # Use smooth sinusoidal trajectories so the plot looks like motion.
        # Higher frequencies and distinct offsets make each joint clearly
        # distinguishable and avoid flat-looking segments.
        positions = [
            0.55 * math.sin(1.5 * t + 0.0) + 0.10,
            0.65 * math.sin(1.8 * t + 0.7) - 0.15,
            0.50 * math.sin(2.1 * t + 1.4) + 0.25,
            0.75 * math.sin(1.6 * t + 2.1) - 0.10,
            0.40 * math.sin(2.4 * t + 2.8) + 0.20,
            0.85 * math.sin(1.9 * t + 3.5) - 0.25,
        ]
        return (t, names, positions)


    def _plot_joint_history(self) -> None:
        """Open a matplotlib window with joint angles over time."""
        if not self._joint_history:
            QMessageBox.information(self, "Sem dados", "Nenhuma amostra de juntas foi gravada.")
            return

        times = []
        series: dict[str, list[float]] = {}
        for t, names, positions in self._joint_history:
            times.append(t)
            for name, pos in zip(names, positions):
                if name == "gripper":
                    continue
                series.setdefault(name, []).append(float(pos))

        if not series:
            QMessageBox.information(self, "Sem dados", "Nenhuma junta do braço foi encontrada nas amostras.")
            return

        fig, ax = plt.subplots(figsize=(12, 6))
        for name, values in series.items():
            # Ensure all series have the same length as times (gripper may be absent in some samples).
            if len(values) == len(times):
                ax.plot(times, values, label=name, lw=1.5)

        # Vertical markers for step starts.
        if self._joint_history_markers:
            y_min, y_max = ax.get_ylim()
            y_range = y_max - y_min if y_max != y_min else 1.0
            for t, step_id in self._joint_history_markers:
                ax.axvline(t, color="gray", linestyle="--", linewidth=0.8, alpha=0.6)
                # Place label near the top, rotated for readability.
                ax.text(
                    t,
                    y_max - 0.02 * y_range,
                    step_id,
                    rotation=90,
                    va="top",
                    ha="right",
                    fontsize=7,
                    alpha=0.75,
                    color="#444444",
                )

        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("Posição (rad)")
        ax.set_title(f"Ângulos das juntas — {self._program.name}")
        ax.legend(loc="upper right")
        ax.grid(True)
        plt.tight_layout()
        plt.show()


    def _on_pose_result(self, tag: str, success: bool, message: str) -> None:
        if tag != self._vm_pending_tag:
            return  # stale / unrelated
        self._vm_pending_tag = None
        if not self._running:
            return
        if success:
            self._vm_advance()
        else:
            failure = f"Falha no passo {self._vm_pc + 1}: {message}"
            if self._dry_run:
                self._finish(ok=False, message=failure)
            else:
                self._enter_recovery(failure)


    def _on_action_result(self, name: str, success: bool, message: str) -> None:
        if not self._running or not self._vm_pending_gripper or name != "gripper_command":
            return
        self._vm_pending_gripper = False
        if self._wait_timer.isActive() and self._wait_kind == "gripper":
            self._wait_timer.stop()
        if success:
            self._vm_advance()
        else:
            failure = f"Falha no gripper no passo {self._vm_pc + 1}: {message}"
            if self._dry_run:
                self._finish(ok=False, message=failure)
            else:
                self._enter_recovery(failure)


    def _on_joint_state(self, msg) -> None:
        """Cache latest joint state; record if a program is running.

        In dry-run mode we use synthetic samples instead, because the real arm
        (or mock) does not move and would produce a flat line in the plot.
        """
        if self._dry_run or not self._joint_recording_active or not msg.name:
            return
        # Use monotonic time relative to the first recorded sample.
        now = time.monotonic()
        if not self._joint_history:
            self._joint_recording_start = now
        t = now - getattr(self, "_joint_recording_start", now)
        self._joint_history.append((t, list(msg.name), list(msg.position)))


    def _on_advance_clicked(self) -> None:
        if not self._running or not self._step_waiting:
            return
        self._step_waiting = False
        self._step_advance_pending = True
        self._btn_advance.setEnabled(False)
        self._vm_step()


    def _check_execution_preconditions(self) -> tuple[bool, str]:
        """Return (ok, message) verifying the arm is ready to run a program."""
        if self._dry_run:
            # Dry-run only needs the bridge for condition/signal evaluation.
            if not self._bridge.is_connected:
                return False, "Não conectado ao ROS 2."
            return True, ""
        return self._bridge.motion_readiness("move_to_pose")


    def load_program(self, program: Program, status: str = "Programa carregado.") -> None:
        if self._running:
            return
        self._program = program
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText(status)
        self.program_loaded.emit()
        self._set_modified(False)


