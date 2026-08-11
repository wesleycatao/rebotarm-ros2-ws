"""MotionTab preflight mixin — extracted from motion_tab.py."""

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


class MotionTabPreflightMixin:
    """Preflight concern methods for MotionTab."""

    def _open_preflight_dialog(self) -> None:
        """Open the preflight report dialog and run validation immediately."""
        if self._running:
            return
        # Commit any pending editor values so the dialog sees the latest program.
        self._sync_editors_to_program()
        dialog = PreflightDialog(
            self._bridge,
            get_program_cb=lambda: (self._program, not self._dry_run),
            parent=self,
        )
        dialog.revalidate()
        dialog.exec()


    def _preview_trajectory(self) -> None:
        """Compile and display the route without contacting the controller."""
        if self._running:
            return
        self._sync_editors_to_program()
        if not self._program.steps:
            QMessageBox.information(
                self, "Programa vazio", "Adicione etapas antes de gerar a prévia."
            )
            return
        dialog = TrajectoryPreviewDialog(
            self._program,
            speed_override=(
                self._bridge.speed_override * self._dynamic_override_ceiling()
            ),
            bridge=self._bridge,
            cell_scene_store=self._cell_scene_store,
            parent=self,
        )
        dialog.exec()


    def _preflight_gate(self, program: Program | None = None) -> bool:
        """Run preflight before execution; return whether the run may proceed.

        Real executions block on any error (including unreachable poses, with
        a bounded wait for the IK service). Dry-runs run the synchronous
        checks only and never block on reachability.
        """
        if self._preflight_in_progress:
            # A second click on Executar while the gate's nested event loop is
            # still running must not start a second probe/run.
            return False
        self._preflight_in_progress = True
        self._btn_run.setEnabled(False)
        self._btn_run_selected.setEnabled(False)
        try:
            return self._run_preflight_inner(not self._dry_run, program)
        finally:
            self._preflight_in_progress = False
            self._update_run_availability()


    def _run_preflight_inner(
        self,
        for_real_run: bool,
        program: Program | None = None,
    ) -> bool:
        try:
            report = run_preflight(
                program or self._program,
                self._bridge,
                for_real_run=for_real_run,
                with_reachability=for_real_run,
                reachability_timeout_ms=REACHABILITY_DEFAULT_TIMEOUT_MS,
                require_certified_envelope=for_real_run,
            )
        except Exception as exc:  # defensive: never let preflight crash a run
            QMessageBox.critical(
                self, "Validação indisponível",
                f"Não foi possível executar o preflight: {exc}",
            )
            return False
        self._last_preflight = report
        if not report.ok:
            self._show_preflight_block(report)
            return False
        if for_real_run:
            checked_program = program or self._program
            try:
                vision_spec = envelope_spec(checked_program)
            except ValueError as exc:
                QMessageBox.critical(
                    self,
                    "Execução bloqueada pelo envelope da visão",
                    f"{exc}\n\nNenhum movimento foi enviado.",
                )
                return False
            if (
                vision_spec is not None
                and checked_program.recipe_status == RECIPE_RELEASED
            ):
                metadata = checked_program.parameters.get(
                    VISION_ENVELOPE_ANALYSIS_KEY, {}
                )
                recorded = (
                    str(metadata.get("signature", ""))
                    if isinstance(metadata, dict)
                    else ""
                )
                if recorded != vision_spec.signature():
                    QMessageBox.critical(
                        self,
                        "Execução bloqueada pelo envelope da visão",
                        "A receita não possui uma certificação íntegra do "
                        "envelope atual. Crie uma nova revisão e libere-a "
                        "novamente.\n\nNenhum movimento foi enviado.",
                    )
                    return False
            scene_ok, _clearance, scene_reason = self._cell_scene_check(
                checked_program,
                require_release_match=True,
            )
            if not scene_ok:
                QMessageBox.critical(
                    self,
                    "Execução bloqueada pela cena da célula",
                    f"{scene_reason}\n\nNenhum movimento foi enviado.",
                )
                return False
            dynamics, _limiting_sample = self._run_envelope_dynamics(
                checked_program,
                speed_override=1.0,
            )
            if not dynamics.verified or dynamics.max_safe_override < 0.05 - 1e-9:
                self._show_dynamics_block(dynamics)
                return False
            self._runtime_dynamic_ceiling = min(
                self._dynamic_override_ceiling(),
                max(0.05, dynamics.max_safe_override),
            )
            self._refresh_effective_speed_label()
        if not report.warnings:
            return True
        # Warnings only: inform once and let the operator decide via the
        # confirm dialog that follows.
        QMessageBox.information(
            self, "Preflight OK com avisos",
            f"{report.summary()}\n\nA execução pode prosseguir; revise os avisos "
            "no botão \"Validar\" se necessário.",
        )
        return True


    def _cell_scene_check(
        self,
        program: Program,
        *,
        require_release_match: bool,
    ) -> tuple[bool, ClearanceReport, str]:
        """Validate the local TCP envelope and its MoveIt synchronization."""

        store = self._cell_scene_store
        if store is not None and store.load_error:
            reason = (
                "O arquivo persistente da cena da célula é inválido: "
                f"{store.load_error}. Corrija e salve a cena antes de movimentar."
            )
            report = ClearanceReport(verified=False, messages=(reason,))
            return False, report, reason
        if store is None or not store.scene.configured:
            report = ClearanceReport(
                verified=True,
                messages=("cena local sem volumes habilitados",),
            )
            return True, report, ""
        scene = store.scene
        if self._collision_planning_required and not store.is_applied:
            reason = (
                "A cena local foi criada ou alterada, mas ainda não foi confirmada "
                "pelo serviço /apply_planning_scene nesta sessão. Abra Robô 3D e "
                "célula e use “Aplicar no MoveIt”."
            )
            report = ClearanceReport(verified=False, messages=(reason,))
            return False, report, reason
        try:
            variants = envelope_variants(program)
        except (TypeError, ValueError) as exc:
            report = ClearanceReport(verified=False, messages=(str(exc),))
            return False, report, f"Envelope da visão inválido: {exc}"
        reports: list[tuple[str, ClearanceReport]] = []
        for sample_label, variant in variants:
            sample_report = analyze_program_tcp_clearance(variant, scene)
            if not sample_report.verified:
                return False, sample_report, (
                    f"{sample_label}: {sample_report.summary()}"
                )
            if not sample_report.safe:
                segment = (
                    f"trecho {sample_report.limiting_segment + 1}"
                    if sample_report.limiting_segment >= 0
                    else "trajetória"
                )
                return (
                    False,
                    sample_report,
                    f"{sample_label}: {sample_report.summary()} ({segment}). "
                    "Revise o envelope, os pontos ou a geometria da célula.",
                )
            reports.append((sample_label, sample_report))
        _sample_label, report = min(
            reports,
            key=lambda item: item[1].minimum_clearance_m,
        )
        if require_release_match and program.recipe_status == RECIPE_RELEASED:
            metadata = program.parameters.get("_cell_scene_analysis", {})
            recorded = (
                str(metadata.get("scene_signature", ""))
                if isinstance(metadata, dict)
                else ""
            )
            if recorded != scene.signature():
                return (
                    False,
                    report,
                    "A cena da célula mudou após a liberação desta receita. "
                    "Crie uma nova revisão e libere-a novamente contra a cena atual.",
                )
        return True, report, ""


    def _run_envelope_dynamics(
        self,
        program: Program,
        *,
        speed_override: float,
    ) -> tuple[DynamicsAnalysisReport, str]:
        """Certify joint demand independently at each envelope sample."""

        try:
            variants = envelope_variants(program)
        except (TypeError, ValueError) as exc:
            return (
                DynamicsAnalysisReport(messages=[f"envelope da visão inválido: {exc}"]),
                "",
            )
        reports: list[tuple[str, DynamicsAnalysisReport]] = []
        for sample_label, variant in variants:
            report = run_dynamic_analysis(
                variant,
                self._bridge,
                speed_override=speed_override,
            )
            if not report.verified:
                report.messages.insert(0, f"{sample_label}: análise não concluída")
                return report, sample_label
            reports.append((sample_label, report))
        limiting_label, limiting = min(
            reports,
            key=lambda item: item[1].max_safe_override,
        )
        if len(reports) > 1:
            limiting.messages.append(
                f"{len(reports)} amostras do envelope verificadas; "
                f"limitante: {limiting_label}"
            )
        return limiting, limiting_label


    def _show_dynamics_block(self, report: DynamicsAnalysisReport) -> None:
        details = "\n".join(
            f"• {metric.joint_name}: vel. {metric.velocity_utilization * 100:.1f}%, "
            f"acel. {metric.acceleration_utilization * 100:.1f}%, "
            f"margem {metric.margin_percent:+.1f}%"
            for metric in report.metrics
        )
        message = report.summary()
        if details:
            message += f"\n\n{details}"
        if report.verified and report.max_safe_override < 0.05:
            message += (
                "\n\nA trajetória exigiria velocidade abaixo do mínimo de 5%. "
                "Aumente as durações ou revise a geometria."
            )
        else:
            message += "\n\nNenhum movimento foi enviado."
        QMessageBox.critical(
            self,
            "Execução bloqueada pela análise dinâmica",
            message,
        )


    def _show_preflight_block(self, report: PreflightReport) -> None:
        errors = report.errors[:12]
        lines = "\n".join(f"• [{e.category}] {e.message}" for e in errors)
        more = "" if len(report.errors) <= len(errors) else f"\n… (+{len(report.errors) - len(errors)} erro(s))"
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Critical)
        box.setWindowTitle("Execução bloqueada pelo preflight")
        box.setText(
            f"{len(report.errors)} erro(s) impedem a execução:\n\n{lines}{more}"
        )
        open_btn = box.addButton("Abrir validação", QMessageBox.AcceptRole)
        box.addButton(QMessageBox.Cancel)
        box.setDefaultButton(open_btn)
        box.exec()
        if box.clickedButton() is open_btn:
            self._open_preflight_dialog()



    def _release_recipe(self) -> None:
        if self._running or self._production_mode:
            return
        if CAP_RELEASE_RECIPE not in self._bridge.authorization.capabilities():
            QMessageBox.warning(
                self,
                "Permissão necessária",
                "Autentique um Supervisor ou Engenharia para liberar a receita.",
            )
            return
        self._sync_editors_to_program()
        calibration_ok, calibration_reason, calibration_reference = (
            self._production_prerequisite_status()
        )
        if not calibration_ok:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                "A calibração da célula não está pronta:\n\n"
                f"{calibration_reason}",
            )
            return
        if calibration_reference:
            self._program.parameters["_calibration_profile"] = (
                calibration_reference
            )
            try:
                bind_calibration_error(
                    self._program,
                    calibration_reference,
                )
            except ValueError as exc:
                QMessageBox.warning(
                    self,
                    "Receita não liberada",
                    f"Não foi possível vincular a incerteza da calibração:\n\n{exc}",
                )
                return
        report = run_preflight(
            self._program,
            self._bridge,
            for_real_run=False,
            with_reachability=True,
            reachability_timeout_ms=10_000,
            require_certified_envelope=True,
        )
        if not report.ok:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                "Corrija os erros do preflight antes de liberar a receita.",
            )
            return
        scene_ok, clearance, scene_reason = self._cell_scene_check(
            self._program,
            require_release_match=False,
        )
        if not scene_ok:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                f"A cena da célula não está pronta:\n\n{scene_reason}",
            )
            return
        self._lbl_status.setText(
            "Analisando demanda de velocidade e aceleração das juntas…"
        )
        dynamics, limiting_envelope_sample = self._run_envelope_dynamics(
            self._program,
            speed_override=1.0,
        )
        if not dynamics.verified:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                "A análise dinâmica não pôde certificar a receita:\n\n"
                f"{dynamics.summary()}",
            )
            return
        ceiling = max(0.0, min(1.0, dynamics.max_safe_override))
        if ceiling < 0.05 - 1e-9:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                "A trajetória exigiria um override abaixo do mínimo operacional "
                f"de 5% ({ceiling * 100:.1f}%). Aumente as durações ou revise "
                "a geometria.",
            )
            return
        controller_limits = self._bridge.trajectory_limits()
        self._program.parameters["_dynamic_analysis"] = {
            "version": 1,
            "override_ceiling": round(ceiling, 9),
            "limiting_joint": dynamics.limiting_joint,
            "limiting_quantity": dynamics.limiting_quantity,
            "limits_signature": limits_signature(controller_limits),
        }
        try:
            vision_spec = envelope_spec(self._program)
        except ValueError as exc:
            QMessageBox.warning(
                self,
                "Receita não liberada",
                f"Envelope da visão inválido:\n\n{exc}",
            )
            return
        if vision_spec is not None:
            self._program.parameters[VISION_ENVELOPE_ANALYSIS_KEY] = {
                "version": 1,
                "signature": vision_spec.signature(),
                "sample_count": len(vision_spec.sample_positions()),
                "limiting_sample": limiting_envelope_sample,
                "override_ceiling": round(ceiling, 9),
            }
        else:
            self._program.parameters.pop(VISION_ENVELOPE_ANALYSIS_KEY, None)
        if self._cell_scene_store is not None and self._cell_scene_store.scene.configured:
            scene = self._cell_scene_store.scene
            self._program.parameters["_cell_scene_analysis"] = {
                "version": 2,
                "scene_signature": scene.signature(),
                "minimum_clearance_m": round(clearance.minimum_clearance_m, 9),
                "required_clearance_m": round(clearance.required_clearance_m, 9),
                "limiting_obstacle": clearance.limiting_obstacle,
                "analysis_scope": "tcp_envelope",
                "monitored_scene_signature": (
                    self._cell_scene_store.monitored_signature
                    if self._cell_scene_store.monitored_ready
                    else ""
                ),
                "monitored_external_geometry_count": len(
                    self._cell_scene_store.external_geometries
                ),
            }
        else:
            self._program.parameters.pop("_cell_scene_analysis", None)
        operator = str(self._operator_provider() or "").strip()
        try:
            digest = self._program.release(operator)
        except ValueError as exc:
            QMessageBox.warning(self, "Receita não liberada", str(exc))
            return
        self._set_modified(True)
        self._refresh_recipe_state()
        self._lbl_status.setText(
            f"Receita r{self._program.revision} liberada ({digest[:12]}…). "
            f"100% global = {ceiling * 100:.0f}% do limite físico."
        )
        self._refresh_effective_speed_label()


    def _start_new_revision(self) -> None:
        if self._running or self._production_mode:
            return
        if CAP_EDIT_RECIPE not in self._bridge.authorization.capabilities():
            QMessageBox.warning(
                self,
                "Permissão necessária",
                "Autentique um Supervisor ou Engenharia para criar uma revisão.",
            )
            return
        self._program.start_new_revision()
        self._invalidate_dynamic_analysis()
        self._set_modified(True)
        self._refresh_recipe_state()
        self._lbl_status.setText(
            f"Revisão r{self._program.revision} aberta para edição."
        )


    def _save_dialog(self) -> None:
        if self._running:
            return
        start_dir = self._last_dir()
        path, _ = QFileDialog.getSaveFileName(
            self, "Salvar programa de movimento", start_dir, "Programas JSON (*.json)"
        )
        if not path:
            return
        path = str(Path(path).with_suffix(".json"))
        try:
            self._save_to_file(path)
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao salvar", str(exc))


    def _load_dialog(self) -> None:
        if self._running:
            return
        if self._is_modified and not self._confirm("Há alterações não salvas. Deseja descartá-las?"):
            return
        start_dir = self._last_dir()
        path, _ = QFileDialog.getOpenFileName(
            self, "Carregar programa de movimento", start_dir, "Programas JSON (*.json)"
        )
        if not path:
            return
        try:
            self._load_from_file(path)
            self._remember_dir(path)
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao carregar", str(exc))


    def _save_preload(self) -> None:
        if self._running:
            return
        try:
            self._config_dir_ensure()
            self._save_to_file(str(_preload_path()))
            self._lbl_status.setText(f"Pré-carga definida: {_preload_path()}")
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao definir pré-carga", str(exc))


    def _config_dir_ensure(self) -> None:
        d = _config_dir()
        d.mkdir(parents=True, exist_ok=True)


    def _save_to_file(self, path: str) -> None:
        """Save the program atomically after structural validation.

        Structural errors block saving. Operational warnings allow saving as a
        draft but keep the recipe unreleasable. The previous file is only
        replaced after the new content is written successfully.
        """
        # make sure editors committed their latest values
        self._sync_editors_to_program()
        report = validate_program(self._program, bridge=self._bridge)
        if not report.can_save:
            self._show_validation_report(report, "Salvar programa")
            raise ValueError("validação estrutural falhou; o arquivo não foi alterado")
        target = Path(path)
        temp = target.with_suffix(target.suffix + ".tmp")
        try:
            temp.write_text(self._program.to_json(), encoding="utf-8")
            temp.replace(target)
        except Exception:
            if temp.exists():
                try:
                    temp.unlink()
                except OSError:
                    pass
            raise
        self._current_path = str(target.resolve())
        self._lbl_status.setText(f"Salvo em {path}")
        self._set_modified(False)
        self._add_recent(path)
        self.toast_requested.emit("Programa salvo", "success")
        if not report.can_release:
            self._show_validation_report(report, "Salvo como rascunho")


    def _show_validation_report(self, report: ProgramValidationResult, title: str) -> None:
        """Show a concise validation report and offer the preflight details."""
        lines = [report.summary(), ""]
        for issue in report.structural_errors[:8]:
            loc = f" (etapa {issue.step_index + 1})" if issue.step_index is not None else ""
            lines.append(f"• [{issue.severity.upper()}]{loc} {issue.message}")
        for issue in report.operational_warnings[:8]:
            loc = f" (etapa {issue.step_index + 1})" if issue.step_index is not None else ""
            lines.append(f"• [{issue.severity.upper()}]{loc} {issue.message}")
        more = len(report.structural_errors) + len(report.operational_warnings) - len(lines) + 2
        if more > 0:
            lines.append(f"\n… e mais {more} item(s).")
        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText("\n".join(lines))
        if report.structural_errors:
            box.setIcon(QMessageBox.Critical)
            box.addButton(QMessageBox.Ok)
        else:
            box.setIcon(QMessageBox.Warning)
            details = box.addButton("Abrir validação", QMessageBox.AcceptRole)
            box.addButton(QMessageBox.Ok)
            box.setDefaultButton(details)
        box.exec()
        if box.clickedButton() is details:
            self._open_preflight_dialog()


    def _sync_editors_to_program(self) -> None:
        # The StepListEditor mutates program.steps in place; speed_scale/name
        # are kept in sync via their change handlers. Nothing extra to do, but
        # this hook documents the intent and is a safe place to extend.
        self._program.name = self._name_edit.text().strip() or "programa"
        self._program.speed_scale = float(self._speed_spinbox.value())


    def _load_from_file(self, path) -> None:
        text = Path(path).read_text(encoding="utf-8")
        program = Program.from_json(text)
        self._program = program
        self._current_path = str(Path(path).resolve())
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText(f"Carregado: {path}")
        self.program_loaded.emit()
        self._set_modified(False)
        self._add_recent(str(path))


    def _confirm(self, text: str) -> bool:
        return QMessageBox.question(self, "Confirmar", text) == QMessageBox.Yes


    def _new_program(self) -> None:
        if self._running:
            return
        if self._program.steps and not self._confirm("Limpar o programa atual?"):
            return
        self._program = Program(name="programa")
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText("Novo programa.")
        self._set_modified(False)


    def _settings(self):
        from PySide6.QtCore import QSettings
        return QSettings("reBotArm", "rebotarm_gui")


    def _last_dir(self) -> str:
        return self._settings().value("motion/lastDir", str(Path.home()))


    def _remember_dir(self, path: str) -> None:
        self._settings().setValue("motion/lastDir", str(Path(path).parent))


    def _recent_paths(self) -> list[str]:
        raw = self._settings().value("motion/recent", []) or []
        if isinstance(raw, str):
            raw = [raw]
        paths = [str(p) for p in raw if str(p)]
        # Drop duplicates while preserving most-recent-first order.
        seen: set[str] = set()
        unique: list[str] = []
        for p in paths:
            if p not in seen:
                seen.add(p)
                unique.append(p)
        return unique[:RECENT_PROGRAMS_MAX]


    def _add_recent(self, path: str) -> None:
        path = str(path)
        if not path:
            return
        paths = [path] + [p for p in self._recent_paths() if p != path]
        self._settings().setValue("motion/recent", paths[:RECENT_PROGRAMS_MAX])
        self._refresh_recent_menu()


    def _refresh_recent_menu(self) -> None:
        self._recent_menu.clear()
        paths = self._recent_paths()
        if not paths:
            empty = self._recent_menu.addAction("(vazio)")
            empty.setEnabled(False)
            return
        for path in paths:
            label = Path(path).name or path
            action = self._recent_menu.addAction(label)
            action.setToolTip(path)
            action.setData(path)
            action.triggered.connect(lambda _checked=False, p=path: self._open_recent(p))


    def _open_recent(self, path: str) -> None:
        if self._running:
            return
        if not Path(path).is_file():
            self._lbl_status.setText(f"Arquivo não encontrado: {path}")
            QMessageBox.warning(self, "Abrir recente", f"Arquivo não encontrado:\n{path}")
            # Prune the stale entry so it stops lingering in the list.
            remaining = [p for p in self._recent_paths() if p != path]
            self._settings().setValue("motion/recent", remaining)
            self._refresh_recent_menu()
            return
        if self._is_modified and not self._confirm(
            "Há alterações não salvas. Deseja descartá-las?"
        ):
            return
        try:
            self._load_from_file(path)
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao carregar", str(exc))

