"""ROS 2 node that orchestrates iterative AI-driven vision calibration."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import SegmentedObjectArray
from rebotarm_msgs.srv import CalibrateVision
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .ai_client import _expected_count_from_prompt, request_vision_profile
from .image_tools import image_msg_to_bgr
from .vision_profile import (
    ProfileMetrics,
    ProfileStatus,
    ReferenceObject,
    VisionProfile,
    apply_scene_workspace_polygon,
    image_hash,
    save_profile,
)


def profile_quality_key(profile: VisionProfile) -> tuple[float, ...]:
    """Return a stable multi-objective key used to retain the best candidate."""
    metrics = profile.metrics
    expected = max(
        1,
        int(profile.scene.expected_objects),
        len(profile.reference_objects),
    )
    has_references = any(
        ref.bbox_rotated.w > 0.0 and ref.bbox_rotated.h > 0.0
        for ref in profile.reference_objects
    )
    penalty = float(metrics.count_error) / float(expected)
    penalty += float(metrics.duplicate_count)
    penalty += float(metrics.grasp_zone_violation_count) / float(expected)
    if has_references:
        penalty += max(0.0, 0.95 - metrics.coverage) / 0.95
        penalty += max(0.0, 0.50 - metrics.mean_iou) / 0.50
        penalty += max(0.0, 0.90 - metrics.orientation_accuracy) / 0.90
    return (
        penalty,
        float(metrics.count_error),
        float(metrics.duplicate_count),
        float(metrics.grasp_zone_violation_count),
        -float(metrics.coverage),
        -float(metrics.mean_iou),
        -float(metrics.orientation_accuracy),
    )


class VisionCalibrationNode(Node):
    """Run the AI calibration loop and persist the resulting profile."""

    def __init__(self) -> None:
        super().__init__("rebotarm_vision_calibration")
        self.declare_parameter("api_key", "")
        self.declare_parameter("model", "gpt-5.6-terra")
        self.declare_parameter("base_url", "https://api.openai.com/v1")
        self.declare_parameter("timeout_s", 60)
        self.declare_parameter("max_iterations", 5)
        self.declare_parameter("max_time_s", 300.0)
        self.declare_parameter("cost_budget_usd", 0.0)
        self.declare_parameter("segmentation_set_parameters", "/rebotarm_segmentation/set_parameters")
        self.declare_parameter("segmentation_process_frame", "/rebotarm_segmentation/process_frame")
        self.declare_parameter("objects_2d_topic", "/rebotarm/vision/objects_2d")
        self.declare_parameter("debug_image_topic", "/rebotarm/vision/debug_image")
        self.declare_parameter("progress_topic", "~/progress")
        self.declare_parameter("persist_profiles", True)
        self.declare_parameter("profile_storage_dir", "")
        self.declare_parameter("parameter_settle_time_s", 0.0)
        self.declare_parameter("validation_timeout_s", 30.0)
        self.declare_parameter("set_parameters_timeout_s", 5.0)
        self.declare_parameter("process_frame_timeout_s", 5.0)
        self.declare_parameter("objects_timeout_s", 10.0)
        self.declare_parameter("max_reference_objects", 120)

        self._latest_objects: SegmentedObjectArray | None = None
        self._latest_debug: Image | None = None
        self._objects_event = threading.Event()
        self._debug_event = threading.Event()
        self._calibration_lock = threading.Lock()

        self.create_subscription(
            SegmentedObjectArray,
            str(self.get_parameter("objects_2d_topic").value),
            self._on_objects,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            self._on_debug,
            qos_profile_sensor_data,
        )
        self._progress_pub = self.create_publisher(
            String,
            str(self.get_parameter("progress_topic").value),
            10,
        )
        self._client_set_params = self.create_client(
            SetParameters,
            str(self.get_parameter("segmentation_set_parameters").value),
        )
        self._client_process_frame = self.create_client(
            Trigger,
            str(self.get_parameter("segmentation_process_frame").value),
        )
        self._calibration_cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(
            CalibrateVision,
            "~/calibrate",
            self._calibrate_callback,
            callback_group=self._calibration_cb_group,
        )
        self.get_logger().info("vision calibration node ready")

    def _on_objects(self, msg: SegmentedObjectArray) -> None:
        self._latest_objects = msg
        self._objects_event.set()

    def _on_debug(self, msg: Image) -> None:
        self._latest_debug = msg
        self._debug_event.set()

    def _publish_progress(self, text: str) -> None:
        self.get_logger().info(text)
        self._progress_pub.publish(String(data=text))

    def _wait_future(self, future, timeout: float) -> bool:
        """Wait for a ROS future to complete, spinning a local executor if needed."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if future.done():
                return True
            time.sleep(0.02)
        return future.done()

    def _calibrate_callback(
        self,
        request: CalibrateVision.Request,
        response: CalibrateVision.Response,
    ) -> CalibrateVision.Response:
        if not self._calibration_lock.acquire(blocking=False):
            response.success = False
            response.message = "Já existe uma calibração em andamento."
            response.profile_json = "{}"
            response.report = ""
            return response
        try:
            profile, report = self._run_calibration(request)
            response.success = True
            response.message = "Calibração concluída."
            response.profile_json = profile.to_json()
            response.report = report
        except Exception as exc:
            import traceback

            self.get_logger().error(f"calibração falhou: {exc}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = str(exc)
            response.profile_json = "{}"
            response.report = getattr(self, "_last_calibration_report", "")
        finally:
            self._calibration_lock.release()
        return response

    def _run_calibration(
        self,
        request: CalibrateVision.Request,
    ) -> tuple[VisionProfile, str]:
        # Prefer the transient key sent by the GUI for this request. It is not
        # logged, persisted, or copied into the generated profile.
        api_key = str(getattr(request, "api_key", "")).strip()
        if not api_key:
            api_key = str(self.get_parameter("api_key").value).strip()
        if not api_key:
            api_key = os.environ.get("OPENAI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("API key não configurada.")

        prompt = request.prompt.strip()
        if not prompt:
            raise RuntimeError("Prompt do cenário vazio.")

        image_bytes = self._image_msg_to_png(request.image)
        image_hash_value = image_hash(image_bytes)
        width = int(request.image.width)
        height = int(request.image.height)

        max_iterations = max(1, int(request.max_iterations or self.get_parameter("max_iterations").value))
        max_time_s = max(10.0, float(request.max_time_s or self.get_parameter("max_time_s").value))
        timeout_s = max(5, int(self.get_parameter("timeout_s").value))
        model = str(self.get_parameter("model").value)
        base_url = str(self.get_parameter("base_url").value)

        self._publish_progress("Iniciando calibração por IA...")
        start_time = time.monotonic()
        previous_profile: VisionProfile | None = None
        last_approved_profile: VisionProfile | None = None
        best_profile: VisionProfile | None = None
        best_quality: tuple[float, ...] | None = None
        report_lines: list[str] = []

        prompt_context = (
            f"Dimensões da imagem: {width}x{height} pixels. "
            "Use coordenadas de pixels nessa escala para scene.workspace_polygon, "
            "workspace_limit_pixels e reference_objects.\n"
        )
        prompt_with_context = prompt + "\n" + prompt_context

        for iteration in range(max_iterations):
            elapsed = time.monotonic() - start_time
            remaining_s = max_time_s - elapsed
            if remaining_s < 5.0:
                report_lines.append(f"Tempo limite atingido após {iteration} iterações.")
                break

            self._publish_progress(f"Iteração {iteration + 1}/{max_iterations}: consultando IA...")
            annotated_bytes: bytes | None = None
            if previous_profile is not None:
                annotated_bytes = self._latest_debug_image_png()

            metrics: dict[str, Any] = {}
            notes = ""
            if previous_profile is not None:
                metrics = previous_profile.metrics.to_dict()
                notes = previous_profile.iteration.notes

            try:
                candidate = request_vision_profile(
                    image_bytes=image_bytes,
                    prompt=prompt_with_context,
                    api_key=api_key,
                    base_url=base_url,
                    model=model,
                    timeout_s=min(timeout_s, max(5, int(remaining_s))),
                    iteration=iteration,
                    previous_profile=previous_profile,
                    annotated_image_bytes=annotated_bytes,
                    metrics=metrics,
                    notes=notes,
                )
            except Exception as exc:
                report_lines.append(f"Iteração {iteration + 1}: falha na API: {exc}")
                break

            candidate.image_hash = image_hash_value
            candidate.image_width = width
            candidate.image_height = height
            candidate.prompt = prompt
            candidate.model = model
            candidate.base_url = base_url
            apply_scene_workspace_polygon(candidate)

            expected_from_prompt = _expected_count_from_prompt(prompt)
            if expected_from_prompt is not None:
                candidate.scene.expected_objects = expected_from_prompt

            self._publish_progress(f"Iteração {iteration + 1}: validando localmente...")
            validated = self._validate_candidate(candidate, iteration + 1)
            self._publish_progress(
                f"Iteração {iteration + 1}: validação local concluída "
                f"({validated.status.reason})"
            )

            current_quality = profile_quality_key(validated)
            if best_quality is None or current_quality < best_quality:
                best_quality = current_quality
                best_profile = validated

            if validated.status.converged:
                validated.status.approved = True
                last_approved_profile = validated
                report_lines.append(
                    f"Iteração {iteration + 1}: convergiu. {validated.status.reason}"
                )
                break

            previous_profile = validated
            report_lines.append(
                f"Iteração {iteration + 1}: não convergiu ({validated.status.reason})."
            )
        else:
            report_lines.append(f"Número máximo de iterações ({max_iterations}) atingido.")

        final_profile = last_approved_profile or best_profile or previous_profile
        self._last_calibration_report = "\n".join(report_lines)
        if final_profile is None:
            raise RuntimeError("Nenhum perfil pôde ser gerado.")

        if not final_profile.status.converged:
            final_profile.status.reason = (
                "Perfil não convergiu dentro do limite de iterações/tempo. "
                f"Último motivo: {final_profile.status.reason}"
            )

        if bool(self.get_parameter("persist_profiles").value):
            try:
                storage_dir = str(self.get_parameter("profile_storage_dir").value).strip()
                import pathlib

                path = save_profile(
                    final_profile,
                    pathlib.Path(storage_dir) if storage_dir else None,
                )
                report_lines.append(f"Perfil salvo em: {path}")
            except Exception as exc:
                report_lines.append(f"Falha ao salvar perfil: {exc}")

        self._publish_progress(f"Calibração encerrada: {final_profile.status.reason}")
        return final_profile, "\n".join(report_lines)

    def _validate_candidate(
        self,
        profile: VisionProfile,
        iteration_number: int | None = None,
        *,
        allow_relaxation: bool = True,
    ) -> VisionProfile:
        """Apply the candidate profile locally and compute metrics."""
        validation_timeout = max(1.0, float(self.get_parameter("validation_timeout_s").value))
        deadline = time.monotonic() + validation_timeout

        def _remaining(default: float) -> float:
            return max(0.0, min(default, deadline - time.monotonic()))

        def _timed_out(stage: str) -> bool:
            if time.monotonic() < deadline:
                return False
            profile.status = ProfileStatus(
                converged=False,
                reason=f"timeout na validação local durante {stage}",
            )
            return True

        max_refs = max(0, int(self.get_parameter("max_reference_objects").value))
        if max_refs and len(profile.reference_objects) > max_refs:
            self._publish_progress(
                f"Validação local: limitando reference_objects de "
                f"{len(profile.reference_objects)} para {max_refs}."
            )
            profile.reference_objects = profile.reference_objects[:max_refs]

        # Apply profile to segmentation node.
        self._publish_progress("Validação local: aguardando serviço de parâmetros...")
        if not self._client_set_params.wait_for_service(timeout_sec=_remaining(2.0)):
            profile.status = ProfileStatus(
                converged=False,
                reason="serviço de parâmetros do segmentation node indisponível",
            )
            return profile
        if _timed_out("aguardar serviço de parâmetros"):
            return profile

        def _str_param(name: str, text: str) -> Parameter:
            p = Parameter()
            p.name = name
            v = ParameterValue()
            v.type = ParameterType.PARAMETER_STRING
            v.string_value = text
            p.value = v
            return p

        def _bool_param(name: str, flag: bool) -> Parameter:
            p = Parameter()
            p.name = name
            v = ParameterValue()
            v.type = ParameterType.PARAMETER_BOOL
            v.bool_value = bool(flag)
            p.value = v
            return p

        params = [
            _bool_param("use_orientation_profiles", True),
            _str_param("calibrated_profile_json", profile.to_json()),
            _str_param(
                "vertical_profile_json",
                json.dumps(profile.profiles["vertical"].to_dict()) if "vertical" in profile.profiles else "{}",
            ),
            _str_param(
                "horizontal_profile_json",
                json.dumps(profile.profiles["horizontal"].to_dict()) if "horizontal" in profile.profiles else "{}",
            ),
        ]
        self._publish_progress("Validação local: aplicando parâmetros no segmentation node...")
        future = self._client_set_params.call_async(SetParameters.Request(parameters=params))
        set_timeout = max(0.1, float(self.get_parameter("set_parameters_timeout_s").value))
        if not self._wait_future(future, _remaining(set_timeout)):
            profile.status = ProfileStatus(
                converged=False,
                reason="timeout ao aplicar parâmetros no segmentation node",
            )
            return profile
        if _timed_out("aplicar parâmetros"):
            return profile
        try:
            set_response = future.result()
            failures = [result.reason for result in set_response.results if not result.successful]
        except Exception as exc:
            failures = [str(exc)]
        if failures:
            profile.status = ProfileStatus(
                converged=False,
                reason="parâmetros rejeitados pelo segmentation node: " + "; ".join(failures),
            )
            return profile
        settle_time = max(0.0, float(self.get_parameter("parameter_settle_time_s").value))
        if settle_time:
            self._publish_progress(f"Validação local: aguardando estabilização ({settle_time:.2f}s)...")
            time.sleep(min(settle_time, _remaining(settle_time)))
        if _timed_out("estabilização dos parâmetros"):
            return profile

        # Trigger frame processing.
        self._publish_progress("Validação local: aguardando serviço process_frame...")
        if not self._client_process_frame.wait_for_service(timeout_sec=_remaining(2.0)):
            profile.status = ProfileStatus(
                converged=False,
                reason="serviço process_frame indisponível",
            )
            return profile
        if _timed_out("aguardar process_frame"):
            return profile
        # Clear immediately before triggering. Clearing before SetParameters can
        # incorrectly accept a publication produced while parameters are applied.
        self._objects_event.clear()
        self._debug_event.clear()
        self._latest_objects = None
        self._latest_debug = None
        self._publish_progress("Validação local: chamando process_frame...")
        future_process = self._client_process_frame.call_async(Trigger.Request())
        process_timeout = max(0.1, float(self.get_parameter("process_frame_timeout_s").value))
        if not self._wait_future(future_process, _remaining(process_timeout)):
            profile.status = ProfileStatus(
                converged=False,
                reason="timeout ao chamar process_frame",
            )
            return profile
        if _timed_out("chamar process_frame"):
            return profile
        try:
            process_response = future_process.result()
        except Exception as exc:
            profile.status = ProfileStatus(
                converged=False,
                reason=f"falha ao chamar process_frame: {exc}",
            )
            return profile
        if not process_response.success:
            profile.status = ProfileStatus(
                converged=False,
                reason=f"process_frame rejeitado: {process_response.message}",
            )
            return profile

        # Wait for results.
        self._publish_progress("Validação local: aguardando objetos segmentados...")
        objects_timeout = max(0.1, float(self.get_parameter("objects_timeout_s").value))
        if not self._objects_event.wait(timeout=_remaining(objects_timeout)):
            profile.status = ProfileStatus(
                converged=False,
                reason="timeout aguardando resultado da segmentação",
            )
            return profile
        if _timed_out("aguardar objetos segmentados"):
            return profile
        objects_msg = self._latest_objects

        detected_count = len(objects_msg.objects) if objects_msg is not None else 0
        expected_count = max(0, int(profile.scene.expected_objects))
        self._publish_progress(
            f"Validação local: calculando métricas ({detected_count} detectados"
            + (f", {expected_count} esperados" if expected_count else "")
            + ")..."
        )
        metrics = self._compute_metrics(profile, objects_msg)
        profile.metrics = metrics
        if _timed_out("calcular métricas"):
            return profile

        converged, reason = self._check_convergence(profile, objects_msg, metrics)
        sparse_threshold = max(1, int(round(expected_count * 0.25))) if expected_count else 0
        if (
            allow_relaxation
            and expected_count > 0
            and detected_count < sparse_threshold
        ):
            self._publish_progress(
                "Validação local: contagem muito baixa; relaxando filtros e revalidando..."
            )
            self._relax_sparse_profile(profile)
            return self._validate_candidate(
                profile,
                iteration_number,
                allow_relaxation=False,
            )
        profile.iteration.number = (
            int(iteration_number)
            if iteration_number is not None
            else max(1, int(profile.iteration.number) + 1)
        )
        profile.iteration.converged = converged
        profile.iteration.notes = reason
        profile.status = ProfileStatus(converged=converged, reason=reason)
        self._publish_progress(f"Validação local: métricas prontas ({reason})")
        return profile

    def _relax_sparse_profile(self, profile: VisionProfile) -> None:
        """Broaden obviously over-restrictive detector profiles in-place.

        The AI still proposes the profile, but transparent objects can easily
        produce contours with low solidity/extent. If local validation finds
        almost nothing, these guardrails keep the next validation from being
        dominated by filters that are too strict for the actual frame.
        """
        image_area = max(1, int(profile.image_width) * int(profile.image_height))
        max_area = max(25000.0, image_area * 0.025)
        for label, calibrated in profile.profiles.items():
            calibrated.opencv_mode = "bright_or_edge"
            calibrated.active_filters = [
                "detect_bright",
                "detect_edge",
                "min_area",
                "max_area",
                "min_aspect",
                "max_aspect",
            ]
            calibrated.shape_filter = "any"
            calibrated.min_area_px = min(float(calibrated.min_area_px), 1200.0)
            calibrated.max_area_px = max(float(calibrated.max_area_px), max_area)
            calibrated.min_solidity = min(float(calibrated.min_solidity), 0.15)
            calibrated.min_extent = min(float(calibrated.min_extent), 0.08)
            calibrated.max_extent = max(float(calibrated.max_extent), 0.98)
            calibrated.brightness_threshold = min(
                95,
                max(70, int(calibrated.brightness_threshold)),
            )
            calibrated.brightness_invert = False
            calibrated.morph_kernel = min(max(3, int(calibrated.morph_kernel)), 7)
            calibrated.morph_operation = max(1, int(calibrated.morph_operation))
            calibrated.edge_operation = max(1, int(calibrated.edge_operation))
            calibrated.canny_low = min(45, max(15, int(calibrated.canny_low)))
            calibrated.canny_high = min(130, max(70, int(calibrated.canny_high)))
            if label == "horizontal":
                calibrated.min_aspect_ratio = min(float(calibrated.min_aspect_ratio), 1.2)
                calibrated.max_aspect_ratio = max(float(calibrated.max_aspect_ratio), 8.0)
            elif label == "vertical":
                calibrated.min_aspect_ratio = min(float(calibrated.min_aspect_ratio), 0.8)
                calibrated.max_aspect_ratio = max(float(calibrated.max_aspect_ratio), 2.0)
            else:
                calibrated.min_aspect_ratio = min(float(calibrated.min_aspect_ratio), 0.8)
                calibrated.max_aspect_ratio = max(float(calibrated.max_aspect_ratio), 8.0)

    def _compute_metrics(
        self,
        profile: VisionProfile,
        objects_msg: SegmentedObjectArray | None,
    ) -> ProfileMetrics:
        expected = max(0, int(profile.scene.expected_objects))
        if expected == 0:
            expected = len(profile.reference_objects)
        if objects_msg is None:
            return ProfileMetrics(count_error=expected)

        detected = list(objects_msg.objects)
        count_error = abs(len(detected) - expected)

        # Reference objects with a non-degenerate bbox are used for IoU/orientation.
        valid_refs = [
            ref
            for ref in profile.reference_objects
            if ref.bbox_rotated.w > 0.0 and ref.bbox_rotated.h > 0.0
        ]

        # Match detections and references one-to-one by descending rotated IoU.
        # The old per-detection nearest-centre match could reuse one reference,
        # inflate coverage above 100%, and reject valid partial reference sets.
        matches: list[tuple[float, int, int]] = []
        for detected_index, obj in enumerate(detected):
            for reference_index, ref in enumerate(valid_refs):
                iou = self._approx_rotated_iou(
                    ref, obj, profile.image_width, profile.image_height
                )
                if iou > 0.0:
                    matches.append((iou, detected_index, reference_index))
        matches.sort(reverse=True)
        matched_detections: set[int] = set()
        matched_references: set[int] = set()
        accepted_matches: list[tuple[float, int, int]] = []
        for match in matches:
            _, detected_index, reference_index = match
            if detected_index in matched_detections or reference_index in matched_references:
                continue
            matched_detections.add(detected_index)
            matched_references.add(reference_index)
            accepted_matches.append(match)

        ious = [match[0] for match in accepted_matches]
        orientation_matches = 0
        orientation_evaluated = 0
        for _, detected_index, reference_index in accepted_matches:
            ref = valid_refs[reference_index]
            if ref.orientation == "ambiguous":
                continue
            orientation_evaluated += 1
            if detected[detected_index].orientation == ref.orientation:
                orientation_matches += 1

        duplicates = 0
        width = max(1, int(profile.image_width))
        height = max(1, int(profile.image_height))
        center_threshold = max(0.0, float(profile.merge.center_distance_threshold))
        iou_threshold = max(0.0, float(profile.merge.rotated_iou_threshold))
        for index, obj in enumerate(detected):
            center = (float(obj.center_pixel.x) / width, float(obj.center_pixel.y) / height)
            for other in detected[:index]:
                other_center = (
                    float(other.center_pixel.x) / width,
                    float(other.center_pixel.y) / height,
                )
                if math.hypot(center[0] - other_center[0], center[1] - other_center[1]) > center_threshold:
                    continue
                duplicate_iou = self._object_iou(obj, other)
                if duplicate_iou is None or duplicate_iou >= iou_threshold:
                    duplicates += 1
                    break

        mean_iou = sum(ious) / len(ious) if ious else 0.0
        orientation_accuracy = (
            orientation_matches / orientation_evaluated if orientation_evaluated else 1.0
        )
        if valid_refs:
            coverage = len(matched_references) / len(valid_refs)
        else:
            # Without valid reference bboxes we can only judge by count.
            coverage = 1.0 if count_error == 0 else 0.0
            orientation_accuracy = 1.0 if detected else 0.0
            mean_iou = 1.0 if count_error == 0 else 0.0

        # Pickability is the actual grasp contract. An ambiguous orientation
        # must not hide an object that the pipeline explicitly marked unsafe.
        grasp_violations = sum(1 for obj in detected if not obj.pickable)

        return ProfileMetrics(
            count_error=count_error,
            mean_iou=mean_iou,
            duplicate_count=duplicates,
            orientation_accuracy=orientation_accuracy,
            coverage=coverage,
            grasp_zone_violation_count=grasp_violations,
        )

    @staticmethod
    def _object_iou(first: Any, second: Any) -> float | None:
        """Return polygon IoU, or None when either detection has no valid box."""
        if len(first.bounding_box_pixels) != 4 or len(second.bounding_box_pixels) != 4:
            return None
        points_a = np.asarray(
            [[point.x, point.y] for point in first.bounding_box_pixels], dtype=np.float32
        )
        points_b = np.asarray(
            [[point.x, point.y] for point in second.bounding_box_pixels], dtype=np.float32
        )
        area_a = abs(float(cv2.contourArea(points_a)))
        area_b = abs(float(cv2.contourArea(points_b)))
        if area_a <= 0.0 or area_b <= 0.0:
            return None
        intersection, _ = cv2.intersectConvexConvex(points_a, points_b)
        union = area_a + area_b - float(intersection)
        return float(intersection) / union if union > 0.0 else 0.0

    def _approx_rotated_iou(
        self,
        ref: ReferenceObject,
        obj: Any,
        image_width: int,
        image_height: int,
    ) -> float:
        """Approximate IoU between a reference bbox and a detected bbox."""
        try:
            import cv2
        except Exception:
            return 0.0

        def box_points(cx, cy, w, h, angle_deg):
            rect = ((cx * image_width, cy * image_height), (w * image_width, h * image_height), angle_deg)
            return cv2.boxPoints(rect).astype(np.int32)

        pts_a = box_points(
            ref.bbox_rotated.cx,
            ref.bbox_rotated.cy,
            ref.bbox_rotated.w,
            ref.bbox_rotated.h,
            ref.bbox_rotated.angle_deg,
        )
        if len(obj.bounding_box_pixels) != 4:
            return 0.0
        pts_b = np.array([[p.x, p.y] for p in obj.bounding_box_pixels], dtype=np.int32)

        pts_a_float = pts_a.astype(np.float32)
        pts_b_float = pts_b.astype(np.float32)
        area_a = abs(float(cv2.contourArea(pts_a_float)))
        area_b = abs(float(cv2.contourArea(pts_b_float)))
        if area_a <= 0.0 or area_b <= 0.0:
            return 0.0
        intersection, _ = cv2.intersectConvexConvex(pts_a_float, pts_b_float)
        union = area_a + area_b - float(intersection)
        return float(intersection) / union if union > 0.0 else 0.0

    def _check_convergence(
        self,
        profile: VisionProfile,
        objects_msg: SegmentedObjectArray | None,
        metrics: ProfileMetrics,
    ) -> tuple[bool, str]:
        expected = max(0, int(profile.scene.expected_objects))
        if expected == 0:
            expected = len(profile.reference_objects)
        has_valid_refs = any(
            ref.bbox_rotated.w > 0.0 and ref.bbox_rotated.h > 0.0
            for ref in profile.reference_objects
        )
        if objects_msg is None:
            return False, "nenhuma mensagem de objetos recebida"
        detected = list(objects_msg.objects)
        count_error = abs(len(detected) - expected)
        count_tolerance = max(1, int(round(expected * 0.05)))
        if count_error > count_tolerance:
            return (
                False,
                f"contagem diferente: {len(detected)} detectados, {expected} esperados "
                f"(tolerância {count_tolerance})",
            )
        if metrics.duplicate_count > 0:
            return False, f"{metrics.duplicate_count} objetos duplicados entre perfis"
        if has_valid_refs:
            if metrics.coverage < 0.95:
                return False, f"cobertura baixa ({metrics.coverage:.2f})"
            if metrics.mean_iou < 0.5:
                return False, f"IoU médio baixo ({metrics.mean_iou:.2f})"
            if metrics.orientation_accuracy < 0.9:
                return False, f"acurácia de orientação baixa ({metrics.orientation_accuracy:.2f})"
        suffix = ""
        if metrics.grasp_zone_violation_count > 0:
            suffix = f"; {metrics.grasp_zone_violation_count} não pickable"
        return True, f"{len(detected)} objetos alinhados com referência{suffix}"

    @staticmethod
    def _image_msg_to_png(msg: Image) -> bytes:
        frame = image_msg_to_bgr(msg)
        ok, encoded = cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("falha ao codificar imagem para PNG")
        return encoded.tobytes()

    def _latest_debug_image_png(self) -> bytes | None:
        if self._latest_debug is None:
            return None
        try:
            return self._image_msg_to_png(self._latest_debug)
        except Exception:
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionCalibrationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
