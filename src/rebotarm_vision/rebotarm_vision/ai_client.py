"""HTTP client for the AI vision calibration API."""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request
from typing import Any

from .vision_profile import VisionProfile, api_response_schema, apply_scene_workspace_polygon


DEFAULT_BASE_URL = "https://api.openai.com/v1"
DEFAULT_MODEL = "gpt-5.6-terra"


def encode_image_to_png_b64(image_bytes: bytes) -> str:
    """Encode an image buffer as a base64 PNG data URL."""
    b64 = base64.b64encode(image_bytes).decode("ascii")
    return f"data:image/png;base64,{b64}"


def request_vision_profile(
    image_bytes: bytes,
    prompt: str,
    *,
    api_key: str,
    base_url: str = DEFAULT_BASE_URL,
    model: str = DEFAULT_MODEL,
    timeout_s: int = 45,
    iteration: int = 0,
    previous_profile: VisionProfile | None = None,
    annotated_image_bytes: bytes | None = None,
    metrics: dict[str, Any] | None = None,
    notes: str = "",
) -> VisionProfile:
    """Call the AI Responses API and return a VisionProfile.

    On iteration 0 only the original image and prompt are sent. On later
    iterations the annotated image, previous profile and metrics are also
    included so the model can refine its proposal.
    """
    api_key = api_key.strip()
    if not api_key:
        raise RuntimeError("API key não configurada.")

    base_url = (base_url or DEFAULT_BASE_URL).rstrip("/")
    url = f"{base_url}/responses"

    text = _build_prompt(
        prompt=prompt,
        iteration=iteration,
        previous_profile=previous_profile,
        metrics=metrics,
        notes=notes,
    )
    content: list[dict[str, Any]] = [
        {"type": "input_text", "text": text},
        {
            "type": "input_image",
            "image_url": encode_image_to_png_b64(image_bytes),
            "detail": "high",
        },
    ]
    if annotated_image_bytes is not None:
        content.append(
            {
                "type": "input_image",
                "image_url": encode_image_to_png_b64(annotated_image_bytes),
                "detail": "high",
            }
        )

    payload = {
        "model": model.strip() or DEFAULT_MODEL,
        "input": [
            {
                "role": "user",
                "content": content,
            }
        ],
        "text": {
            "format": {
                "type": "json_schema",
                "name": "rebotarm_vision_autotune",
                "strict": True,
                "schema": api_response_schema(),
            }
        },
    }

    data = json.dumps(payload).encode("utf-8")
    http_request = urllib.request.Request(
        url,
        data=data,
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(http_request, timeout=max(5, int(timeout_s))) as response:
            response_body = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"API HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Falha de rede ao consultar API: {exc.reason}") from exc

    raw = json.loads(response_body)
    text_response = _extract_response_text(raw)
    parsed = json.loads(text_response)
    return _vision_profile_from_api_response(parsed)


def _build_prompt(
    prompt: str,
    iteration: int,
    previous_profile: VisionProfile | None,
    metrics: dict[str, Any] | None,
    notes: str,
) -> str:
    operator_prompt = (prompt or "objetos de interesse na mesa/esteira").strip()
    expected = _expected_count_from_prompt(prompt)
    expected_line = f"Contagem esperada: {expected}. " if expected else ""
    base_config = _calibration_config_template()
    schema = api_response_schema()
    parts = [
        "Você é um especialista em visão computacional industrial com OpenCV. "
        "A tarefa é calibrar parâmetros para segmentar objetos no frame sem redes neurais. "
        "Retorne somente o JSON final já sintonizado no schema solicitado, sem markdown, "
        "sem explicações e sem alternativas narrativas.\n",
        f"Entrada do operador:\n{operator_prompt}\n",
        expected_line,
        "O operador informa apenas quantidade esperada e tipo de objeto. "
        "Reconheça na imagem se as unidades estão em pé, deitadas, horizontais, verticais ou ambíguas. "
        "Sintetize os parâmetros OpenCV para cada perfil de orientação usando a imagem, "
        "a configuração base e o schema abaixo. Não trate a configuração base como preset recomendado; "
        "ela é somente o formato inicial e os defaults seguros do sistema.\n",
        "Retorne profiles.vertical e profiles.horizontal já calibrados. "
        "Identifique a área útil de trabalho na imagem e preencha scene.workspace_polygon com "
        "os vértices do polígono em pixels, na mesma escala da imagem. Esse polígono delimita "
        "a área de interesse e deve excluir trilhos, estrutura, chão, robô, grades e bordas fora da mesa/esteira. "
        "Use os mesmos vértices em workspace_limit_pixels dos perfis e ative use_workspace_mask=true. "
        "Se uma orientação não existir claramente na imagem, ainda retorne um perfil conservador e explique "
        "a incerteza apenas em iteration.notes. Inclua reference_objects somente quando houver alta confiança.\n",
        "Configuração base JSON:\n"
        + json.dumps(base_config, separators=(",", ":"), ensure_ascii=False),
        "Schema JSON dos parâmetros aceitos:\n"
        + json.dumps(schema, separators=(",", ":"), ensure_ascii=False),
    ]
    if iteration == 0:
        parts.append("Esta é a iteração inicial. Analise a imagem original e proponha perfis vertical/horizontal.")
    else:
        parts.append(f"Iteração {iteration}. Refine os perfis com base na imagem anotada e nas métricas.")
        if previous_profile is not None:
            # Only send fields the model can change. Persisted metadata, status
            # and duplicated metrics add tokens but provide no refinement signal.
            previous_candidate = {
                "scene": previous_profile.scene.to_dict(),
                "reference_objects": [obj.to_dict() for obj in previous_profile.reference_objects],
                "profiles": {
                    name: profile.to_dict()
                    for name, profile in previous_profile.profiles.items()
                },
                "merge": previous_profile.merge.to_dict(),
            }
            parts.append(
                "Perfil anterior:\n"
                + json.dumps(previous_candidate, separators=(",", ":"), ensure_ascii=False)
            )
        if metrics:
            parts.append(f"Métricas da validação local:\n{json.dumps(metrics, indent=2, ensure_ascii=False)}")
        if notes:
            parts.append(f"Observações: {notes}")
    return "\n".join(parts)


def _calibration_config_template() -> dict[str, Any]:
    """Return the mutable calibration JSON shape sent as model context."""
    profile = VisionProfile()
    return {
        "scene": profile.scene.to_dict(),
        "reference_objects": [obj.to_dict() for obj in profile.reference_objects],
        "profiles": {
            name: calibrated.to_dict()
            for name, calibrated in profile.profiles.items()
        },
        "merge": profile.merge.to_dict(),
        "iteration": profile.iteration.to_dict(),
    }


def _expected_count_from_prompt(prompt: str) -> int | None:
    import re
    matches = re.findall(
        r"\b(\d+)\s+(?:garrafas?|objetos?|itens?|peças?|pecas?)\b|\b(?:garrafas?|objetos?|itens?|peças?|pecas?)\s+(\d+)\b",
        prompt,
        re.IGNORECASE,
    )
    if not matches:
        return None
    values: list[int] = []
    for before_term, after_term in matches:
        count = before_term or after_term
        try:
            value = int(count)
        except Exception:
            continue
        if value > 0:
            values.append(value)
    if not values:
        return None
    return sum(values)


def _extract_response_text(response: dict[str, Any]) -> str:
    direct = response.get("output_text")
    if isinstance(direct, str) and direct.strip():
        return direct.strip()
    chunks: list[str] = []
    for item in response.get("output", []):
        for content in item.get("content", []):
            text = content.get("text")
            if isinstance(text, str):
                chunks.append(text)
    text = "\n".join(chunks).strip()
    if not text:
        raise RuntimeError("Resposta da API não trouxe texto JSON.")
    return text


def _vision_profile_from_api_response(data: dict[str, Any]) -> VisionProfile:
    """Convert the API response into a VisionProfile, normalising fields."""
    profile = VisionProfile.from_dict(data)
    apply_scene_workspace_polygon(profile)
    # The API response populates profiles and reference objects; iteration info
    # is also provided by the model. We keep the incoming structure but ensure
    # the schema version is current.
    profile.schema_version = "1.0.0"
    return profile
