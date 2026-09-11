"""Spec-driven OpenAI concept-image generation for H2AIR run folders.

Builds dimension-aware prompts from the closed design (geometry, packaging,
cooling) and optionally calls the OpenAI Images API. Sizing stays offline unless
the caller invokes ``generate_concept_images``.
"""
from __future__ import annotations

import base64
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

VIEW_STEMS = (
    "isometric_clean",
    "top_concept",
    "side_concept",
    "perspective_concept",
    "cutaway_internal",
    "cooling_airflow",
)

_STYLE = (
    "Clean industrial product-design concept render of a hydrogen-hybrid VTOL "
    "fixed-wing UAV. Neutral studio lighting, white or light-grey seamless backdrop, "
    "matte composite airframe with subtle panel lines, no logos, no watermarks, "
    "no readable text, no people. Proportions should follow the numerical brief; "
    "this is concept art, not a certified CAD drawing."
)


def _f(v: Any, n: int = 3) -> str:
    try:
        return f"{float(v):.{n}f}"
    except Exception:
        return str(v)


def _get(m: Mapping[str, Any] | None, *keys: str, default: Any = None) -> Any:
    cur: Any = m
    for k in keys:
        if not isinstance(cur, Mapping) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _tip(root: float, taper: float) -> float:
    return float(root) * float(taper)


def build_aircraft_brief(cfg: Mapping[str, Any], result: Mapping[str, Any]) -> str:
    """Compact dimensional brief mirroring the SUMMARY geometry language."""
    d = cfg.get("design") or {}
    s = result.get("summary") or {}
    geom = _get(cfg, "geometry_mass", "geometry_model", "geometry", default={}) or {}
    lay_cfg = _get(cfg, "aero_propulsion", "lift_layout", default={}) or {}
    layout = _get(result, "aero_propulsion", "layout", default={}) or {}
    cooling = result.get("fuel_cell_cooling") or _get(cfg, "_runtime", "fuel_cell_cooling", default={}) or {}
    mm = cfg.get("manufacturing_mass_model") or {}
    wing_z = float(_get(mm, "main_wing", "z_m", default=0.10) or 0.10)
    ht_z = float(_get(mm, "horizontal_tail", "z_m", default=0.03) or 0.03)
    pusher_xyz = _get(cfg, "sixdof_control", "aircraft", "pusher_station_m", default=[None, None, None]) or [None, None, None]

    b = float(d.get("wing_span_m", 0.0) or 0.0)
    S = float(d.get("wing_area_m2", 0.0) or 0.0)
    mac = float(d.get("wing_mac_m", 0.0) or 0.0)
    cr = float(d.get("main_wing_root_chord_m", 0.0) or 0.0)
    lam = float(d.get("main_wing_taper_ratio", 0.0) or 0.0)
    ar = (b * b / S) if S > 1e-12 else 0.0
    Lf = float(d.get("fuselage_length_m", 0.0) or 0.0)
    Wf = float(d.get("fuselage_width_m", d.get("fuselage_max_diameter_m", 0.0)) or 0.0)
    Hf = float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", 0.0)) or 0.0)
    Df = float(d.get("fuselage_max_diameter_m", 0.0) or 0.0)
    xs = str(d.get("fuselage_cross_section", "rectangular_filleted"))
    fillet_mm = 1000.0 * float(d.get("fuselage_fillet_radius_m") or 0.0)

    boom_y = geom.get("boom_center_y_m") or [-0.35, 0.35]
    boom_span = (
        abs(float(boom_y[1]) - float(boom_y[0]))
        if isinstance(boom_y, (list, tuple)) and len(boom_y) >= 2
        else None
    )
    boom_x0 = geom.get("boom_x_start_m")
    boom_x1 = geom.get("boom_x_end_m")
    y_lift = float(lay_cfg.get("y_abs_m", 0.48) or 0.48)
    prop_in = layout.get("prop_diameter_in", d.get("lift_propeller_diameter_in"))

    inlet = str(cooling.get("inlet_side", "left")).lower()
    exhaust = str(cooling.get("exhaust_side", "right")).lower()
    arch = str(cooling.get("architecture", "cross_fuselage"))
    fan_dir = str(cooling.get("fan_flow_direction", "suction_through_stack_to_fan_side"))

    bh = float(d.get("horizontal_tail_span_m", 0.0) or 0.0)
    hv = float(d.get("vertical_tail_height_m", 0.0) or 0.0)
    mtow = s.get("post_structure_mass_kg", d.get("mtow_ceiling_kg"))

    lines = [
        "H2AIR aircraft specification brief (use for proportions):",
        (
            f"Hydrogen-hybrid VTOL / fixed-wing UAV, closed MTOW about {_f(mtow, 2)} kg, "
            f"cruise {_f(d.get('cruise_speed_mps', 0), 1)} m/s, "
            f"payload {_f(d.get('payload_mass_kg', 0), 1)} kg."
        ),
        (
            f"Central fuselage pod: length {_f(Lf, 3)} m, outer width {_f(Wf, 3)} m, "
            f"height {_f(Hf, 3)} m (max outer {_f(Df * 1000, 0)} mm), cross-section {xs}"
            + (f" with {_f(fillet_mm, 0)} mm fillets" if xs != "circular" and fillet_mm > 0 else "")
            + "."
        ),
        (
            f"Main wing: span {_f(b, 3)} m, area {_f(S, 3)} m², AR {_f(ar, 2)}, "
            f"root/tip/MAC {_f(cr, 3)}/{_f(_tip(cr, lam), 3)}/{_f(mac, 3)} m, "
            f"airfoil {d.get('main_wing_naca_code', '—')}, "
            f"sweep {_f(d.get('main_wing_sweep_deg', 0), 1)} deg, "
            f"dihedral {_f(d.get('main_wing_dihedral_deg', 0), 1)} deg, "
            f"structure waterline z ≈ {_f(wing_z, 3)} m."
        ),
        (
            f"Twin-boom empennage: boom spacing centre-to-centre {_f(boom_span, 3)} m"
            if boom_span is not None
            else "Twin-boom empennage"
        )
        + (
            f", boom extent x={_f(boom_x0, 3)} to {_f(boom_x1, 3)} m"
            if boom_x0 is not None and boom_x1 is not None
            else ""
        )
        + (
            f"; horizontal tail span {_f(bh, 3)} m at z≈{_f(ht_z, 3)} m; "
            f"twin vertical fins height {_f(hv, 3)} m each."
        ),
        (
            f"Lift layout: four boom-mounted lift propellers, diameter {_f(prop_in, 1)} in, "
            f"lateral station |y|={_f(y_lift, 3)} m, "
            f"front/rear rows at x≈{_f(layout.get('proposed_front_x_m'), 3)} / "
            f"{_f(layout.get('proposed_rear_x_m'), 3)} m; "
            f"cruise lift-prop state {d.get('cruise_lift_propeller_state', 'folded')}; "
            f"rear fuselage pusher at x≈{_f(pusher_xyz[0], 3)} m."
        ),
        (
            f"Internal packaging (nose→aft stations): energy bay order "
            f"{d.get('energy_bay_order', 'fuel_cell_forward')}; "
            f"fuel cell x={_f(d.get('fuel_cell_x_m'), 3)} m, z={_f(d.get('fuel_cell_z_m'), 3)} m; "
            f"buffer battery x={_f(d.get('battery_x_m'), 3)} m, z={_f(d.get('battery_z_m'), 3)} m; "
            f"H2 cylinder x={_f(d.get('hydrogen_cylinder_x_m'), 3)} m, "
            f"z={_f(d.get('hydrogen_cylinder_z_m'), 3)} m; "
            f"payload x={_f(d.get('payload_x_m'), 3)} m."
        ),
        (
            f"Fuel-cell cooling: {arch} architecture, fan direction {fan_dir}; "
            f"{inlet}-side NACA-style flush inlet → stack → built-in fans → "
            f"{exhaust}-side exhaust grille; "
            f"design waste heat {_f(cooling.get('waste_heat_design_W'), 0)} W, "
            f"required airflow {_f(cooling.get('required_airflow_cfm'), 1)} CFM."
        ),
    ]
    return "\n".join(lines)


def build_view_prompts(brief: str) -> Dict[str, str]:
    """One OpenAI prompt per fixed concept view."""
    views = {
        "isometric_clean": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: clean isometric exterior render, three-quarter high angle, "
            "complete airframe visible, lift props folded or stowed for cruise if specified, "
            "sharp silhouette, product-catalog quality."
        ),
        "top_concept": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: true top orthographic concept (plan view), camera directly above, "
            "symmetric twin-boom layout clear, wing and horizontal tail planforms accurate "
            "to the span and taper brief, no perspective foreshortening."
        ),
        "side_concept": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: true side orthographic concept (profile), camera on the left side, "
            "fuselage length vs wing/boom/tail stations readable, pusher at aft fuselage, "
            "no perspective foreshortening."
        ),
        "perspective_concept": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: cinematic 3/4 front-left perspective exterior concept, "
            "slightly low camera, show wing span vs short central pod contrast, "
            "twin booms and empennage readable."
        ),
        "cutaway_internal": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: longitudinal cutaway of the central fuselage pod (side section), "
            "outer skin ghosted or removed on the near side; show ordered internal architecture: "
            "nose payload/avionics, fuel-cell stack, buffer battery, hydrogen cylinder, "
            "power distribution hardware along the pod axis per the station brief; "
            "wings/booms may be cropped; technical concept illustration style."
        ),
        "cooling_airflow": (
            f"{_STYLE}\n\n{brief}\n\n"
            "VIEW: cooling airflow concept focused on the mid-fuselage fuel-cell bay; "
            "cross-fuselage suction path clearly diagrammed with translucent blue airflow arrows: "
            "NACA-style flush inlet on the inlet side, through the fuel-cell stack, "
            "to built-in fans and the opposite-side exhaust grille; "
            "semi-transparent fuselage skin so the air path is obvious; "
            "engineering concept visualization, not a CFD plot."
        ),
    }
    return views


def _require_openai():
    try:
        from openai import OpenAI  # type: ignore
    except ImportError as exc:  # pragma: no cover - exercised when dep missing
        raise ImportError(
            "The openai package is required for --concept-images. "
            "Install with: pip install -r requirements-concept-images.txt"
        ) from exc
    return OpenAI


def _resolve_api_key(api_key: Optional[str]) -> str:
    key = (api_key or os.environ.get("OPENAI_API_KEY") or "").strip()
    if not key:
        raise RuntimeError(
            "OpenAI API key missing. Set OPENAI_API_KEY or pass --openai-api-key."
        )
    return key


def _save_b64_png(path: Path, b64_json: str) -> None:
    path.write_bytes(base64.b64decode(b64_json))


def generate_concept_images(
    cfg: Mapping[str, Any],
    result: Mapping[str, Any],
    out_dir: str | Path,
    *,
    api_key: Optional[str] = None,
    model: str = "gpt-image-1",
    views: Optional[Sequence[str]] = None,
    size: str = "1024x1024",
    quality: str = "high",
) -> Dict[str, Any]:
    """Generate concept PNGs under ``out_dir/details/concept_images``.

    Continues on per-view API failures and records them in ``manifest.json``.
    """
    out = Path(out_dir)
    img_dir = out / "details" / "concept_images"
    img_dir.mkdir(parents=True, exist_ok=True)

    brief = build_aircraft_brief(cfg, result)
    all_prompts = build_view_prompts(brief)
    selected = list(views) if views is not None else list(VIEW_STEMS)
    unknown = [v for v in selected if v not in all_prompts]
    if unknown:
        raise ValueError(f"Unknown concept view(s): {unknown}; choose from {list(VIEW_STEMS)}")

    prompts = {stem: all_prompts[stem] for stem in selected}
    (img_dir / "prompts.json").write_text(
        json.dumps({"brief": brief, "prompts": prompts}, indent=2) + "\n",
        encoding="utf-8",
    )

    OpenAI = _require_openai()
    key = _resolve_api_key(api_key)
    client = OpenAI(api_key=key)
    started = datetime.now(timezone.utc).isoformat()
    view_results: Dict[str, Any] = {}

    for stem, prompt in prompts.items():
        png_path = img_dir / f"{stem}.png"
        entry: Dict[str, Any] = {"prompt_chars": len(prompt), "path": str(png_path.name)}
        try:
            resp = client.images.generate(
                model=model,
                prompt=prompt,
                size=size,
                quality=quality,
            )
            data0 = resp.data[0]
            b64 = getattr(data0, "b64_json", None)
            if not b64:
                raise RuntimeError("OpenAI response missing b64_json image payload")
            _save_b64_png(png_path, b64)
            entry["status"] = "ok"
            entry["bytes"] = png_path.stat().st_size
        except Exception as exc:
            entry["status"] = "error"
            entry["error"] = f"{type(exc).__name__}: {exc}"
            if png_path.exists():
                try:
                    png_path.unlink()
                except OSError:
                    pass
        view_results[stem] = entry

    manifest = {
        "model": model,
        "size": size,
        "quality": quality,
        "started_utc": started,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "output_dir": str(img_dir),
        "views": view_results,
        "ok_count": sum(1 for v in view_results.values() if v.get("status") == "ok"),
        "error_count": sum(1 for v in view_results.values() if v.get("status") == "error"),
    }
    (img_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return manifest
