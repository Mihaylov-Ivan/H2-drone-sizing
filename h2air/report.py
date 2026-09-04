from __future__ import annotations
from pathlib import Path
from typing import Any, Mapping

def _f(v, n=3):
    try: return f"{float(v):.{n}f}"
    except Exception: return str(v)

def write_markdown_report(path: str|Path, r: Mapping[str,Any]) -> None:
    s=r["summary"]; gates=r["gates"]
    lines=[
      "# H2AIR Integrated Model — Run Summary", "",
      f"**Model status:** {s['design_status']}", "",
      "## Aircraft state", "",
      f"- Pre-structure screening MTOW: **{_f(s['reference_mass_kg'])} kg**",
      f"- Post-structure screening MTOW: **{_f(s['post_structure_mass_kg'])} kg**",
      f"- Post-structure CG x: **{_f(s['post_structure_cg_x_m'],4)} m**",
      f"- Remaining mass margin to 15 kg: **{_f(s['mass_margin_to_15kg_kg'])} kg**",
      f"- Current-layout static margin: **{_f(100*s['current_static_margin'],2)}% MAC**",
      "",
      "## Performance / mission", "",
      f"- Folded-prop cruise bus power: **{_f(s['cruise_bus_power_W'],1)} W**",
      f"- Forward transition: **{_f(s['forward_transition_duration_s'],2)} s**",
      f"- Reverse transition: **{_f(s['reverse_transition_duration_s'],2)} s**",
      f"- 120-min mission energy: **{_f(s['mission_energy_Wh'],1)} Wh**",
      f"- H₂ used: **{_f(1000*s['h2_used_kg'],1)} g**",
      f"- Final battery SOC: **{_f(100*s['final_battery_soc'],1)}%**",
      "",
      "## Structural screen", "",
      f"- Recommended wing spar: **{_f(s['wing_spar_od_mm'],2)}/{_f(s['wing_spar_id_mm'],2)} mm OD/ID**",
      f"- Recommended boom: **{_f(s['boom_od_mm'],2)}/{_f(s['boom_id_mm'],2)} mm OD/ID**",
      f"- Motor-mount limit thrust: **{_f(s['motor_mount_limit_thrust_N'],1)} N/motor**",
      "",
      "## Integrated gates", ""
    ]
    for g in gates:
        lines.append(f"- **{g['status']}** — {g['name']}: {g.get('note','')}")
    lines += ["", "## Fidelity boundary", "", r["fidelity_note"], ""]
    Path(path).write_text("\n".join(lines),encoding="utf-8")
