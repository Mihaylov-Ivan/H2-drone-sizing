#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from h2air.config import load_model_config
from h2air.pipeline import run_model


def _run_design_study(root:Path,args,base_cfg):
    from h2air.optimization import (
        optimize_design, write_optimization_outputs, optimized_config,
        derive_margin_variant, select_recommended_candidate,
    )
    from h2air.design_coupling import design_overrides
    from h2air.uncertainty import run_uncertainty,write_uncertainty_outputs
    from h2air.sensitivity import local_sensitivity,write_sensitivity_outputs
    from h2air.flight_envelope import build_flight_envelope,write_flight_envelope
    from h2air.calibration import write_template

    study=args.output/"design_study"
    study.mkdir(parents=True,exist_ok=True)

    if args.reuse_reference_search:
        ref=root/"outputs"/"optimization"/"optimization_results.json"
        if not ref.exists():
            raise FileNotFoundError(f"Reference optimization result not found: {ref}")
        print(f"Reusing frozen reference screening search: {ref}")
        opt=json.loads(ref.read_text(encoding="utf-8"))
        write_optimization_outputs(opt,study/"optimization")
    else:
        print(f"Running multidisciplinary screening search ({args.samples} + local refinement candidates)...")
        opt=optimize_design(base_cfg,sample_count=args.samples,local_refinement_count=max(8,args.samples//4),seed=args.seed)
        write_optimization_outputs(opt,study/"optimization")
    nominal_cfg=optimized_config(base_cfg,opt)
    (study/"nominal_optimized_config.json").write_text(json.dumps(nominal_cfg,indent=2)+"\n",encoding="utf-8")
    print(f"Screening-feasible candidates: {opt['feasible_count']} / {opt['sample_count']}")

    print("Running full EMS + 6-DOF + structural verification on the nominal optimum...")
    nominal_verified=run_model(nominal_cfg,study/"nominal_verified",write_histories=not args.no_histories)
    print(f"Nominal verified status: {nominal_verified['summary']['design_status']}")

    print("Deriving margin-biased robust candidate...")
    margin=derive_margin_variant(base_cfg,opt)
    (study/"robust_derivation.json").write_text(json.dumps(margin,indent=2)+"\n",encoding="utf-8")
    robust_cfg=design_overrides(base_cfg,margin["best"]["values"])
    (study/"recommended_robust_config.json").write_text(json.dumps(robust_cfg,indent=2)+"\n",encoding="utf-8")

    print("Running full EMS + 6-DOF + structural verification on the robust candidate...")
    robust_verified=run_model(robust_cfg,study/"recommended_verified",write_histories=not args.no_histories)
    print(f"Robust verified status: {robust_verified['summary']['design_status']}")

    print(f"Running uncertainty propagation on nominal and robust candidates ({args.uncertainty_samples} samples each)...")
    nominal_unc=run_uncertainty(base_cfg,opt["best"]["values"],samples=args.uncertainty_samples,seed=args.seed+35)
    robust_unc=run_uncertainty(base_cfg,margin["best"]["values"],samples=args.uncertainty_samples,seed=args.seed+35)
    write_uncertainty_outputs(nominal_unc,study/"uncertainty_nominal")
    write_uncertainty_outputs(robust_unc,study/"uncertainty_recommended")

    # Accept the robust candidate only when the full integrated verification
    # passes and its uncertainty screen is not worse than the nominal optimum.
    robust_prob=float(robust_unc["summary"]["screening_pass_probability"])
    nominal_prob=float(nominal_unc["summary"]["screening_pass_probability"])
    selected_name=select_recommended_candidate(
        nominal_verified["summary"], robust_verified["summary"],
        nominal_unc["summary"], robust_unc["summary"],
    )
    choose_robust=selected_name=="robust_margin_biased"
    selected_cfg=robust_cfg if choose_robust else nominal_cfg
    selected_verified=robust_verified if choose_robust else nominal_verified
    selected_values=margin["best"]["values"] if choose_robust else opt["best"]["values"]
    selected_unc=robust_unc if choose_robust else nominal_unc

    sens=local_sensitivity(base_cfg,selected_values)
    write_sensitivity_outputs(sens,study/"sensitivity")
    env=build_flight_envelope(selected_cfg)
    write_flight_envelope(env,study/"flight_envelope")
    write_template(study/"calibration_measurements_template.csv")
    (study/"SELECTED_CONFIG.json").write_text(json.dumps(selected_cfg,indent=2)+"\n",encoding="utf-8")

    summary={
        "selection":{
            "selected":selected_name,
            "rule":"full integrated verification must pass; prefer margin-biased candidate when uncertainty pass probability is >= nominal",
            "nominal_uncertainty_pass_probability":nominal_prob,
            "robust_uncertainty_pass_probability":robust_prob,
        },
        "optimization_best":opt["best"],
        "robust_derivation_best":margin["best"],
        "nominal_verified_summary":nominal_verified["summary"],
        "robust_verified_summary":robust_verified["summary"],
        "selected_verified_summary":selected_verified["summary"],
        "selected_uncertainty_summary":selected_unc["summary"],
    }
    (study/"DESIGN_STUDY_SUMMARY.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    print(f"Selected design: {selected_name}")
    print(f"Nominal/robust uncertainty pass: {nominal_prob:.1%} / {robust_prob:.1%}")
    print(f"Design-study outputs: {study}")
    return selected_verified


def main():
    root=Path(__file__).resolve().parent
    ap=argparse.ArgumentParser(description="Run the integrated H2AIR UAV model")
    ap.add_argument("--config",type=Path,default=root/"config"/"baseline.json")
    ap.add_argument("--output",type=Path,default=root/"outputs"/"latest")
    ap.add_argument("--no-histories",action="store_true",help="Skip large time-history CSVs")
    ap.add_argument("--design-study",action="store_true",help="Run MDO search, full verification, uncertainty, sensitivity and flight-envelope study")
    ap.add_argument("--samples",type=int,default=48,help="Latin-hypercube screening samples for --design-study")
    ap.add_argument("--uncertainty-samples",type=int,default=32,help="Uncertainty samples for --design-study")
    ap.add_argument("--seed",type=int,default=42)
    ap.add_argument("--reuse-reference-search",action="store_true",help="Reuse the frozen deterministic 60-candidate optimization result shipped with the package")
    args=ap.parse_args()
    cfg=load_model_config(args.config)
    if args.design_study:
        result=_run_design_study(root,args,cfg)
    else:
        result=run_model(cfg,args.output,write_histories=not args.no_histories)
    s=result["summary"]
    print("H2AIR Integrated Model")
    print("----------------------")
    print(f"Status                    : {s['design_status']}")
    print(f"Reference MTOW            : {s['reference_mass_kg']:.3f} kg")
    print(f"Structurally closed MTOW  : {s['post_structure_mass_kg']:.3f} kg")
    print(f"Closed CG x               : {s['post_structure_cg_x_m']:.4f} m")
    print(f"Cruise bus power          : {s['cruise_bus_power_W']:.1f} W")
    print(f"120-min mission energy    : {s['mission_energy_Wh']:.1f} Wh")
    print(f"Closed static margin      : {100*s['current_static_margin']:.2f}% MAC")
    return 0
if __name__=="__main__":raise SystemExit(main())
