# Integration notes

The cumulative development modules were moved behind a clean `h2air` package and renamed by physical function. A single `run_model.py` now orchestrates the model and passes results in memory between subsystems.

The historical phase documents are retained only under `docs/development_history/`. There are no `run_phaseX.py` files in normal operation.

Structural feedback is now applied to actual ledger components (wing structure, boom tubes and tail structures), after which 3-D mass properties are recalculated. This produces an integrated post-structure mass/CG/inertia snapshot rather than leaving the Phase-7 mass correction isolated.
