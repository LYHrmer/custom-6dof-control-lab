# First-stage implementation

1. Preserve and hash ZIP, processed package, and advice; fingerprint upstream before/after. Only read upstream.
2. Reproduce audit: chain, inertias, duplicate terminal instance, mesh counts, limits, compilation, singularity. Separate evidence from inference.
3. Isolated venv plus a minimal upstream controller wheel generated from the pinned Git object, with licence and per-file hashes.
4. Derive versioned free-space MJCF with six ideal torque actuators, joint sensors and explicit tool site. Remove duplicate tool inertia/geometry only; retain its coordinate frame. No global inertia-sign changes, uncalibrated friction, or collision claims.
5. Verify independent URDF FK/Jacobian and potential-energy gravity against MuJoCo; torque limiting, gravity hold and slow impedance perturbation/recovery using the pinned controller.
6. Document exact commands, evidence, support boundaries, motor questions and future milestones. Recheck all preserved inputs and upstream.

Acceptance: repeatable audit/build; nv=nu=6; named indexing; finite-difference 6D Jacobian and gravity checks; stable unsaturated nominal free-space controls; singularity/invalid-input handling; tests passing; upstream fingerprints unchanged.

Completed first stage: 50 tests pass, three complete simulation runs pass, host six-axis C++ parity passes, all 294 preserved input files verify. See docs/phase1_report.md and reports/phase1_results.json.

Upstream audit qualification: tracked files match the reference commit and baseline runtime assets remain byte-identical; full-tree hashes differ due observed concurrent planning, new tests/tools, caches and new temporary results. Differences are reported without reverting another session's work. No claim of full-tree immutability is made.

GitHub: user subsequently authorized upload and selected a private repository, LYHrmer/custom-6dof-control-lab. Original CAD archives, video and extracted reference source remain local; manifests and verified dependency artifact are tracked. Claude calls paused at user request pending quota recovery.
