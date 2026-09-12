# Project boundaries

- Shell commands use `rtk`; exact output uses `rtk proxy`.
- Edit source and documentation with `apply_patch`.
- `/home/lyh/robot-arm-compliant-control-lab` and desktop inputs are read-only. Never run its environment, tests, or build tools in place.
- Reuse upstream algorithms at commit `7aec01379ff9a8b135cbac75f18102eb9a27ea8f` through a generated, hashed dependency artifact. Do not fork the full repository.
- No hardware I/O, motor enable, stall tests, or changes to upstream experiments.
- User authorized pushing this independent project to a private GitHub repository. Original CAD archives and the user's video remain local; upload only their manifests.
- Simulation settings must be labelled as such; unknown hardware parameters remain unknown.
- World-frame spatial vectors are ordered `[Fx,Fy,Fz,Tx,Ty,Tz]` and `[vx,vy,vz,wx,wy,wz]`, at the named tool-frame origin.
- No seven-axis indexing, nullspace posture control, learned weights, or upstream Panda experiment defaults.
