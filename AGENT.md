# Workspace Conventions

- Do not write temporary outputs to the system `/tmp` directory.
- Use the repo-local [tmp](/home/liuyi/projects/ee4308_proj2/tmp) directory for bags, plots, logs, and sweep artifacts.
- When adding commands to docs or scripts, prefer paths like `tmp/proj2_run1`, `tmp/proj2_plots`, and `tmp/proj2_sweeps/...`.
- If a comparison or sweep experiment is expected to run for a long time, prefer giving the user the exact command to run locally instead of running the long experiment here.
