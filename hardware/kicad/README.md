# KiCad CLI via Docker

This directory uses the official `kicad/kicad:9.0` Docker image so KiCad CLI
commands can run without installing KiCad on the host.

## Prerequisites

- Docker installed and working for the current user
- The repo checked out locally

## Wrapper

Run KiCad CLI through `scripts/kicad-cli.sh` from the repo root:

```bash
./scripts/kicad-cli.sh version
./scripts/kicad-cli.sh sch erc hardware/kicad/demo/demo.kicad_sch --format json
./scripts/kicad-cli.sh pcb drc hardware/kicad/demo/demo.kicad_pcb --format json --exit-code-violations
```

The wrapper:

- pins KiCad to `kicad/kicad:9.0`
- mounts the current directory at `/work`
- runs the container as the current UID/GID to avoid root-owned outputs
- forwards all remaining arguments to `kicad-cli`

If you want to mount a different directory, use `--workdir`:

```bash
./scripts/kicad-cli.sh --workdir hardware/kicad/demo sch erc demo.kicad_sch --format json
```

## Demo project

`hardware/kicad/demo/` contains a tiny schematic and PCB used to prove the
Docker wrapper works end-to-end.

Expected behavior from the repo root:

```bash
./scripts/kicad-cli.sh sch erc hardware/kicad/demo/demo.kicad_sch --format json
./scripts/kicad-cli.sh pcb drc hardware/kicad/demo/demo.kicad_pcb --format json --exit-code-violations
```

On a clean checkout with Docker available, both commands should complete and
write JSON reports in the mounted working directory. If you run them from the
repo root as shown above, KiCad writes `demo-erc.json` and `demo-drc.json` to
the repo root unless you pass `--output` or use `--workdir hardware/kicad/demo`.

## Known limitations

- KiCad is intentionally pinned to `9.0`; update the wrapper if the project
  should move to a newer KiCad release.
- The wrapper only mounts one directory. Input paths must live under the
  mounted tree.
- Generated report files follow KiCad CLI behavior and default to the mounted
  working directory rather than the source file's directory.
