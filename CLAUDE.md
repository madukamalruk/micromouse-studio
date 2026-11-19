# CLAUDE.md

All project rules live in **[AGENTS.md](AGENTS.md)** — read it first, every session. Scope: `docs/prd.md`. Progress: `docs/plan.md`.

Key reminders:
- Firmware (`firmware/`) includes only `mmhal/*.h` + C stdlib. Simulator never alters firmware logic.
- Run the slice's verification commands and show output before ticking a checkbox.
- Never push without asking. Scan staged diffs for secrets before committing.
