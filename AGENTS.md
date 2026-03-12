# AGENTS.md

Repository-wide operating rules for TheseusOS.

## Repository Philosophy

- The kernel boots as a UEFI executable in a single-binary boot flow.
- The codebase is `no_std` unless a file explicitly documents otherwise.
- Invariants are part of the architecture; they are not optional commentary.
- Unsafe code requires explicit contracts, not vibes.
- Plans and axioms are first-class architectural artifacts and must track reality.

## Global Agent Rules

- Do not invent architecture, invariants, or APIs.
- Any architectural claim must be backed by repo evidence: file path + symbol/section.
- Keep diffs minimal, reviewable, and easy to verify.
- Do not mix logic changes into mechanical documentation refactors.
- When reality is unclear, write `TODO`, `UNKNOWN`, or a mismatch note instead of guessing.
- Treat `docs/_inventory.md` as an evidence dump, not as binding architecture on its own.

## Documentation Synchronization Rule

Any architectural code change must:

1. Update the relevant checklist items in `docs/plans/`.
2. Update `docs/axioms/` if a binding truth changed.
3. Update `docs/map.md` so links stay bidirectional.
4. Keep affected Rust module doc headers accurate.

## Before Writing Code

- Identify the governing axiom sections.
- Identify the governing plan sections.
- Restate the invariants that the change must preserve.

## After Writing Code

- Confirm the invariants still hold.
- Update plan status markers.
- Update axioms if the implementation changed architectural truth.
- Update module headers when their source-of-truth links or safety notes changed.

## Documentation Topology

- `docs/index.md` is the human-first entry point.
- `docs/axioms/` contains binding architectural truths.
- `docs/plans/` contains evolving work, checklists, mismatch notes, and risks.
- `docs/map.md` links plans ↔ axioms ↔ modules.
- `docs/_inventory.md` is an evidence-only implementation inventory.

Scoped `AGENTS.md` files may add stricter local rules, but they must not contradict this file.
