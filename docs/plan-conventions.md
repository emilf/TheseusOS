# Plan Conventions

This page defines how plans in `docs/plans/` must be written and maintained.

## Plan Lifecycle

1. Draft intent.
2. State assumptions and evidence constraints.
3. Define the invariants the work is meant to establish.
4. Implement incrementally.
5. Convert completed work into present-tense invariants.

## Checklist Markers

- `[ ] TODO:` future work that is not yet true in the repo.
- `[~] IN PROGRESS:` work currently underway.
- `[x] IMPLEMENTED:` a present-tense invariant backed by code.
- `[!] RISK:` a known footgun, mismatch, or hazard in the current implementation.

## Writing Rules

- Do not describe unimplemented ideas as if they already exist.
- Prefer present tense for `[x] IMPLEMENTED:` items.
- Prefer concrete, linkable sections over vague narrative blobs.
- Link aggressively to:
  - relevant axioms
  - implementing modules
  - related plans
  - evidence in `docs/_inventory.md` when helpful

## Updating Rules

When editing a plan:

- Preserve implemented invariants unless you are intentionally revising them.
- If an invariant changes, document why.
- Add a short "Plan vs Repo" mismatch note when reality diverges from older intent.
- Move completed design truth into `docs/axioms/` when it becomes globally binding.

## Model Guidance

Before making architectural changes:

1. Read the relevant axiom sections.
2. Read the relevant plan sections.
3. Confirm the proposed change does not violate documented invariants.
4. If it does, update the axioms explicitly as part of the same change.

## Scope Discipline

Plans are allowed to contain intent, risks, and sequencing.
Axioms are not.

If a sentence starts sounding like "we should probably" or "eventually," it belongs in a plan, not an axiom.
