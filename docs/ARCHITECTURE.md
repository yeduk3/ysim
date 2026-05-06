# Architecture

> Owner: **Planner**. Generators read this to know boundaries; Estimators read this to detect violations.

The Planner authors this file from the PRD/FRD/BDD and updates it whenever the system grows a new boundary. Keep it at the level of "boxes and arrows that take more than one file to derive" — file-level structure belongs in code, not here.

## Suggested sections

(Delete this section once the file is populated.)

1. **System purpose** — one paragraph from PRD, in your own words.
2. **Boxes** — the main components and what each owns. One paragraph per box.
3. **Arrows** — how the boxes talk: protocols, sync vs async, who initiates.
4. **Boundaries** — invariants that must hold across boxes (the things the Estimator should flag if violated).
5. **Open structural questions** — things you have not decided yet, with the constraints that will force the decision.
