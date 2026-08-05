# Design notes

> Optional. Per-slice design sketches (data shapes, sequence sketches, ASCII diagrams) that are too detailed for `ARCHITECTURE.md` but that the **Generator** will need before writing code.
>
> One file per slice, named after the behavior id or PLAN milestone. Delete or archive after the slice ships if the design is now obvious from the code.

## Active simulation design

- [PD continuum cloth와 coupled contact 변경 설계](pd-continuum-cloth-contact.md):
  `TriangularCloth`의 triangle strain, cotangent bending, coupled
  vertex–triangle/edge–edge contact, contact stiffness GUI와 검증 기준.
- [PBD / 충돌 시스템 handoff](pbd-system-handoff.md): 현재 CPU PBD/PD 및
  collision pipeline의 구현 상태와 남은 작업.
