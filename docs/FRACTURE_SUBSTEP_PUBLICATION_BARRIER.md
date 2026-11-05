# Same-substep fracture collider publication barrier

Avian's normal contact impulse ceiling is a **constitutive solver limit**,
not proof that an owning voxel engine has committed a new shape. The
normal/relaxation solver may already have applied a bounded impulse before
the `ContactBreakageSystems::Apply` callback runs.

This fork provides a **generation-tagged, two-phase contact publication
authorization** on `ContactReseolveQueue`:

1. `stage_fracture_contact(contact_id)` records a provisional breakage
   proposal and returns a `FractureCommitTicket`. Staging alone never
   requests a changed-geometry re-solve.
2. The owner validates the original source revision, exact voxel material
   ledger and available execution budget; it commits the resident voxel cut,
   publishes its new colliders, then calls the affected narrow-phase contact
   refresh. Only afterward may it call
   `acknowledge_fracture_publication(ticket)`.
3. A failed or deferred source edit, missing resident, or unpublished
   collider calls `reject_fracture_publication(ticket)`. Avian rebuilds
   the **unchanged load-bearing contact** at its normal unconstrained
   capacity and performs a bounded same-substep re-solve.
4. Any ticket still awaiting acknowledgment at
   `ContactBreakageSystems::ReSolve` is automatically rejected and
   registered as an unconfirmed fracture. The counter
   `avian/solver/contact_fracture_unconfirmed` exposes these failures.
5. A new ticket generation is issued for each new staging episode.
   A ticket from a previous episode/substep cannot publish a contact that
   reused the same `ContactId`.

The `S1nF0x/reasons` client uses the ticket after an exact, revision-checked
resident cut and collider publication. The application, not Avian, owns voxel
material and collider-swap proofs.

## Guarantees and boundaries

The substep pipeline now performs these operations in order:

1. The owner publishes the exact material-volume and collider replacement.
2. The owner commits the current solver pose of the affected dynamic
   contact island to ECS Position/Rotation and rebases its solver deltas.
3. Avian refreshes changed collider AABBs, collects BVH pairs for newly
   published bodies, and updates the narrow-phase manifolds.
4. Before contact-status changes prune old constraints, Avian refreshes
   solver-body indices by stable entity identity and reverses the current
   substep's net impulse for every replaced contact.
5. The owner transfers a dynamic source's post-rollback rigid velocity field
   to its surviving and detached components. Avian recomputes changed mass
   properties, initializes new solver bodies, rebuilds the affected dynamic
   contact island, and re-solves its constraints.
6. Static and kinematic contacts remain island boundaries; a shared ground
   collider does not connect unrelated dynamic bodies.
7. Velocity damping runs, then the current substep's position integration runs.

New fragments can therefore create ordinary contact constraints with the
impactor and neighboring bodies in the same substep. Every dynamic body whose
velocity is changed by rollback brings its other contacts into the same local
island re-solve. Constraints outside that island retain their already-solved
state and are not solved a second time.

This is a local substep contact transaction, not a bit-for-bit replay of the
entire fixed step or contact island. Impulses and positions committed by
earlier substeps remain committed; only the current substep's replaced-contact
impulses are reversed before its position integration. Joints, CCD, and
end-of-step restitution are not rolled back and replayed as part of the
fracture transaction. Cross-platform deterministic replay is not claimed.

Owner responsibilities remain unchanged:
- Validate the source revision and exact material ledger before committing topology.
- Flush the collider replacement before acknowledging the publication ticket.
- Reject or defer incomplete collider publication.
- Keep contact re-solve work within the configured per-substep pass and ID limits.

The solver queue tests verify that rollback and re-solve use the identical
sorted contact set, including the full-graph fallback. They also retain
coverage for unconfirmed, rejected, duplicate, stale-generation, and bounded
requests.

## Verification

Run locally:

```shell
cargo test -p avian3d contact_re_solve_tests
cargo test -p avian3d
```

This connector session cannot execute Cargo. The changes are committed directly
to the fork main branch and consumed by the Reasons client through its git
dependency. No local Cargo test result is asserted here.
