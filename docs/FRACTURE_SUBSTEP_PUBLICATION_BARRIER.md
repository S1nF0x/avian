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

The client example in `S1nF0x/ccc` uses the ticket after an exact,
revision-checked resident cut and clean collider dirty-brick journal.
The application, not Avian, owns voxel material and collider-swap proofs.

## Guarantees and boundaries

This barrier enforces ordering and conservative fallback for contact
constraint *rebuilding*; it does **not** create fictional mass or return an
unspent solver impulse to a body. A previous normal impulse ceiling may
already have influenced solver velocities before the barrier runs.
The post-cut manifold is re-solved against the verified collider, or a
rejected proposal is re-solved against the original manifold, but neither
is a bit-for-bit replay of all contacts, joints, and integrated positions.

**Strict physical atomicity (all-or-nothing impulse rollback) is not
claimed.** Achieving that requires a staged solver-body/contact-island
snapshot before provisional contact warm start, plus deterministic rollback
and island replay after voxel/collider publication (including CCD,
restitution, joint constraints, and neighboring manifolds). No such
arbitrary mid-substep snapshot has been added in this commit.

Owner responsibilities:
- Reserve a bounded edit/publication slot *before* source topology commit.
- Apply the same exact material ledger to source topology and resident volume.
- Flush ECS collider publication before narrow-phase refresh and ticket ack.
- Reject tickets if any affected collider remains stale. A partial
  multi-brick cut must not masquerade as a fully published opening.
- Preserve deferred transactions with original source revision and reject
  material mismatches rather than rebasing stale plans.
- Observe finite `SolverConfig::contact_re_solve_max_passes` and count
  `contact_re_solve_dropped`.

Regression coverage in `plugin.rs` verifies: no early re-solve from staging,
implicit timeout fallback, duplicate and rejected acknowledgments, bounded
passes, and stale generation rejection.

## Verification

Run locally:

```shell
cargo test -p avian3d contact_re_solve_tests
cargo test -p avian3d
```

This connector session cannot execute Cargo. The changes were committed to
the fork main branch and adopted by `ccc` through an exact pinned Git SHA,
but no green CI/test result is asserted here.
