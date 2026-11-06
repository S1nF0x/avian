# Same-substep fracture contact update

The material owner is responsible for the authoritative voxel cut and the
matching collider/mass publication. Once that transaction is committed, it
queues the old affected contact island before Avian's AABB refresh.

The substep then runs one ordinary physics path:

1. Rebase affected bodies to their current solver pose.
2. Refresh moved collider AABBs.
3. Update BVH pairs and narrow-phase manifolds.
4. Add newly discovered contacts to the same bounded re-solve batch.
5. Roll back the replaced contacts' current-substep impulses.
6. Rebuild the affected constraints and solve them once.
7. Integrate positions once.

The queue contains only contact IDs or a full-graph fallback. It has no
publication tickets, compatibility states, retry protocol, or configurable
multi-pass loop: the owner requests a rebuild only after publishing the
authoritative geometry. Queue overflow promotes the transaction to a full
contact-graph rebuild rather than silently dropping part of the island.

This is a local substep transaction, not a replay of earlier substeps. Their
already-integrated positions remain committed. The solver reverses only the
current-substep impulses for the affected contact island before this substep's
position integration.
