# BREAKLINE Avian: bundled Bevy 0.20 compatibility dependencies

These four third-party crates are **vendored source snapshots** used by the
BREAKLINE-maintained Avian engine fork. They retain their original package
names, copyright notices and MIT/Apache-2.0 license files. Bundling avoids
floating Git dependencies or fetching these code snapshots from a third-party
maintainer's fork during a production build.

| Bundled path | Original upstream | Bevy 0.20 compatible source snapshot |
| --- | --- | --- |
| `vendor/bevy_transform_interpolation` | https://github.com/Jondolf/bevy_transform_interpolation | `andriyDev/bevy_transform_interpolation@4b9ff214bd782e5003da602025c77210abe27545` |
| `vendor/bevy_heavy` | https://github.com/avianphysics/bevy_heavy | `andriyDev/bevy_heavy@a692ccad7cbfc7e62ebd6c54aff49715eab8c0b0` |
| `vendor/glam_matrix_extras` | https://github.com/Jondolf/glam_matrix_extras | `andriyDev/glam_matrix_extras@2a70128a003b82b906897353da3aa05863e42827` |
| `vendor/bevy_mod_debugdump` (development only) | https://github.com/jakobhellermann/bevy_mod_debugdump | `andriyDev/bevy_mod_debugdump@e4666e57e0b046864f5e446c16ecbeadd9608697` |

Avian's two crate manifests use relative `path` dependencies for these
snapshots. The bundled `bevy_heavy` manifest also depends on the bundled
`glam_matrix_extras`, not the remote Git fork.

**Maintenance rules:** Keep this provenance and both license files for every
snapshot. Update upstream versions deliberately and only after Bevy compatibility,
Cargo.lock and physics regression checks pass. Vendoring changes dependency
provenance and reproducibility; it does not itself change collision physics.
