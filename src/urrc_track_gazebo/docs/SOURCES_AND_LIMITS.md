# Monza source provenance and limits

- Plan geometry: `bacinger/f1-circuits`, revision `394d8fbe70ef2c0b0c8d23ff7bee61fa09606055`; immutable URL and hash are in `tracks/raw/SOURCES.json`.
- Upstream coordinates are unofficial. The upstream MIT license is retained in `docs/references/upstream_LICENSE.md`.
- Official visual/lap reference: Formula 1 Monza page, 5,793 m. Official artwork is not redistributed or used as a texture.
- XY uses the source vertices and a 0.2 scale without target-length fitting. Sparse-source interpolation can differ from a surveyed circuit.
- Width is an engineering estimate from the broad 10–12 m full-scale range, not surveyed left/right boundaries.
- Elevation uses smoothed SRTM 30 m terrain calibrated to a published 12.8 m full-scale range. It is not a road survey and contains no camber or surface bumps.
- The 20 grid boxes are approximate: 1.2 × 0.5 m, 1.6 m successive spacing, 3.2 m same-column spacing.
- Static geometry validation does not replace Gazebo server, GUI, vehicle-contact, sensor, or race testing on the target machine.
