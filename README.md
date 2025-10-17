# Animal Behavior — Simulations

This repository hosts simple, beautiful web-based simulations for a course on Animal Behavior.

Currently included:

- Schelling's Model of Segregation (interactive, controllable, runs until convergence or step-by-step)

## Quick Start

Open `web/index.html` directly in your browser, or serve the `web/` folder with a local server:

```
python3 -m http.server --directory web 8080
# then visit http://localhost:8080
```

No build step or external dependencies are required.

## Schelling Controls

- Grid size: Number of rows/columns in the grid.
- Occupancy: Percentage of cells initially filled with agents.
- Type A ratio: Split between Type A and Type B among agents.
- Minimum similar neighbors: Tolerance threshold for satisfaction.
- Wrap edges: Use a torus topology; otherwise edges are borders.
- Delay per step: Rendering delay when running until convergence.
- Max steps: Safety cap to stop extremely slow convergence.

Actions:

- Initialize: Create a new random configuration with current parameters.
- Step: Advance a single iteration.
- Run: Run iterations with delay until convergence (or pause).

## Notes

- The model uses Moore neighborhoods (8 neighbors).
- Agents with no neighbors are considered satisfied.
- Each step moves unsatisfied agents to random empty cells in parallel.
## Branding

- Favicon: Mouse icon at `web/icon.svg`. Replace if desired.
- HUJI logo: The header uses `web/huji-logo.svg`. Replace this file with the institution-approved asset to update the logo.
