# docs/

An interactive web version of [`CODE_EXPLAINED.md`](../CODE_EXPLAINED.md).

## Reading it

```sh
./docs/serve.sh            # http://localhost:8000
```

`docs/index.html` is fully self-contained — no network, no CDN, fonts and all
scripts inlined — so you can also just open the file directly:

```sh
xdg-open docs/index.html
```

## What it adds over the markdown

- A sidebar with scroll tracking, filtering (press `/`) and collapsible sections.
- Four interactive panels, which are **direct ports of the C++**:
  - the trapezoidal profile generator (`src/profile.cpp`),
  - the friction deadband,
  - the encoder-to-inches conversion,
  - the whole control loop — profile, PID, feedforward and a simulated
    drivetrain (`src/pid.cpp` plus the loop in `src/chassis.cpp`), with presets
    for each failure mode described in the text.
- Light/dark themes, copy buttons, `[` / `]` to step between sections.

## Editing

The markdown is the source of truth. After editing it:

```sh
node docs/build.mjs
```

`serve.sh` does this automatically when the markdown is newer than the page.

First time on a new machine, install the build-time dependencies (marked for
markdown, KaTeX for math, jsdom for the tests):

```sh
cd docs/.build && npm install
```

Nothing is needed at runtime — KaTeX renders the equations to static HTML at
build time and the fonts are base64-inlined.

## Tests

```sh
node docs/test-port.mjs    # JS ports vs the C++ reference values
node docs/test-page.mjs    # the built page loads and every panel mounts
```

`test-port.mjs` is the important one: it checks the simulations in the browser
panels still agree with `src/profile.cpp` and `src/pid.cpp`, so the page cannot
quietly drift from the robot code.

## Layout

| Path | |
|---|---|
| `index.html` | the built page — the only file you need to serve |
| `build.mjs` | generator: markdown + KaTeX + templates → `index.html` |
| `src/template.html` | page shell |
| `src/styles.css` | all styling |
| `src/widgets.js` | the interactive panels (ports of the C++) |
| `src/app.js` | sidebar, search, collapsing, theme |
| `.build/` | build-time npm packages, not committed |
