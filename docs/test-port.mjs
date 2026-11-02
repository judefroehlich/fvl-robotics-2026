// Checks the JavaScript ports in docs/src/widgets.js against the reference
// values produced by the C++ unit tests, so the interactive panels cannot
// silently drift from src/profile.cpp and src/pid.cpp.
//
//   node docs/test-port.mjs

import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const src = fs.readFileSync(path.join(here, 'src/widgets.js'), 'utf8');

// Minimal stubs: widgets.js only touches the DOM at the very bottom.
const sandbox = {
  window: {},
  document: { addEventListener() {}, querySelectorAll: () => [], createElement: () => ({ style: {} }) },
  getComputedStyle: () => ({ getPropertyValue: () => '#000' }),
  requestAnimationFrame: () => {},
  console,
  Math,
};
sandbox.window.addEventListener = () => {};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(src, sandbox);

const { TrapezoidProfile, PID, Settler } = sandbox.window.__docsInternals;

let fails = 0;
function check(what, got, want, tol = 1e-6) {
  const ok = Math.abs(got - want) <= tol;
  if (!ok) { fails++; console.log(`  FAIL ${what.padEnd(38)} got ${got} want ${want}`); }
  else console.log(`  ok   ${what.padEnd(38)} ${typeof got === 'number' ? got.toFixed(4) : got}`);
}

console.log('TrapezoidProfile vs C++ reference values');
// These numbers come from the C++ test run: d=24, v=28, a=60.
const p = new TrapezoidProfile(24, 28, 60);
check('duration', p.tTotal, 28 / 60 + 24 / 28, 1e-9);
check('duration (C++ printed 1.3238)', p.tTotal, 1.3238, 5e-5);
check('cruise velocity', p.cruiseV, 28);
check('position at t_accel', p.at(28 / 60).position, (28 * 28) / 120, 1e-9);
check('shape', p.shape === 'trapezoidal' ? 1 : 0, 1);

const tri = new TrapezoidProfile(2, 28, 60);
check('triangular peak = sqrt(a*d)', tri.cruiseV, Math.sqrt(120), 1e-9);
check('triangular duration (C++ 0.3651)', tri.tTotal, 0.3651, 5e-5);
check('triangular midpoint = d/2', tri.at(tri.tTotal / 2).position, 1, 1e-9);

const turn = new TrapezoidProfile(90, 240, 600);
check('turn duration (C++ 0.7746)', turn.tTotal, 0.7746, 5e-5);
check('turn peak (C++ 232.379)', turn.cruiseV, 232.379, 5e-4);

console.log('\nprofile self-consistency (integral of v equals position)');
for (const [name, d, v, a] of [['fwd trapezoid', 24, 28, 60], ['rev trapezoid', -36, 28, 60],
                               ['fwd triangle', 2, 28, 60], ['rev triangle', -1.5, 28, 60],
                               ['boundary', (28 * 28) / 60, 28, 60]]) {
  const pr = new TrapezoidProfile(d, v, a);
  const dt = 1e-5;
  let integrated = 0, peakV = 0, peakA = 0;
  for (let t = 0; t < pr.tTotal; t += dt) {
    const s = pr.at(t + dt / 2);
    integrated += s.velocity * dt;
    peakV = Math.max(peakV, Math.abs(s.velocity));
    peakA = Math.max(peakA, Math.abs(s.acceleration));
  }
  check(`${name}: integral(v) == distance`, integrated, d, 1e-3);
  check(`${name}: ends at distance`, pr.at(pr.tTotal).position, d);
  check(`${name}: ends at rest`, pr.at(pr.tTotal).velocity, 0);
  check(`${name}: |v| <= cap`, peakV <= v + 1e-6 ? 1 : 0, 1);
  check(`${name}: |a| <= cap`, peakA <= a + 1e-6 ? 1 : 0, 1);
}

console.log('\ndegenerate inputs');
check('zero distance duration', new TrapezoidProfile(0, 28, 60).tTotal, 0);
check('zero constraints -> step', new TrapezoidProfile(10, 0, 0).at(0).position, 10);
check('negative constraints abs', new TrapezoidProfile(10, -28, -60).tTotal,
      new TrapezoidProfile(10, 28, 60).tTotal, 1e-12);
check('before start at rest', p.at(-5).position, 0);
check('past end holds target', p.at(99).position, 24);

console.log('\nPID vs C++ reference values');
check('proportional', new PID({ kP: 100, kI: 0, kD: 0, iz: 0 }).update(5, 0.01), 500);
const sat = new PID({ kP: 100, kI: 0, kD: 0, iz: 0 });
check('output clamped', sat.update(1000, 0.01), 12000);
const d1 = new PID({ kP: 0, kI: 0, kD: 10, iz: 0 });
check('no D on first call', d1.update(5, 0.01), 0);
check('D on second call', d1.update(6, 0.01), 1000);
const i1 = new PID({ kP: 0, kI: 50, kD: 0, iz: 0 });
i1.update(2, 0.1);
check('integral accumulates', i1.update(2, 0.1), 50 * 0.4);
check('outside integral zone', new PID({ kP: 0, kI: 50, kD: 0, iz: 1 }).update(5, 0.1), 0);
const w = new PID({ kP: 0, kI: 1, kD: 0, iz: 0 }, 100);
for (let k = 0; k < 10000; k++) w.update(100, 0.01);
check('windup capped', w.update(100, 0.01), 100);
const x = new PID({ kP: 0, kI: 50, kD: 0, iz: 0 });
x.update(2, 0.1);
check('integral reset on sign flip', x.update(-2, 0.1), -10);
check('dt=0 is P only', new PID({ kP: 100, kI: 50, kD: 10, iz: 0 }).update(3, 0), 300);

console.log('\nSettler');
const s1 = new Settler({ tolerance: 0.5, velocityTolerance: 1, settleTimeMs: 150 });
check('not settled immediately', s1.update(0.1, 0.1, 10) ? 1 : 0, 0);
let settled = false;
for (let k = 0; k < 14; k++) settled = s1.update(0.1, 0.1, 10);
check('settled after 150ms', settled ? 1 : 0, 1);
const s2 = new Settler({ tolerance: 0.5, velocityTolerance: 1, settleTimeMs: 150 });
for (let k = 0; k < 14; k++) s2.update(0.1, 0.1, 10);
s2.update(0.1, 5, 10);
check('velocity resets the timer', s2.update(0.1, 0.1, 10) ? 1 : 0, 0);

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASS');
process.exit(fails ? 1 : 0);
