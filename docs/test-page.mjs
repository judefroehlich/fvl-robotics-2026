// Loads the built page in a DOM and checks that the scripts run cleanly and
// every interactive panel actually mounts.
//
//   node docs/test-page.mjs

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { JSDOM, VirtualConsole } from './.build/node_modules/jsdom/lib/api.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.join(here, 'index.html'), 'utf8');

const errors = [];
const vc = new VirtualConsole();
vc.on('jsdomError', (e) => errors.push(e.message + '\n    ' + (e.stack || '').split('\n')[1]));
vc.on('error', (...a) => errors.push('console.error: ' + a.join(' ')));

const dom = new JSDOM(html, {
  runScripts: 'dangerously',
  pretendToBeVisual: true,
  virtualConsole: vc,
  url: 'http://localhost/',
});
const { window } = dom;

// jsdom has no canvas backend; a no-op 2d context lets the drawing code run so
// we still exercise every code path in the widgets.
const noop = new Proxy({}, { get: (t, k) => (k === 'canvas' ? {} : () => {}) });
window.HTMLCanvasElement.prototype.getContext = () => noop;

await new Promise((r) => window.addEventListener('load', r, { once: true }));
await new Promise((r) => setTimeout(r, 50));

const doc = window.document;
const $ = (s) => doc.querySelector(s);
const $$ = (s) => Array.from(doc.querySelectorAll(s));

let fails = 0;
function ok(what, cond, detail) {
  if (cond) console.log('  ok   ' + what + (detail ? '  ' + detail : ''));
  else { fails++; console.log('  FAIL ' + what + (detail ? '  ' + detail : '')); }
}

console.log('page scripts');
ok('no script errors', errors.length === 0, errors.length ? '\n    ' + errors.join('\n    ') : '');

console.log('\nchrome');
ok('title replaced', !doc.title.includes('{{'), `"${doc.title}"`);
ok('no leftover placeholders', !html.includes('{{'),
   (html.match(/\{\{\w+\}\}/g) || []).join(' '));
const rt = $('#readingTime').textContent || '';
ok('reading time plausible', /^[\d,]{4,} words/.test(rt), `"${rt}"`);
ok('theme button labelled', ($('#themeBtn').textContent || '').length > 0, `"${$('#themeBtn').textContent}"`);
ok('toc links built', $$('#toc a').length > 40, $$('#toc a').length + ' links');
ok('copy buttons added', $$('.copy-btn').length > 90, $$('.copy-btn').length + ' buttons');

console.log('\nwidgets');
for (const host of $$('[data-widget]')) {
  const id = host.dataset.widget;
  const sliders = host.querySelectorAll('input[type=range]').length;
  const canvases = host.querySelectorAll('canvas').length;
  const verdict = host.querySelector('.verdict');
  const failed = host.querySelector('.verdict.bad') && host.textContent.includes('failed to load');
  ok(`${id} mounted`, sliders > 0 && !failed, `${sliders} sliders, ${canvases} canvases`);
  ok(`${id} produced a verdict`, verdict && verdict.textContent.trim().length > 20,
     verdict ? '"' + verdict.textContent.trim().slice(0, 64) + '…"' : 'none');
}
ok('loop widget has presets', $$('#w-loop .presets button').length === 7,
   $$('#w-loop .presets button').length + ' presets');

console.log('\ninteraction');
// Collapsing a chapter should hide its body.
const ch = $('.chapter');
$('.chapter .toggle').dispatchEvent(new window.MouseEvent('click', { bubbles: true }));
ok('collapse toggles class', ch.classList.contains('collapsed'));
$('#expandAll').dispatchEvent(new window.MouseEvent('click', { bubbles: true }));
ok('expand all clears it', !ch.classList.contains('collapsed'));

// Filtering the contents list.
const search = $('#search');
search.value = 'deadband';
search.dispatchEvent(new window.Event('input', { bubbles: true }));
const visible = $$('#toc li').filter((li) => !li.classList.contains('hidden'));
ok('search filters the toc', visible.length > 0 && visible.length < $$('#toc li').length,
   `${visible.length} of ${$$('#toc li').length} shown`);
ok('search highlights match', $$('#toc mark').length > 0, $$('#toc mark').length + ' marks');

console.log(fails ? `\nFAILED (${fails})` : '\nALL PASS');
process.exit(fails ? 1 : 0);
