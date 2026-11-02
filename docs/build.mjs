// Builds docs/index.html from CODE_EXPLAINED.md.
//
// Math is rendered to static HTML with KaTeX and the fonts are base64-inlined,
// so the finished page has no runtime dependencies at all: it works served
// over localhost or opened straight from disk.
//
//   node docs/build.mjs

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { marked } from './.build/node_modules/marked/lib/marked.esm.js';
import katex from './.build/node_modules/katex/dist/katex.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '..');
const vendor = path.join(here, '.build/node_modules/katex/dist');

const read = (p) => fs.readFileSync(p, 'utf8');

/* ---------------------------------------------------------------- math --- */

function renderMath(tex, displayMode) {
  try {
    return katex.renderToString(tex, {
      displayMode,
      output: 'html',
      throwOnError: true,
      strict: false,
    });
  } catch (err) {
    console.error(`\n  KaTeX failed on: ${tex}\n  ${err.message}\n`);
    process.exitCode = 1;
    return `<code class="math-error">${tex}</code>`;
  }
}

// $$...$$ on its own line, and $...$ inline. Registered as marked extensions
// so that fenced code and inline code spans are tokenized first and never
// reach these rules.
const mathExtensions = [
  {
    name: 'mathBlock',
    level: 'block',
    start: (src) => src.indexOf('$$'),
    tokenizer(src) {
      const m = /^\$\$([\s\S]+?)\$\$(?:\n+|$)/.exec(src);
      if (m) return { type: 'mathBlock', raw: m[0], text: m[1].trim() };
    },
    renderer: (t) => `<div class="math-block">${renderMath(t.text, true)}</div>`,
  },
  {
    name: 'mathInline',
    level: 'inline',
    start: (src) => src.indexOf('$'),
    tokenizer(src) {
      const m = /^\$([^$\n]+?)\$/.exec(src);
      if (m) return { type: 'mathInline', raw: m[0], text: m[1].trim() };
    },
    renderer: (t) => renderMath(t.text, false),
  },
];

/* --------------------------------------------------------------- slugs --- */

// Matches GitHub's anchor generation, so the cross-references already written
// into the markdown keep working.
function slug(raw) {
  return raw
    .replace(/<[^>]+>/g, '')
    .replace(/`/g, '')
    .toLowerCase()
    .replace(/[^a-z0-9 _-]/g, '')
    .trim()
    // One hyphen per whitespace character, not per run: GitHub does the same,
    // so a heading with a stripped em dash keeps its double hyphen.
    .replace(/\s/g, '-')
    .replace(/^-+|-+$/g, '');
}

/* ------------------------------------------------------------- widgets --- */

// Interactive panels, keyed by the heading slug they are inserted after.
const widgets = {
  '41-the-idea': {
    id: 'w-profile',
    title: 'Try it: the profile generator',
    blurb:
      'The exact math from <code>profile.cpp</code>, ported to JavaScript. Drag the sliders and watch the shape change — shrink the distance far enough and the trapezoid collapses into a triangle.',
  },
  '39-why-p-alone-always-stops-short-the-friction-deadband': {
    id: 'w-deadband',
    title: 'Try it: the friction deadband',
    blurb:
      'How far short a pure-P controller parks, for a given friction threshold and gain.',
  },
  '53-small-member-functions': {
    id: 'w-units',
    title: 'Try it: unit conversion',
    blurb:
      'The same arithmetic as <code>degrees_to_inches</code>. Set it to your robot and check it against a tape measure.',
  },
  '54-follow_profile--the-control-loop': {
    id: 'w-loop',
    title: 'Try it: the whole control loop',
    blurb:
      'Profile, PID, feedforward and a simulated drivetrain, all ported from the real code. This is the same simulation that was used to pick the shipped gains — including the two findings described above.',
  },
};

function widgetHtml(key) {
  const w = widgets[key];
  return `
<div class="widget" id="${w.id}">
  <div class="widget-head">
    <span class="widget-tag">interactive</span>
    <h4>${w.title}</h4>
  </div>
  <p class="widget-blurb">${w.blurb}</p>
  <div class="widget-body" data-widget="${w.id}"></div>
</div>`;
}

/* ----------------------------------------------------------- rendering --- */

marked.use({ extensions: mathExtensions, gfm: true });

const md = read(path.join(root, 'CODE_EXPLAINED.md'));
const tokens = marked.lexer(md);

// No options: marked.parser() then uses the defaults registered via use(),
// which is where the math extensions live.
const renderTokens = (ts) => (ts.length ? marked.parser(ts) : '');

let pageTitle = 'Code Explained';
let lede = '';
const toc = [];
const chapters = [];
let chapter = null;
let sub = null;
let preamble = [];

function closeSub() {
  if (!chapter || !sub) return;
  let body = renderTokens(sub.tokens);
  if (widgets[sub.id]) body += widgetHtml(sub.id);
  chapter.subs.push({ ...sub, body });
  sub = null;
}

function closeChapter() {
  closeSub();
  if (!chapter) return;
  chapter.intro = renderTokens(chapter.tokens);
  chapters.push(chapter);
  chapter = null;
}

for (const token of tokens) {
  if (token.type === 'heading' && token.depth === 1) {
    pageTitle = token.text;
    continue;
  }
  if (token.type === 'heading' && token.depth === 2) {
    closeChapter();
    const id = slug(token.text);
    chapter = { id, raw: token.text, html: renderTokens([{ ...token, depth: 2 }]), tokens: [], subs: [] };
    // Strip the <h2> wrapper; the template supplies its own.
    chapter.heading = chapter.html.replace(/^<h2[^>]*>|<\/h2>\s*$/g, '');
    toc.push({ id, label: token.text, children: [] });
    continue;
  }
  if (token.type === 'heading' && token.depth === 3 && chapter) {
    closeSub();
    const id = slug(token.text);
    const html = renderTokens([{ ...token, depth: 3 }]);
    sub = { id, raw: token.text, heading: html.replace(/^<h3[^>]*>|<\/h3>\s*$/g, ''), tokens: [] };
    toc[toc.length - 1].children.push({ id, label: token.text });
    continue;
  }
  if (token.type === 'hr') continue; // chapters are visually separated already

  if (sub) sub.tokens.push(token);
  else if (chapter) chapter.tokens.push(token);
  else preamble.push(token);
}
closeChapter();

lede = renderTokens(preamble);

/* ----------------------------------------------------- assemble the page -- */

const tocHtml = toc
  .map(
    (c) => `
    <li class="toc-chapter">
      <a href="#${c.id}" data-target="${c.id}">${stripTags(c.label)}</a>
      ${
        c.children.length
          ? `<ul>${c.children
              .map((s) => `<li><a href="#${s.id}" data-target="${s.id}">${stripTags(s.label)}</a></li>`)
              .join('')}</ul>`
          : ''
      }
    </li>`
  )
  .join('');

function stripTags(s) {
  return s.replace(/`/g, '').replace(/<[^>]+>/g, '').replace(/&/g, '&amp;').replace(/</g, '&lt;');
}

const bodyHtml = chapters
  .map(
    (c, i) => `
<section class="chapter" id="${c.id}" data-chapter="${i + 1}">
  <h2 class="chapter-title">
    <button class="toggle" aria-expanded="true" title="Collapse section"><svg viewBox="0 0 16 16" aria-hidden="true"><path d="M4 6l4 4 4-4"/></svg></button>
    <a class="anchor" href="#${c.id}">${c.heading}</a>
  </h2>
  <div class="chapter-body">
    ${c.intro}
    ${c.subs
      .map(
        (s) => `
    <section class="sub" id="${s.id}">
      <h3><a class="anchor" href="#${s.id}">${s.heading}</a></h3>
      ${s.body}
    </section>`
      )
      .join('')}
  </div>
</section>`
  )
  .join('');

/* ------------------------------------------------------- inline assets --- */

// KaTeX ships many font weights; inline the ones its stylesheet declares so
// the page carries its own typography.
let katexCss = read(path.join(vendor, 'katex.min.css'));
const fontDir = path.join(vendor, 'fonts');
const used = new Set();
katexCss = katexCss.replace(/url\(fonts\/([^)]+?)\)\s*format\(["']?(\w+)["']?\)/g, (m, file, fmt) => {
  if (fmt !== 'woff2') return 'url(about:blank)'; // drop woff/ttf fallbacks
  const full = path.join(fontDir, file);
  if (!fs.existsSync(full)) return m;
  used.add(file);
  const b64 = fs.readFileSync(full).toString('base64');
  return `url(data:font/woff2;base64,${b64}) format("woff2")`;
});
// Remove the now-dead fallback sources.
katexCss = katexCss.replace(/,\s*url\(about:blank\)\s*format\(["']?\w+["']?\)/g, '');

const styles = read(path.join(here, 'src/styles.css'));
const appJs = read(path.join(here, 'src/app.js'));
const widgetJs = read(path.join(here, 'src/widgets.js'));

const template = read(path.join(here, 'src/template.html'));
// Replacer FUNCTIONS, not strings: in a replacement string `$'` means "the
// text after the match" and `$&` means "the match". Script bodies are full of
// `$('#id')`, which would otherwise splice the rest of the page into the file.
const fill = (s, token, value) => s.replace(token, () => value);

let html = template;
html = html.replaceAll('{{TITLE}}', () => pageTitle);
html = fill(html, '{{KATEX_CSS}}', katexCss);
html = fill(html, '{{STYLES}}', styles);
html = fill(html, '{{TOC}}', tocHtml);
html = fill(html, '{{LEDE}}', lede);
html = fill(html, '{{BODY}}', bodyHtml);
html = fill(html, '{{WIDGETS_JS}}', widgetJs);
html = fill(html, '{{APP_JS}}', appJs);

const out = path.join(here, 'index.html');
fs.writeFileSync(out, html);

const kb = (n) => `${(n / 1024).toFixed(0)} KB`;
console.log(`built ${path.relative(root, out)}  ${kb(Buffer.byteLength(html))}`);
console.log(`  ${chapters.length} chapters, ${toc.reduce((n, c) => n + c.children.length, 0)} subsections`);
console.log(`  ${used.size} fonts inlined, ${Object.keys(widgets).length} widgets`);
if (process.exitCode) console.log('  (with math errors — see above)');
