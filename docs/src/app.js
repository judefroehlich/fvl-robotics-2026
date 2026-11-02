/* Reading aids: table of contents, scrollspy, filter, collapsing, theme. */
(function () {
  'use strict';

  const $ = (s, r) => (r || document).querySelector(s);
  const $$ = (s, r) => Array.from((r || document).querySelectorAll(s));

  /* ----------------------------------------------------------- theme --- */

  const html = document.documentElement;
  const themeBtn = $('#themeBtn');
  const THEMES = ['auto', 'light', 'dark'];
  const LABEL = { auto: 'Theme: auto', light: 'Theme: light', dark: 'Theme: dark' };

  function applyTheme(t) {
    html.dataset.theme = t;
    themeBtn.textContent = LABEL[t];
    try { localStorage.setItem('docs-theme', t); } catch (e) { /* private mode */ }
    if (window.__docsRedraw) requestAnimationFrame(window.__docsRedraw);
  }
  let saved = 'auto';
  try { saved = localStorage.getItem('docs-theme') || 'auto'; } catch (e) { /* ignore */ }
  applyTheme(THEMES.includes(saved) ? saved : 'auto');
  themeBtn.addEventListener('click', () => {
    applyTheme(THEMES[(THEMES.indexOf(html.dataset.theme) + 1) % THEMES.length]);
  });
  if (window.matchMedia) {
    const mq = window.matchMedia('(prefers-color-scheme: dark)');
    mq.addEventListener('change', () => {
      if (html.dataset.theme === 'auto' && window.__docsRedraw) window.__docsRedraw();
    });
  }

  /* ------------------------------------------------- reading estimate --- */

  // Scoped to <main>: the inline <script> tags live in <body>, so counting the
  // body's text would count the source of this file as prose.
  const words = (($('main') || {}).textContent || '').trim().split(/\s+/).length;
  $('#readingTime').textContent = `${words.toLocaleString()} words · about ${Math.round(words / 220)} min read`;

  /* ------------------------------------------------------- collapsing --- */

  const chapters = $$('.chapter');

  function setCollapsed(chapter, collapsed) {
    chapter.classList.toggle('collapsed', collapsed);
    const btn = $('.toggle', chapter);
    if (btn) {
      btn.setAttribute('aria-expanded', String(!collapsed));
      btn.title = collapsed ? 'Expand section' : 'Collapse section';
    }
  }
  chapters.forEach((c) => {
    const btn = $('.toggle', c);
    if (!btn) return;
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      setCollapsed(c, !c.classList.contains('collapsed'));
    });
  });
  $('#collapseAll').addEventListener('click', () => chapters.forEach((c) => setCollapsed(c, true)));
  $('#expandAll').addEventListener('click', () => chapters.forEach((c) => setCollapsed(c, false)));

  // Following a link into a collapsed chapter has to open it first.
  function revealTarget(id) {
    const node = document.getElementById(id);
    if (!node) return null;
    const chapter = node.closest('.chapter');
    if (chapter) setCollapsed(chapter, false);
    return node;
  }
  document.addEventListener('click', (e) => {
    const a = e.target.closest('a[href^="#"]');
    if (!a) return;
    const id = decodeURIComponent(a.getAttribute('href').slice(1));
    if (!id) return;
    if (revealTarget(id)) {
      // Let the browser scroll, now that the section is open.
      setTimeout(() => {
        const n = document.getElementById(id);
        if (n) n.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }, 0);
      e.preventDefault();
      history.pushState(null, '', '#' + id);
      closeNav();
    }
  });
  if (location.hash) {
    const node = revealTarget(location.hash.slice(1));
    if (node) setTimeout(() => node.scrollIntoView({ block: 'start' }), 30);
  }

  /* ------------------------------------------------- copy code buttons --- */

  $$('pre').forEach((pre) => {
    const btn = document.createElement('button');
    btn.className = 'copy-btn';
    btn.type = 'button';
    btn.textContent = 'copy';
    btn.addEventListener('click', async () => {
      const text = ($('code', pre) || pre).innerText;
      try {
        await navigator.clipboard.writeText(text);
      } catch (e) {
        const ta = document.createElement('textarea');
        ta.value = text;
        document.body.appendChild(ta);
        ta.select();
        try { document.execCommand('copy'); } catch (e2) { /* nothing more to try */ }
        ta.remove();
      }
      btn.textContent = 'copied';
      btn.classList.add('done');
      setTimeout(() => { btn.textContent = 'copy'; btn.classList.remove('done'); }, 1400);
    });
    pre.appendChild(btn);
  });

  /* ------------------------------------------------------------ navi --- */

  const sidebar = $('#sidebar');
  const navToggle = $('#navToggle');
  function closeNav() { sidebar.classList.remove('open'); }
  navToggle.addEventListener('click', () => sidebar.classList.toggle('open'));
  document.addEventListener('click', (e) => {
    if (window.innerWidth > 992) return;
    if (sidebar.contains(e.target) || navToggle.contains(e.target)) return;
    closeNav();
  });

  /* -------------------------------------------------------- scrollspy --- */

  const links = $$('#toc a');
  const byId = new Map(links.map((a) => [a.dataset.target, a]));
  const targets = $$('.chapter, .sub').filter((n) => byId.has(n.id));
  let activeId = null;

  function syncActive() {
    const probe = 110;
    let current = null;
    for (const node of targets) {
      const r = node.getBoundingClientRect();
      if (r.top <= probe) current = node.id;
      else break;
    }
    if (!current && targets.length) current = targets[0].id;
    if (current === activeId) return;
    if (activeId && byId.get(activeId)) byId.get(activeId).classList.remove('active');
    activeId = current;
    const a = byId.get(current);
    if (a) {
      a.classList.add('active');
      const nav = $('.sidebar nav');
      const ar = a.getBoundingClientRect(), nr = nav.getBoundingClientRect();
      if (ar.top < nr.top + 8 || ar.bottom > nr.bottom - 8) {
        nav.scrollTop += ar.top - nr.top - nav.clientHeight / 3;
      }
    }
  }

  const progress = $('#progress');
  const toTop = $('#toTop');
  function onScroll() {
    const h = document.documentElement;
    const max = h.scrollHeight - h.clientHeight;
    progress.style.width = (max > 0 ? (h.scrollTop / max) * 100 : 0) + '%';
    toTop.classList.toggle('show', h.scrollTop > 600);
    syncActive();
  }
  let queued = false;
  window.addEventListener('scroll', () => {
    if (queued) return;
    queued = true;
    requestAnimationFrame(() => { queued = false; onScroll(); });
  }, { passive: true });
  onScroll();

  toTop.addEventListener('click', () => window.scrollTo({ top: 0, behavior: 'smooth' }));

  /* ----------------------------------------------------------- search --- */

  const search = $('#search');
  const count = $('#searchCount');
  const items = links.map((a) => ({
    a,
    li: a.closest('li'),
    text: a.textContent.toLowerCase(),
    original: a.textContent,
  }));

  function runSearch(q) {
    q = q.trim().toLowerCase();
    if (!q) {
      items.forEach((it) => {
        it.li.classList.remove('hidden');
        it.a.textContent = it.original;
      });
      count.textContent = '';
      return;
    }
    let hits = 0;
    // A parent stays visible when any of its children match.
    const matched = new Set();
    items.forEach((it) => {
      if (it.text.includes(q)) {
        matched.add(it.li);
        let p = it.li.parentElement.closest('li');
        while (p) { matched.add(p); p = p.parentElement.closest('li'); }
      }
    });
    items.forEach((it) => {
      const hit = it.text.includes(q);
      it.li.classList.toggle('hidden', !matched.has(it.li));
      if (hit) {
        hits++;
        const i = it.text.indexOf(q);
        const s = it.original;
        it.a.innerHTML =
          escapeHtml(s.slice(0, i)) + '<mark>' + escapeHtml(s.slice(i, i + q.length)) + '</mark>' +
          escapeHtml(s.slice(i + q.length));
      } else {
        it.a.textContent = it.original;
      }
    });
    count.textContent = hits ? hits + '' : 'none';
  }
  function escapeHtml(s) {
    return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  }
  search.addEventListener('input', () => runSearch(search.value));
  search.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { search.value = ''; runSearch(''); search.blur(); }
    if (e.key === 'Enter') {
      const first = items.find((it) => !it.li.classList.contains('hidden') && it.text.includes(search.value.trim().toLowerCase()));
      if (first) first.a.click();
    }
  });

  /* -------------------------------------------------------- shortcuts --- */

  document.addEventListener('keydown', (e) => {
    const typing = /^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement.tagName);
    if (e.key === '/' && !typing) { e.preventDefault(); sidebar.classList.add('open'); search.focus(); return; }
    if (typing || e.metaKey || e.ctrlKey || e.altKey) return;
    if (e.key === '[' || e.key === ']') {
      e.preventDefault();
      const ids = targets.map((t) => t.id);
      let i = ids.indexOf(activeId);
      i = e.key === ']' ? Math.min(i + 1, ids.length - 1) : Math.max(i - 1, 0);
      const node = revealTarget(ids[i]);
      if (node) node.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  });
})();
