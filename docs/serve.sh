#!/usr/bin/env sh
# Serve the docs at http://localhost:8000 (pass a port to override).
#
#   ./docs/serve.sh          -> http://localhost:8000
#   ./docs/serve.sh 9000     -> http://localhost:9000
set -e
cd "$(dirname "$0")"

# Rebuild if the markdown or the sources are newer than the page.
if [ ! -f index.html ] \
   || [ ../CODE_EXPLAINED.md -nt index.html ] \
   || [ src/widgets.js -nt index.html ] \
   || [ src/app.js -nt index.html ] \
   || [ src/styles.css -nt index.html ] \
   || [ src/template.html -nt index.html ]; then
  if [ -d .build/node_modules ]; then
    echo "rebuilding..."
    node build.mjs
  else
    echo "note: docs/.build/node_modules is missing, serving the existing page."
    echo "      run 'cd docs/.build && npm install' to rebuild from markdown."
  fi
fi

PORT="${1:-8000}"
echo "serving on http://localhost:$PORT/   (ctrl-c to stop)"
exec python3 -m http.server "$PORT" --bind 127.0.0.1
