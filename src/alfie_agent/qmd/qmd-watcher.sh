#!/usr/bin/env bash
# qmd-watcher — keep the QMD index fresh when vault notes change.
#
# Ported from lloyd's agent-services/scripts/qmd-watcher.sh. Watches the vault
# for markdown changes and, after a short debounce, re-runs `qmd update` (FTS)
# then `qmd embed` (vectors). Run as a systemd service (alfie-qmd-watcher).
set -euo pipefail

QMD="${QMD_BIN:-/home/alfie/.bun/bin/qmd}"
VAULT="${QMD_VAULT:-/home/alfie/obsidian}"
DEBOUNCE_SEC="${QMD_DEBOUNCE_SEC:-2}"

echo "qmd-watcher: watching $VAULT for .md changes (debounce ${DEBOUNCE_SEC}s)"

inotifywait -m -r -e close_write,create,delete,moved_to,moved_from \
  --include '\.md$' "$VAULT" |
while read -r _; do
  # Debounce: drain further events for DEBOUNCE_SEC before reindexing.
  while read -r -t "$DEBOUNCE_SEC" _; do :; done
  echo "qmd-watcher: change detected, reindexing..."
  if "$QMD" update; then
    "$QMD" embed || echo "qmd-watcher: WARNING embed failed"
  else
    echo "qmd-watcher: WARNING update failed"
  fi
done
