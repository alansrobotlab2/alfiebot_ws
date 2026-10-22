#!/usr/bin/env bash
# qmd-watcher — keep the QMD index fresh when vault notes change.
#
# Ported from lloyd's agent-services/scripts/qmd-watcher.sh. Watches the vault
# for markdown changes and, after a short debounce, re-runs `qmd update` (FTS)
# then `qmd embed` (vectors). Run as a systemd service (alfie-qmd-watcher).
#
# Uses inotifywait (apt: inotify-tools) when installed; otherwise falls back to
# polling the vault for newer .md files every QMD_POLL_SEC seconds.
set -euo pipefail

QMD="${QMD_BIN:-/home/alfie/qmd/bin/qmd}"
VAULT="${QMD_VAULT:-/home/alfie/obsidian}"
DEBOUNCE_SEC="${QMD_DEBOUNCE_SEC:-2}"
POLL_SEC="${QMD_POLL_SEC:-10}"

reindex() {
  echo "qmd-watcher: change detected, reindexing..."
  if "$QMD" update; then
    "$QMD" embed || echo "qmd-watcher: WARNING embed failed"
  else
    echo "qmd-watcher: WARNING update failed"
  fi
}

if command -v inotifywait >/dev/null 2>&1; then
  echo "qmd-watcher: watching $VAULT for .md changes (debounce ${DEBOUNCE_SEC}s)"
  inotifywait -m -r -e close_write,create,delete,moved_to,moved_from \
    --include '\.md$' "$VAULT" |
  while read -r _; do
    # Debounce: drain further events for DEBOUNCE_SEC before reindexing.
    while read -r -t "$DEBOUNCE_SEC" _; do :; done
    reindex
  done
else
  echo "qmd-watcher: inotifywait not found; polling $VAULT every ${POLL_SEC}s"
  stamp="$(mktemp)"
  count() { find "$VAULT" -name '*.md' | wc -l; }
  last_count="$(count)"
  while sleep "$POLL_SEC"; do
    # New/changed notes are newer than the stamp; deletions change the count.
    changed="$(find "$VAULT" -name '*.md' -newer "$stamp" -print -quit)"
    now_count="$(count)"
    if [[ -n "$changed" || "$now_count" != "$last_count" ]]; then
      touch "$stamp"
      last_count="$now_count"
      reindex
    fi
  done
fi
