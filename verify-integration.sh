#!/usr/bin/env bash
# Verifies the rebuild-forward branch against a proper pinned-base 3-way merge.
# Every path listed is either a deliberate decision or a mistake — review the list.
set -euo pipefail
REF=$(git merge-tree --write-tree --merge-base=7d49349 \
        edouardrolland/release/xprize juanba/main 2>/dev/null | head -1)
echo "reference merge tree: $REF"
git diff --name-only "$REF" HEAD | grep -v 'wildbridge_mavros\|mavlink_proxy' || echo "(no differences)"
