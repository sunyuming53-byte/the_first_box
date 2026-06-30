#!/bin/bash
# Generate merged compile_commands.json from colcon build output.
# clangd requires this at the workspace root for IntelliSense.
# Run from the workspace root (colcon workspace directory).
set -euo pipefail

entries_file=$(mktemp)
first=true
echo -n "[" > "$entries_file"

for ccdb in build/*/compile_commands.json; do
    if [ -f "$ccdb" ]; then
        if [ "$first" = false ]; then
            echo -n "," >> "$entries_file"
        fi
        # Strip outer [] and append entries
        python3 -c "
import json, sys
with open('$ccdb') as f:
    data = json.load(f)
json.dump(data, sys.stdout)
" | sed 's/^\[//;s/\]$//' >> "$entries_file"
        first=false
        echo "  + $(basename "$(dirname "$ccdb")")"
    fi
done

echo "]" >> "$entries_file"
mv "$entries_file" ./build/compile_commands.json
count=$(python3 -c "import json; print(len(json.load(open('build/compile_commands.json'))))")
echo "build/compile_commands.json: $count entries (merged from all packages)"
