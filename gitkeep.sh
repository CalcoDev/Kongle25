#!/bin/bash

# Usage information
tool_name=$(basename "$0")
usage() {
  echo "Usage: $tool_name [-a|--add | -r|--remove]"
  echo "  -a, --add     Add .gitkeep to empty directories"
  echo "  -r, --remove  Remove .gitkeep from non-empty directories"
  exit 1
}

# ensure one argument
if [ $# -ne 1 ]; then
  usage
fi

case "$1" in
  -a|--add)
    # add .gitkeep to empty directories
    find . -type d ! -path '*/.*' | while read -r dir; do
      if [ -z "$(ls -A "$dir" 2>/dev/null)" ] && [ ! -f "$dir/.gitkeep" ]; then
        touch "$dir/.gitkeep"
        echo "Added .gitkeep to: $dir"
      fi
    done
    ;;
  -r|--remove)
    # remove .gitkeep from directories that have other content
    find . -type f -name .gitkeep | while read -r file; do
      dir=$(dirname "$file")
      # list all items except .gitkeep
      others=$(ls -A "$dir" | grep -v '^\.gitkeep$')
      if [ -n "$others" ]; then
        rm "$file"
        echo "Removed .gitkeep from: $dir"
      fi
    done
    ;;
  *)
    usage
    ;;
esac