#!/bin/sh
if [ -e ".linttemp" ]; then
  for file in $*; do
    if [ "$file" -nt ".linttemp" ]; then
      newer="$newer $file"
    fi
  done
else
  newer="$*"
fi

# Run in parallel
echo `pwd`
echo "$newer"
if [ -n "$newer" ]; then
  echo $newer | xargs -P 8 -n 8 $PYTHON testing/cpplint.py \
    --root=include --verbose=2 && touch .linttemp
fi
