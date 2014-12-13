#!/bin/sh

echo "Ready...OK."
make || { echo oops!; exit 1; }
echo "Done...GO.\n>>> - - - "
./exe