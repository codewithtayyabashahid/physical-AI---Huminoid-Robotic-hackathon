#!/bin/bash
# Script to record a demo video of the application using asciinema or similar tools

echo "=== Demo Recording Script ==="
echo ""
echo "This script helps you record a demo of the application."
echo "Prerequisites: Install 'asciinema' for terminal recording"
echo ""

if ! command -v asciinema &> /dev/null; then
    echo "asciinema not found. Install it with: pip install asciinema"
    exit 1
fi

echo "Starting recording in 3 seconds..."
sleep 3

asciinema rec demo-$(date +%Y%m%d-%H%M%S).cast