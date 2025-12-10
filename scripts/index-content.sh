#!/bin/bash
# Script to index Docusaurus content into Qdrant vector database

set -e

echo "=== Indexing Content into Qdrant ==="
echo ""

# Check if virtual environment exists
if [ ! -d "app/venv" ]; then
    echo "Error: Python virtual environment not found. Run bootstrap.sh first."
    exit 1
fi

# Activate virtual environment
cd app
source venv/bin/activate || source venv/Scripts/activate

# Run indexing script
echo "Starting content indexing..."
python indexing.py

echo ""
echo "=== Indexing Complete! ==="