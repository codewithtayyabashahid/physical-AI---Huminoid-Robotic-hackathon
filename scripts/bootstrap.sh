#!/bin/bash
# Bootstrap script - sets up Python venv and installs all dependencies non-interactively

set -e

echo "=== Panaversity Hackathon Bootstrap ==="
echo ""

# Check for Python
if ! command -v python &> /dev/null && ! command -v python3 &> /dev/null; then
    echo "Error: Python is not installed. Please install Python 3.9+ and try again."
    exit 1
fi

PYTHON_CMD=$(command -v python3 || command -v python)

# Create Python virtual environment
echo "Creating Python virtual environment..."
cd app
$PYTHON_CMD -m venv venv

# Activate venv (Linux/Mac/Git Bash)
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
# Windows (Git Bash emulation)
elif [ -f "venv/Scripts/activate" ]; then
    source venv/Scripts/activate
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

cd ..

# Install Node dependencies
echo "Installing website dependencies..."
cd website

if ! command -v pnpm &> /dev/null; then
    echo "Installing pnpm..."
    npm install -g pnpm
fi

pnpm install

cd ..

echo ""
echo "=== Bootstrap Complete! ==="
echo "Next steps:"
echo "1. Copy .env.example to .env and add your API keys"
echo "2. Run 'docker-compose up -d' to start services"
echo "3. Run 'python app/indexing.py' to index content"
echo "4. Start backend: 'cd app && uvicorn main:app --reload'"
echo "5. Start frontend: 'cd website && pnpm start'"