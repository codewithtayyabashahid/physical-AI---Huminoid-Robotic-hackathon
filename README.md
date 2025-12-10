# Panaversity Hackathon: Physical AI & Humanoid Robotics RAG Platform

A comprehensive educational platform combining Docusaurus documentation with RAG-powered AI assistance for learning Physical AI and Humanoid Robotics.

## 🚀 Features

- **Interactive Docusaurus Book**: Comprehensive guide to Physical AI and Humanoid Robotics
- **RAG-Powered Q&A**: Ask questions about the entire book or selected text
- **Urdu Translation**: Toggle between English and Urdu content
- **Personalization Engine**: Customize learning experience based on user preferences
- **Dark Futuristic UI**: Neon-styled, glassmorphic interface with gradient effects
- **Authentication**: Secure signup/login system
- **Vector Search**: Qdrant-powered semantic search over book content
- **Selection-Based Queries**: Ask questions about highlighted text only

## 📋 Prerequisites

- Python 3.9+
- Node.js 16+ and pnpm
- Docker and Docker Compose
- Git
- OpenAI API Key
- Qdrant Cloud account (or use local Qdrant via Docker)

## 🛠️ Quick Start

### 1. Clone and Setup
```bash
git clone <your-repo-url>
cd panaversity-hackathon
copy .env.example .env
```

Edit `.env` and add your real API keys.

### 2. Bootstrap Environment
```bash
# On Windows with Git Bash or WSL
bash scripts/bootstrap.sh

# Or manually:
cd app
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
cd ..\website
pnpm install
cd ..
```

### 3. Start Services
```bash
docker-compose up --build -d
```

### 4. Index Content
```bash
cd app
venv\Scripts\activate
python indexing.py
```

### 5. Run Application

Terminal 1 (Backend):
```bash
cd app
venv\Scripts\activate
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Terminal 2 (Frontend):
```bash
cd website
pnpm start
```

Visit: http://localhost:3000

## 🧪 Testing
```bash
cd app
venv\Scripts\activate
pytest tests/
```

## 🚢 Deployment
```bash
set GIT_USER=your-github-username
bash deploy.sh
```

## 📁 Project Structure

- `/app` - FastAPI backend with RAG endpoints
- `/website` - Docusaurus frontend with React components
- `/scripts` - Automation scripts for setup and deployment
- `/docs` - Markdown content for the book

## 🔐 Environment Variables

See `.env.example` for all required configuration.

## 📝 License

MIT License - see LICENSE file for details.