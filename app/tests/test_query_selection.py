# Tests for selection query endpoint - ensures empty selection returns 400
import pytest
from fastapi.testclient import TestClient
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from main import app

client = TestClient(app)

def test_query_selection_empty_text():
    """Test that empty selected_text returns 400 error"""
    response = client.post(
        "/api/query/selection",
        json={
            "question": "What is Physical AI?",
            "selected_text": ""
        }
    )
    assert response.status_code == 400
    assert "Selected text cannot be empty" in response.json()["detail"]

def test_query_selection_empty_question():
    """Test that empty question returns 400 error"""
    response = client.post(
        "/api/query/selection",
        json={
            "question": "",
            "selected_text": "Physical AI is embodied intelligence"
        }
    )
    assert response.status_code == 400
    assert "Question cannot be empty" in response.json()["detail"]

def test_health_check():
    """Test health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"
