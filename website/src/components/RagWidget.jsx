import React, { useState } from 'react';

export default function RagWidget() {
  const [question, setQuestion] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [answer, setAnswer] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const API_BASE = process.env.NODE_ENV === 'production' 
    ? 'https://your-backend.com' 
    : 'http://localhost:8000';

  // Ask question about entire book
  const handleAskBook = async () => {
    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    setLoading(true);
    setError('');
    setAnswer('');

    try {
      const response = await fetch(`${API_BASE}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      setAnswer(data.answer || 'No answer returned');
    } catch (err) {
      setError(`Failed to get answer: ${err.message}`);
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  // Ask question about selected text only
  const handleAskSelection = async () => {
    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    if (!selectedText.trim()) {
      setError('Please select some text first');
      return;
    }

    setLoading(true);
    setError('');
    setAnswer('');

    try {
      const response = await fetch(`${API_BASE}/api/query/selection`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
          question,
          selected_text: selectedText 
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP ${response.status}`);
      }

      const data = await response.json();
      setAnswer(data.answer || 'No answer returned');
    } catch (err) {
      setError(`Failed to get answer: ${err.message}`);
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  // Capture selected text from page
  const captureSelection = () => {
    const selection = window.getSelection();
    const text = selection.toString().trim();
    if (text) {
      setSelectedText(text);
      setError('');
    } else {
      setError('No text selected. Please highlight some text on the page.');
    }
  };

  return (
    <div className="glass" style={{
      padding: '2rem',
      borderRadius: '16px',
      maxWidth: '800px',
      margin: '0 auto'
    }}>
      <h2 style={{ marginBottom: '1.5rem', fontSize: '1.8rem' }}>
        🤖 AI Assistant
      </h2>

      {/* Question Input */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 600 }}>
          Your Question:
        </label>
        <textarea
          value={question}
          onChange={(e) => setQuestion(e.target.value)}
          placeholder="Ask anything about Physical AI and Robotics..."
          style={{
            width: '100%',
            minHeight: '100px',
            padding: '1rem',
            borderRadius: '8px',
            background: 'rgba(0, 0, 0, 0.3)',
            border: '1px solid rgba(255, 255, 255, 0.2)',
            color: 'white',
            fontSize: '1rem',
            resize: 'vertical'
          }}
        />
      </div>

      {/* Selected Text Display */}
      {selectedText && (
        <div style={{
          marginBottom: '1.5rem',
          padding: '1rem',
          background: 'rgba(0, 217, 255, 0.1)',
          borderRadius: '8px',
          border: '1px solid rgba(0, 217, 255, 0.3)'
        }}>
          <strong>Selected Text:</strong>
          <p style={{ marginTop: '0.5rem', fontSize: '0.9rem', color: '#a0aec0' }}>
            {selectedText.substring(0, 200)}
            {selectedText.length > 200 ? '...' : ''}
          </p>
        </div>
      )}

      {/* Action Buttons */}
      <div style={{ display: 'flex', gap: '1rem', marginBottom: '1.5rem', flexWrap: 'wrap' }}>
        <button 
          onClick={handleAskBook}
          disabled={loading}
          className="neon-btn"
          style={{ opacity: loading ? 0.6 : 1 }}
        >
          📚 Ask Book
        </button>
        <button 
          onClick={captureSelection}
          style={{
            background: 'rgba(255, 255, 255, 0.1)',
            border: '1px solid rgba(255, 255, 255, 0.3)',
            borderRadius: '12px',
            padding: '0.75rem 1.5rem',
            color: 'white',
            cursor: 'pointer',
            fontWeight: 600
          }}
        >
          ✂️ Capture Selection
        </button>
        <button 
          onClick={handleAskSelection}
          disabled={loading || !selectedText}
          style={{
            background: selectedText ? 'linear-gradient(135deg, #7c3aed, #ff6ec7)' : 'rgba(100, 100, 100, 0.3)',
            border: 'none',
            borderRadius: '12px',
            padding: '0.75rem 1.5rem',
            color: 'white',
            cursor: selectedText ? 'pointer' : 'not-allowed',
            fontWeight: 600,
            opacity: loading ? 0.6 : 1
          }}
        >
          🎯 Ask Selection
        </button>
      </div>

      {/* Loading State */}
      {loading && (
        <div style={{ textAlign: 'center', padding: '2rem', color: '#00d9ff' }}>
          <div>⏳ Thinking...</div>
        </div>
      )}

      {/* Error Display */}
      {error && (
        <div style={{
          padding: '1rem',
          background: 'rgba(255, 0, 0, 0.1)',
          border: '1px solid rgba(255, 0, 0, 0.3)',
          borderRadius: '8px',
          color: '#ff6b6b',
          marginBottom: '1rem'
        }}>
          ⚠️ {error}
        </div>
      )}

      {/* Answer Display */}
      {answer && (
        <div style={{
          padding: '1.5rem',
          background: 'rgba(0, 217, 255, 0.05)',
          borderRadius: '8px',
          border: '1px solid rgba(0, 217, 255, 0.2)'
        }}>
          <strong style={{ color: '#00d9ff' }}>Answer:</strong>
          <div style={{ marginTop: '1rem', lineHeight: '1.8' }}>
            {answer}
          </div>
        </div>
      )}
    </div>
  );
}