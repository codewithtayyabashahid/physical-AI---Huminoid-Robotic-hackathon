import React, { useState } from 'react';

export default function UrduToggle({ chapterSlug = '01-physical-ai' }) {
  const [urduContent, setUrduContent] = useState('');
  const [showUrdu, setShowUrdu] = useState(false);
  const [loading, setLoading] = useState(false);

  const toggleUrdu = async () => {
    if (showUrdu) {
      setShowUrdu(false);
      return;
    }

    const API_BASE = process.env.NODE_ENV === 'production' 
      ? 'https://your-backend.com' 
      : 'http://localhost:8000';

    setLoading(true);

    try {
      const response = await fetch(`${API_BASE}/api/translate/urdu?chapter=${chapterSlug}`);
      
      if (!response.ok) {
        throw new Error('Failed to load Urdu content');
      }

      const data = await response.json();
      setUrduContent(data.content || 'Urdu content not available');
      setShowUrdu(true);
    } catch (err) {
      console.error('Failed to load Urdu content:', err);
      alert('Failed to load Urdu translation');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div>
      <button 
        onClick={toggleUrdu}
        disabled={loading}
        style={{
          background: 'rgba(0, 217, 255, 0.2)',
          border: '1px solid rgba(0, 217, 255, 0.4)',
          borderRadius: '8px',
          padding: '0.5rem 1rem',
          color: 'white',
          cursor: 'pointer',
          marginBottom: '1rem'
        }}
      >
        {loading ? '⏳ Loading...' : showUrdu ? '🇬🇧 English' : '🇵🇰 اردو'}
      </button>

      {showUrdu && urduContent && (
        <div style={{
          padding: '1.5rem',
          background: 'rgba(0, 217, 255, 0.05)',
          borderRadius: '8px',
          border: '1px solid rgba(0, 217, 255, 0.2)',
          marginTop: '1rem',
          direction: 'rtl',
          fontFamily: 'Noto Nastaliq Urdu, serif'
        }}>
          {urduContent}
        </div>
      )}
    </div>
  );
}