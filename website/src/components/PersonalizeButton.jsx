// Personalization button component for customizing user experience
import React, { useState } from 'react';

export default function PersonalizeButton() {
  const [showModal, setShowModal] = useState(false);
  const [preferences, setPreferences] = useState({
    learningLevel: 'intermediate',
    interests: [],
  });

  const handleSave = async () => {
    const API_BASE = process.env.NODE_ENV === 'production' 
      ? 'https://your-backend.com' 
      : 'http://localhost:8000';

    try {
      const response = await fetch(`${API_BASE}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(preferences),
      });

      if (response.ok) {
        alert('Preferences saved!');
        setShowModal(false);
      }
    } catch (err) {
      console.error('Failed to save preferences:', err);
      alert('Failed to save preferences');
    }
  };

  return (
    <>
      <button 
        onClick={() => setShowModal(true)}
        style={{
          background: 'linear-gradient(135deg, #7c3aed, #ff6ec7)',
          border: 'none',
          borderRadius: '12px',
          padding: '0.75rem 1.5rem',
          color: 'white',
          cursor: 'pointer',
          fontWeight: 600,
          boxShadow: '0 4px 12px rgba(124, 58, 237, 0.4)'
        }}
      >
        ⚙️ Personalize
      </button>

      {showModal && (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          background: 'rgba(0, 0, 0, 0.8)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 2000,
          padding: '2rem'
        }}>
          <div className="glass" style={{
            padding: '2rem',
            borderRadius: '16px',
            maxWidth: '500px',
            width: '100%'
          }}>
            <h3 style={{ marginBottom: '1.5rem' }}>Personalize Your Experience</h3>
            
            <div style={{ marginBottom: '1rem' }}>
              <label style={{ display: 'block', marginBottom: '0.5rem' }}>Learning Level:</label>
              <select 
                value={preferences.learningLevel}
                onChange={(e) => setPreferences({...preferences, learningLevel: e.target.value})}
                style={{
                  width: '100%',
                  padding: '0.75rem',
                  borderRadius: '8px',
                  background: 'rgba(0, 0, 0, 0.3)',
                  border: '1px solid rgba(255, 255, 255, 0.2)',
                  color: 'white'
                }}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div style={{ display: 'flex', gap: '1rem', marginTop: '2rem' }}>
              <button onClick={handleSave} className="neon-btn">
                Save
              </button>
              <button 
                onClick={() => setShowModal(false)}
                style={{
                  background: 'rgba(255, 255, 255, 0.1)',
                  border: '1px solid rgba(255, 255, 255, 0.2)',
                  borderRadius: '8px',
                  padding: '0.75rem 1.5rem',
                  color: 'white',
                  cursor: 'pointer'
                }}
              >
                Cancel
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}