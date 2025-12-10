// Homepage layout matching dark futuristic design from screenshots
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import RagWidget from '../components/RagWidget';
import PersonalizeButton from '../components/PersonalizeButton';
import UrduToggle from '../components/UrduToggle';

export default function Home() {
  const [showRag, setShowRag] = useState(false);
  const [showPersonalize, setShowPersonalize] = useState(false);

  return (
    <Layout
      title="Physical AI and Humanoid Robotics"
      description="A Comprehensive Guide to Embodied Intelligence">
      
      {/* AI Assistant Button - Top Right */}
      <button 
        className="ai-btn"
        onClick={() => setShowRag(!showRag)}
        aria-label="Toggle AI Assistant"
      >
        🤖 AI Assistant
      </button>

      <div style={{ display: 'flex', minHeight: '100vh' }}>
        {/* Left Sidebar */}
        <aside className="sidebar" style={{ width: '280px', flexShrink: 0 }}>
          <div style={{ marginBottom: '2rem' }}>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '1.5rem' }}>
              <div style={{ 
                width: '40px', 
                height: '40px', 
                background: 'linear-gradient(135deg, #00d9ff, #7c3aed)',
                borderRadius: '8px',
                marginRight: '0.75rem'
              }} />
              <h3 style={{ margin: 0, fontSize: '1.1rem' }}>Physical AI and Humanoid Robotics</h3>
            </div>
          </div>

          <nav>
            <div style={{ marginBottom: '0.5rem' }}>
              <details open>
                <summary style={{ 
                  cursor: 'pointer', 
                  padding: '0.75rem',
                  borderRadius: '8px',
                  background: 'rgba(0, 217, 255, 0.1)',
                  marginBottom: '0.5rem'
                }}>
                  <strong>1</strong> Introduction to Physical AI
                </summary>
                <ul style={{ listStyle: 'none', paddingLeft: '1.5rem', marginTop: '0.5rem' }}>
                  <li style={{ padding: '0.5rem 0' }}>What is Physical AI?</li>
                  <li style={{ padding: '0.5rem 0' }}>The Importance of Embodied AI</li>
                  <li style={{ padding: '0.5rem 0' }}>Book Overview</li>
                </ul>
              </details>
            </div>

            {[
              'Fundamentals of Robotics',
              'Perception Systems',
              'Motion Planning & Control',
              'Machine Learning for Robotics',
              'Humanoid Locomotion',
              'Manipulation & Grasping',
              'Human-Robot Interaction'
            ].map((chapter, idx) => (
              <div key={idx} style={{ 
                padding: '0.75rem',
                borderRadius: '8px',
                marginBottom: '0.25rem',
                cursor: 'pointer',
                transition: 'all 0.2s'
              }}
              onMouseEnter={(e) => e.currentTarget.style.background = 'rgba(0, 217, 255, 0.05)'}
              onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
              >
                <strong>{idx + 2}</strong> {chapter}
              </div>
            ))}
          </nav>
        </aside>

        {/* Main Content Area */}
        <main style={{ flex: 1, padding: '3rem 2rem' }}>
          {/* Hero Section */}
          <div style={{ textAlign: 'center', marginBottom: '4rem' }}>
            <div style={{ 
              display: 'inline-block',
              background: 'rgba(0, 217, 255, 0.1)',
              padding: '0.5rem 1.5rem',
              borderRadius: '24px',
              marginBottom: '2rem',
              border: '1px solid rgba(0, 217, 255, 0.3)'
            }}>
              ⚡ Comprehensive AI & Robotics Guide
            </div>

            <h1 className="hero-title">
              Physical AI and Humanoid Robotics
            </h1>

            <p style={{ 
              fontSize: '1.2rem', 
              color: '#a0aec0', 
              maxWidth: '800px', 
              margin: '0 auto 3rem',
              lineHeight: '1.8'
            }}/>
              A Comprehensive
              <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center', flexWrap: 'wrap' }}>
          <button className="neon-btn" onClick={() => window.location.href = '/panaversity-hackathon/docs/chapters/01-physical-ai'}>
            Start Reading →
          </button>
          <button 
            style={{
              background: 'rgba(255, 255, 255, 0.1)',
              border: '1px solid rgba(255, 255, 255, 0.2)',
              borderRadius: '12px',
              padding: '0.75rem 2rem',
              color: 'white',
              cursor: 'pointer',
              fontWeight: 600
            }}
          >
            📖 About This Book
          </button>
        </div>
      </div>

      {/* Feature Cards */}
      <div style={{ 
        display: 'grid', 
        gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
        gap: '1.5rem',
        marginTop: '3rem'
      }}>
        {[
          { icon: '🧠', title: 'Deep Learning', desc: 'Neural networks, reinforcement learning, and imitation learning for robotics' },
          { icon: '⚙️', title: 'Physical Systems', desc: 'Kinematics, dynamics, and control theory for embodied intelligence' },
          { icon: '🤖', title: 'AI Integration', desc: 'Foundation models, LLMs, and vision-language systems for robots' }
        ].map((feature, idx) => (
          <div key={idx} className="card">
            <div style={{ fontSize: '2.5rem', marginBottom: '1rem' }}>{feature.icon}</div>
            <h3 style={{ fontSize: '1.3rem', marginBottom: '0.75rem' }}>{feature.title}</h3>
            <p style={{ color: '#a0aec0', lineHeight: '1.6' }}>{feature.desc}</p>
          </div>
        ))}
      </div>
    </main>
  </div>

  {/* RAG Widget Modal */}
  {showRag && (
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
      zIndex: 1000,
      padding: '2rem'
    }}>
      <div style={{ position: 'relative', maxWidth: '800px', width: '100%' }}>
        <button 
          onClick={() => setShowRag(false)}
          style={{
            position: 'absolute',
            top: '-40px',
            right: '0',
            background: 'rgba(255, 255, 255, 0.1)',
            border: 'none',
            color: 'white',
            padding: '0.5rem 1rem',
            borderRadius: '8px',
            cursor: 'pointer'
          }}
        >
          ✕ Close
        </button>
        <RagWidget />
      </div>
    </div>
  )}
</Layout>
  );
  }