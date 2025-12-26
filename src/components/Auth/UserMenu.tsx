import React, { useState } from 'react';
import { authClient } from './authClient';

export default function UserMenu() {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const { data: session } = authClient.useSession();
  
  // Login States
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [background, setBackground] = useState('software'); // Default
  const [isSignup, setIsSignup] = useState(false);

  const handleAuth = async () => {
    try {
      if (isSignup) {
        await authClient.signUp.email({ 
          email, 
          password, 
          name, 
          // @ts-ignore
          background 
        }, {
          onSuccess: () => {
              setIsModalOpen(false);
              window.location.reload(); 
          },
          onError: (ctx) => alert(ctx.error.message)
        });
      } else {
        await authClient.signIn.email({ 
          email, 
          password 
        }, {
          onSuccess: () => {
              setIsModalOpen(false);
              window.location.reload();
          },
          onError: (ctx) => alert(ctx.error.message)
        });
      }
    } catch (err) {
      alert("Authentication Error: " + err.message);
    }
  };

  const handleLogout = async () => {
    await authClient.signOut();
    window.location.reload();
  };

  if (session) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: '10px', marginRight: '1rem' }}>
        <span style={{ fontSize: '0.9rem', fontWeight: 'bold' }}>
           👤 {session.user.name} ({
             // @ts-ignore
             session.user.background || 'student'
           })
        </span>
        <button 
          onClick={handleLogout}
          className="button button--sm button--outline button--danger">
          Logout
        </button>
      </div>
    );
  }

  return (
    <>
      <button 
        onClick={() => setIsModalOpen(true)}
        className="button button--primary button--sm"
        style={{ fontWeight: 'bold' }}>
        Login / Signup
      </button>

      {isModalOpen && (
        <div style={{
          position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
          backgroundColor: 'rgba(0,0,0,0.85)', 
          zIndex: 9999,
          display: 'flex', justifyContent: 'center', alignItems: 'center'
        }}>
          <div style={{
            backgroundColor: '#ffffff', // White Background
            padding: '2rem', 
            borderRadius: '12px', 
            width: '400px',
            boxShadow: '0 4px 20px rgba(0,0,0,0.5)',
            color: '#000000' // Force all text to be black
          }}>
            <h2 style={{ textAlign: 'center', marginBottom: '1.5rem', color: '#000000', fontWeight: '800' }}>
                {isSignup ? 'Create Account' : 'Welcome Back'}
            </h2>
            
            {isSignup && (
              <div style={{ marginBottom: '1rem' }}>
                {/* LABEL FOR NAME */}
                <label style={{display: 'block', fontWeight: 'bold', marginBottom: '5px', color: '#000000'}}>
                    Name
                </label>
                <input 
                  type="text"
                  style={{ width: '100%', padding: '10px', border: '1px solid #ccc', borderRadius: '5px', background: 'white', color: 'black' }}
                  value={name} onChange={e => setName(e.target.value)}
                  placeholder="John Doe"
                />
              </div>
            )}

            <div style={{ marginBottom: '1rem' }}>
              {/* LABEL FOR EMAIL */}
              <label style={{display: 'block', fontWeight: 'bold', marginBottom: '5px', color: '#000000'}}>
                Email Address
              </label>
              <input 
                 type="email"
                 style={{ width: '100%', padding: '10px', border: '1px solid #ccc', borderRadius: '5px', background: 'white', color: 'black' }}
                 value={email} onChange={e => setEmail(e.target.value)} 
                 placeholder="name@example.com"
              />
            </div>

            <div style={{ marginBottom: '1rem' }}>
              {/* LABEL FOR PASSWORD */}
              <label style={{display: 'block', fontWeight: 'bold', marginBottom: '5px', color: '#000000'}}>
                Password
              </label>
              <input 
                 type="password"
                 style={{ width: '100%', padding: '10px', border: '1px solid #ccc', borderRadius: '5px', background: 'white', color: 'black' }}
                 value={password} onChange={e => setPassword(e.target.value)} 
                 placeholder="••••••••"
              />
            </div>

            {isSignup && (
              <div style={{ marginBottom: '1.5rem' }}>
                <label style={{display: 'block', fontWeight: 'bold', marginBottom: '5px', color: '#000000'}}>
                    Your Background
                </label>
                <select 
                  style={{ width: '100%', padding: '10px', border: '1px solid #ccc', borderRadius: '5px', background: 'white', color: 'black' }}
                  value={background} 
                  onChange={e => setBackground(e.target.value)}>
                  <option value="software">Software Engineer</option>
                  <option value="hardware">Hardware Engineer</option>
                </select>
                <small style={{display: 'block', marginTop: '5px', color: '#555'}}>
                    *This customizes the explanations in the book.
                </small>
              </div>
            )}

            <button 
              className="button button--primary button--block button--lg" 
              style={{ width: '100%', marginBottom: '15px' }}
              onClick={handleAuth}>
              {isSignup ? 'Sign Up' : 'Login'}
            </button>
            
            <div style={{ textAlign: 'center', marginBottom: '15px' }}>
                <button 
                  style={{ background: 'none', border: 'none', color: '#25c2a0', cursor: 'pointer', textDecoration: 'underline', fontWeight: 'bold' }}
                  onClick={() => setIsSignup(!isSignup)}>
                  {isSignup ? 'Already have an account? Login' : 'Need an account? Sign Up'}
                </button>
            </div>

            <button 
              className="button button--secondary button--block" 
              style={{ width: '100%' }}
              onClick={() => setIsModalOpen(false)}>
              Close
            </button>
          </div>
        </div>
      )}
    </>
  );
}