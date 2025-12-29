import React, { useState, useRef, useEffect } from 'react';

export default function ChatWidget() {
  // 1. State to track if the window is open or closed
  const [isOpen, setIsOpen] = useState(false);
  
  // 2. State for messages and input
  const [messages, setMessages] = useState([
    { role: 'system', text: 'Hello! I am your AI Professor. Ask me anything about the textbook.' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom of chat
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, isOpen]);

  // 3. Function to send message to Python Backend
  const handleSend = async () => {
    if (!inputValue.trim()) return;

    const userMessage = inputValue;
    setMessages(prev => [...prev, { role: 'user', text: userMessage }]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Talking to your Python Brain (Port 8000)
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: userMessage }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'bot', text: data.reply }]);
    } catch (error) {
      setMessages(prev => [...prev, { role: 'bot', text: "❌ Error: Could not connect to the Brain. Is 'app.py' running?" }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999 }}>
      
      {/* --- THE CHAT WINDOW --- */}
      {isOpen && (
        <div style={{
          width: '350px',
          height: '500px',
          backgroundColor: 'white',
          borderRadius: '12px',
          boxShadow: '0 4px 20px rgba(0,0,0,0.2)',
          display: 'flex',
          flexDirection: 'column',
          marginBottom: '15px',
          overflow: 'hidden',
          border: '1px solid #ddd'
        }}>
          {/* Header */}
          <div style={{ backgroundColor: '#25c2a0', padding: '15px', color: 'white', fontWeight: 'bold', display: 'flex', justifyContent: 'space-between' }}>
            <span>🤖 AI Professor</span>
            <button onClick={() => setIsOpen(false)} style={{ background: 'none', border: 'none', color: 'white', cursor: 'pointer', fontSize: '1.2rem' }}>✖</button>
          </div>

          {/* Messages Area */}
          <div style={{ flex: 1, padding: '15px', overflowY: 'auto', backgroundColor: '#f9f9f9', color: 'black' }}>
            {messages.map((msg, idx) => (
              <div key={idx} style={{
                marginBottom: '10px',
                textAlign: msg.role === 'user' ? 'right' : 'left'
              }}>
                <span style={{
                  display: 'inline-block',
                  padding: '10px',
                  borderRadius: '10px',
                  backgroundColor: msg.role === 'user' ? '#007bff' : '#e0e0e0',
                  color: msg.role === 'user' ? 'white' : 'black',
                  maxWidth: '80%'
                }}>
                  {msg.text}
                </span>
              </div>
            ))}
            {isLoading && <div style={{ color: '#888', fontStyle: 'italic' }}>Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div style={{ padding: '10px', borderTop: '1px solid #ddd', display: 'flex', gap: '5px' }}>
            <input 
              type="text" 
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleSend()}
              placeholder="Ask a question..."
              style={{ flex: 1, padding: '10px', borderRadius: '5px', border: '1px solid #ccc', color: 'black' }}
            />
            <button 
              onClick={handleSend}
              style={{ padding: '10px 15px', backgroundColor: '#25c2a0', color: 'white', border: 'none', borderRadius: '5px', cursor: 'pointer' }}>
              Send
            </button>
          </div>
        </div>
      )}

      {/* --- THE ROBOT BUTTON --- */}
      <button 
        onClick={() => setIsOpen(!isOpen)} // ✅ This toggles the window
        style={{
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#25c2a0',
          border: 'none',
          boxShadow: '0 4px 10px rgba(0,0,0,0.3)',
          cursor: 'pointer',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          fontSize: '2rem'
        }}
      >
        {isOpen ? '❌' : '🤖'}
      </button>
    </div>
  );
}