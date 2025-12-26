import React, { useState, useRef, useEffect } from 'react';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I am your AI Professor. Ask me anything about the book.", sender: 'bot' }
  ]);
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState("");
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Monitor text selection
  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection().toString();
      if (text && text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const handleSend = async () => {
    if (!input.trim()) return;

    const userMessage = { id: Date.now(), text: input, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInput("");
    setIsLoading(true);

    // Capture context at the moment of sending
    const currentSelection = selectedText;
    // Clear selection state after sending
    setSelectedText("");

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.text,
          selected_text: currentSelection || null
        }),
      });

      const data = await response.json();
      
      const botMessage = { 
        id: Date.now() + 1, 
        text: data.response || "Sorry, I encountered an error.", 
        sender: 'bot' 
      };
      
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error("Chat Error:", error);
      setMessages(prev => [...prev, { 
        id: Date.now() + 1, 
        text: "Error: Could not connect to the Professor.", 
        sender: 'bot' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-widget-container">
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <span>AI Professor 🤖</span>
            <button 
              onClick={() => setIsOpen(false)} 
              style={{background: 'none', border: 'none', color: 'white', cursor: 'pointer', fontSize: '16px'}}
            >
              ✕
            </button>
          </div>
          
          <div className="chat-messages">
            {messages.map((msg) => (
              <div key={msg.id} className={`message ${msg.sender}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && <div className="message bot">Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="selection-indicator">
              Selected context: "{selectedText.substring(0, 30)}..."
            </div>
          )}

          <div className="chat-input-area">
            <input
              type="text"
              className="chat-input"
              placeholder="Ask a question..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleSend()}
              disabled={isLoading}
            />
            <button 
              className="chat-send-btn" 
              onClick={handleSend}
              disabled={isLoading}
            >
              ➤
            </button>
          </div>
        </div>
      )}

      <button className="chat-widget-button" onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? '💬' : '🤖'}
      </button>
    </div>
  );
};

export default ChatWidget;
