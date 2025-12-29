import React from 'react';
// Import the ChatWidget
import ChatWidget from '../components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      {/* ✅ UNCOMMENTED: The Chatbot is back! */}
      <ChatWidget />
    </>
  );
}