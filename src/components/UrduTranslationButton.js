import React, { useState } from 'react';

const UrduTranslationButton = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [showModal, setShowModal] = useState(false);
  const [translatedText, setTranslatedText] = useState("");

  const handleTranslate = async () => {
    // In a real Docusaurus site, we would target the specific markdown container
    // For this prototype, we grab the main content text
    const pageContent = document.querySelector('article') 
      ? document.querySelector('article').innerText 
      : document.body.innerText;

    if (!pageContent) return;

    setIsTranslating(true);
    setShowModal(true);

    try {
      const response = await fetch('http://localhost:8000/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: pageContent.substring(0, 3000) // Limit chars for demo/cost
        }),
      });

      const data = await response.json();
      setTranslatedText(data.translation);
    } catch (error) {
      console.error("Translation failed:", error);
      setTranslatedText("Error: Could not retrieve translation.");
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <>
      <button 
        onClick={handleTranslate}
        style={{
          backgroundColor: '#ff9800', 
          color: 'white', 
          border: 'none', 
          padding: '8px 12px', 
          borderRadius: '4px',
          cursor: 'pointer',
          fontWeight: 'bold',
          marginLeft: '10px'
        }}
      >
        🇺pk Translate to Urdu
      </button>

      {showModal && (
        <div style={{
          position: 'fixed',
          top: 0, left: 0, right: 0, bottom: 0,
          backgroundColor: 'rgba(0,0,0,0.7)',
          zIndex: 10000,
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center'
        }}>
          <div style={{
            backgroundColor: 'white',
            width: '80%',
            height: '80%',
            padding: '20px',
            borderRadius: '8px',
            overflowY: 'auto',
            position: 'relative',
            direction: 'rtl', // Right-to-left for Urdu
            textAlign: 'right',
            fontFamily: '"Noto Nastaliq Urdu", serif'
          }}>
            <button 
              onClick={() => setShowModal(false)}
              style={{
                position: 'absolute',
                top: '10px',
                left: '10px', // Left because RTL
                border: 'none',
                background: 'transparent',
                fontSize: '20px',
                cursor: 'pointer'
              }}
            >
              ✕ Close
            </button>
            
            <h2 style={{textAlign: 'center', marginBottom: '20px'}}>Urdu Translation</h2>
            
            {isTranslating ? (
              <div style={{textAlign: 'center', marginTop: '50px'}}>Translating page... (this may take a moment)</div>
            ) : (
              <div style={{whiteSpace: 'pre-wrap', lineHeight: '2.0'}}>
                {translatedText}
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
};

export default UrduTranslationButton;
