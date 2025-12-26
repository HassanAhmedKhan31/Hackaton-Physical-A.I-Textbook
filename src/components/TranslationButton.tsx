import React, { useState } from 'react';

const TranslationButton = () => {
    // Reuse the logic from UrduTranslationButton.js but in TSX if needed.
    // For now, importing the existing JS component logic is fine, 
    // but here is the TSX version for consistency with the plan.
    
    const [showModal, setShowModal] = useState(false);
    const [translatedText, setTranslatedText] = useState("");
    const [isLoading, setIsLoading] = useState(false);

    const handleTranslate = async () => {
        const text = document.body.innerText; // Simplified scraping
        setIsLoading(true);
        setShowModal(true);
        
        try {
            const res = await fetch('http://localhost:8000/translate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text: text.substring(0, 2000) })
            });
            const data = await res.json();
            setTranslatedText(data.translation);
        } catch (e) {
            setTranslatedText("Error translating page.");
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <>
            <button onClick={handleTranslate} className="navbar__item navbar__link" style={{ cursor: 'pointer', background: 'transparent', border: 'none' }}>
                Translate (Urdu)
            </button>
            {showModal && (
                <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, background: 'rgba(0,0,0,0.8)', zIndex: 9999, display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
                    <div style={{ background: 'white', padding: '20px', width: '80%', height: '80%', overflow: 'auto', textAlign: 'right', direction: 'rtl' }}>
                        <button onClick={() => setShowModal(false)}>Close</button>
                        <h2>Urdu Translation</h2>
                        {isLoading ? <p>Loading...</p> : <p>{translatedText}</p>}
                    </div>
                </div>
            )}
        </>
    );
};

export default TranslationButton;
