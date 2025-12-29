import React from 'react';
import { useAuth } from './Auth/AuthContext';

type Props = {
    software: React.ReactNode;
    hardware: React.ReactNode;
};

const PersonalizedContent = ({ software, hardware }: Props) => {
    // 🛡️ CRASH FIX: Safe Check
    const auth = useAuth();
    const user = auth ? auth.user : null; // If auth is broken, user is null.

    const background = user?.background || 'student';

    if (background === 'software') {
        return (
            <div style={{ borderLeft: '4px solid #25c2a0', paddingLeft: '1rem', marginBottom: '1rem' }}>
                <div style={{ fontSize: '0.8rem', color: '#25c2a0', textTransform: 'uppercase', fontWeight: 'bold' }}>Software Track</div>
                {software}
            </div>
        );
    }

    if (background === 'hardware') {
        return (
            <div style={{ borderLeft: '4px solid #ff6b6b', paddingLeft: '1rem', marginBottom: '1rem' }}>
                <div style={{ fontSize: '0.8rem', color: '#ff6b6b', textTransform: 'uppercase', fontWeight: 'bold' }}>Hardware Track</div>
                {hardware}
            </div>
        );
    }

    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '2rem' }}>
            <div style={{ border: '1px solid #ddd', padding: '1rem', borderRadius: '8px' }}>
                 <h4 style={{ color: '#25c2a0' }}>Software Perspective</h4>
                 {software}
            </div>
            <div style={{ border: '1px solid #ddd', padding: '1rem', borderRadius: '8px' }}>
                 <h4 style={{ color: '#ff6b6b' }}>Hardware Perspective</h4>
                 {hardware}
            </div>
        </div>
    );
};

export default PersonalizedContent;