import React from 'react';
import { useAuth } from './Auth/AuthContext';

type Props = {
    software: React.ReactNode;
    hardware: React.ReactNode;
};

const PersonalizedContent = ({ software, hardware }: Props) => {
    const { user } = useAuth();
    // Default to 'student' (show both) if no user or background is 'student'
    const background = user?.background || 'student';

    if (background === 'software') {
        return (
            <div style={{ borderLeft: '4px solid #25c2a0', paddingLeft: '1rem', marginBottom: '1rem' }}>
                <div style={{ fontSize: '0.8rem', color: '#25c2a0', textTransform: 'uppercase', fontWeight: 'bold' }}>
                    Software Track
                </div>
                {software}
            </div>
        );
    }

    if (background === 'hardware') {
        return (
            <div style={{ borderLeft: '4px solid #ff6b6b', paddingLeft: '1rem', marginBottom: '1rem' }}>
                <div style={{ fontSize: '0.8rem', color: '#ff6b6b', textTransform: 'uppercase', fontWeight: 'bold' }}>
                    Hardware Track
                </div>
                {hardware}
            </div>
        );
    }

    // Default: Show both side-by-side (or stacked if mobile, using simple styles)
    return (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '2rem', marginTop: '1rem', marginBottom: '1rem' }}>
            <div style={{ border: '1px solid #e5e7eb', borderRadius: '8px', padding: '1rem' }}>
                 <h4 style={{ color: '#25c2a0', borderBottom: '1px solid #eee', paddingBottom: '0.5rem' }}>Software Perspective</h4>
                 {software}
            </div>
            <div style={{ border: '1px solid #e5e7eb', borderRadius: '8px', padding: '1rem' }}>
                 <h4 style={{ color: '#ff6b6b', borderBottom: '1px solid #eee', paddingBottom: '0.5rem' }}>Hardware Perspective</h4>
                 {hardware}
            </div>
        </div>
    );
};

export default PersonalizedContent;