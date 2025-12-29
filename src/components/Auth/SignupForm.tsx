import React, { useState } from 'react';
import { useAuth } from './authContext';

const SignupForm = () => {
    const { signUp } = useAuth();
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [name, setName] = useState('');
    const [background, setBackground] = useState('student');
    const [hardwareSpecs, setHardwareSpecs] = useState('');

    const handleSubmit = async (e) => {
        e.preventDefault();
        await signUp(email, password, name, background, hardwareSpecs);
    };

    return (
        <div style={{ padding: '20px', border: '1px solid #ccc', borderRadius: '8px', maxWidth: '400px', margin: '20px auto' }}>
            <h3>Join the Class</h3>
            <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
                <div>
                    <label>Name: </label>
                    <input style={{width: '100%'}} value={name} onChange={e => setName(e.target.value)} required />
                </div>
                <div>
                    <label>Email: </label>
                    <input style={{width: '100%'}} type="email" value={email} onChange={e => setEmail(e.target.value)} required />
                </div>
                <div>
                    <label>Password: </label>
                    <input style={{width: '100%'}} type="password" value={password} onChange={e => setPassword(e.target.value)} required />
                </div>
                <div>
                    <label>What is your background? </label>
                    <select style={{width: '100%'}} value={background} onChange={e => setBackground(e.target.value)}>
                        <option value="student">Student (General)</option>
                        <option value="software">Software Engineer (Coding Focus)</option>
                        <option value="hardware">Hardware Engineer (Physics Focus)</option>
                    </select>
                </div>
                <div>
                    <label>Do you have an NVIDIA GPU? (Optional) </label>
                    <input 
                        style={{width: '100%'}} 
                        placeholder="e.g., RTX 3060, or 'No'" 
                        value={hardwareSpecs} 
                        onChange={e => setHardwareSpecs(e.target.value)} 
                    />
                </div>
                <button type="submit" style={{ padding: '10px', background: '#25c2a0', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}>Enroll</button>
            </form>
        </div>
    );
};

export default SignupForm;