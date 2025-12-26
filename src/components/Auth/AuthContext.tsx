import React, { createContext, useContext, useState, useEffect } from 'react';
import { authClient } from './authClient';

type User = {
    id: string;
    email: string;
    name: string;
    background: 'software' | 'hardware' | 'student';
    hardware_specs?: string;
};

type AuthContextType = {
    user: User | null;
    isLoading: boolean;
    signIn: (email, password) => Promise<void>;
    signUp: (email, password, name, background, hardware_specs) => Promise<void>;
    signOut: () => Promise<void>;
};

const AuthContext = createContext<AuthContextType>(null!);

export const AuthProvider = ({ children }) => {
    const [user, setUser] = useState<User | null>(null);
    const [isLoading, setIsLoading] = useState(true);

    useEffect(() => {
        // Fetch session on load
        async function fetchSession() {
            try {
                const session = await authClient.getSession();
                if (session.data) {
                    setUser(session.data.user as User);
                }
            } catch (e) {
                console.error("Session fetch error", e);
            } finally {
                setIsLoading(false);
            }
        }
        fetchSession();
    }, []);

    const signIn = async (email, password) => {
        setIsLoading(true);
        await authClient.signIn.email({ email, password });
        window.location.reload();
    };

    const signUp = async (email, password, name, background, hardware_specs) => {
        setIsLoading(true);
        await authClient.signUp.email({ 
            email, 
            password, 
            name, 
            background,
            hardware_specs
        });
        window.location.reload();
    };

    const signOut = async () => {
        await authClient.signOut();
        window.location.reload();
    };

    return (
        <AuthContext.Provider value={{ user, isLoading, signIn, signUp, signOut }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => useContext(AuthContext);