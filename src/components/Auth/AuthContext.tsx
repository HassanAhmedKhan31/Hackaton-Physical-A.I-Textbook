import React, { createContext, useContext, useState, useEffect } from 'react';

// Define the shape of our Auth Context
interface AuthContextType {
    user: any;
    signIn: () => void;
    signOut: () => void;
}

const AuthContext = createContext<AuthContextType | null>(null);

export const AuthProvider = ({ children }: { children: React.ReactNode }) => {
    // For now, we are in "Guest Mode" to stop the crashes
    const [user, setUser] = useState<{ background: string } | null>(null);

    const signIn = () => {
        console.log("Sign in clicked (Demo Mode)");
        setUser({ background: 'student' });
    };

    const signOut = () => {
        setUser(null);
    };

    return (
        <AuthContext.Provider value={{ user, signIn, signOut }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => useContext(AuthContext);