import React from 'react';
import Layout from '@theme-original/Layout';
import TranslationButton from '@site/src/components/TranslationButton';
// ✅ Import the Auth Provider
import { AuthProvider } from '@site/src/components/Auth/AuthContext';

export default function LayoutWrapper(props) {
  return (
    // ✅ ENABLED: This wraps the app so "useAuth" works
    <AuthProvider>
      <Layout {...props}>
        
        {/* Translation Button */}
        <div style={{ position: 'absolute', top: 10, right: 100, zIndex: 100 }}>
             <TranslationButton />
        </div>

        {props.children}
        
      </Layout>
    </AuthProvider>
  );
}