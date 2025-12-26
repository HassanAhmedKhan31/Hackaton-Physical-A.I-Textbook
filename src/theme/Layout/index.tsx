import React from 'react';
import Layout from '@theme-original/Layout';
import { AuthProvider } from '@site/src/components/Auth/AuthContext';
import UserMenu from '@site/src/components/Auth/UserMenu';
import ChatWidget from '@site/src/components/ChatWidget';
import TranslationButton from '@site/src/components/TranslationButton'; // Assuming we rename the JS file to TSX later or import as is

export default function LayoutWrapper(props) {
  return (
    <AuthProvider>
      <Layout {...props}>
        <div style={{ position: 'absolute', top: 10, right: 100, zIndex: 100, display: 'flex', gap: '10px' }}>
            <TranslationButton />
            <UserMenu />
        </div>
        {props.children}
        <ChatWidget />
      </Layout>
    </AuthProvider>
  );
}
