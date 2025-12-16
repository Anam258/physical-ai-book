import React from 'react';
import Layout from '@theme-original/Layout';
import ChatBot from '../components/ChatBot';
import UrduTranslation from '../components/UrduTranslation';
import ProgressBar from '../components/ProgressBar';
import { AuthProvider } from '../components/Auth/AuthProvider';

export default function LayoutWrapper(props) {
  return (
    <AuthProvider>
      <Layout {...props}>
        <ProgressBar />
        {props.children}
        <ChatBot />
        <UrduTranslation />
      </Layout>
    </AuthProvider>
  );
}