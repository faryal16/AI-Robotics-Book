import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidgetInjector from '@site/src/components/ChatWidgetInjector';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatWidgetInjector />
    </>
  );
}
