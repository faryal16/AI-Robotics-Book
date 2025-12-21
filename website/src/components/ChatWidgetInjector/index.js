import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ChatWidgetInjector = () => {
  return (
    <BrowserOnly>
      {() => {
        const FloatingChatWidget = require('../FloatingChatWidget').default;
        return <FloatingChatWidget />;
      }}
    </BrowserOnly>
  );
};

export default ChatWidgetInjector;