import React, { useState, useEffect } from 'react';
import ChatInterface from '../ChatInterface';

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);

  // Only initialize on client side
  useEffect(() => {
    setIsInitialized(true);
  }, []);

  if (!isInitialized) {
    // Render nothing on the server, to avoid hydration issues
    return null;
  }

  return (
    <div className="chat-input-container">
      {isOpen ? (
        <div className="chat-modal">
          <div className="chat-modal-header">
            <h3>AI Assistant</h3>
            <button
              onClick={() => setIsOpen(false)}
              style={{ background: 'none', border: 'none', fontSize: '18px', cursor: 'pointer', color: 'inherit' }}
            >
              ×
            </button>
          </div>
          <div className="chat-modal-body">
            <ChatInterface />
          </div>
        </div>
      ) : (
        <button
          className="chat-toggle-button"
          onClick={() => setIsOpen(true)}
        >
          💬
        </button>
      )}
    </div>
  );
};

export default FloatingChatWidget;