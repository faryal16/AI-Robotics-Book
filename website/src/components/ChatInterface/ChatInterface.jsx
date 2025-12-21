import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatInterface.css';

const ChatInterface = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [mode, setMode] = useState('book'); // 'book' or 'selected_text'
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  const { siteConfig } = useDocusaurusContext();

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
      setMode('selected_text');
      // Show a visual confirmation to the user
      alert(`Selected text captured: "${selectedText.substring(0, 50)}${selectedText.length > 50 ? '...' : ''}"`);
    } else {
      alert('Please select some text on the page first.');
    }
  };

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Function to send message to backend
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { text: inputValue, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Capture current selected text if in selected_text mode
      const currentSelectedText = mode === 'selected_text' ? window.getSelection().toString().trim() || selectedText : '';

      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_query: inputValue,
          mode: mode,
          selected_text: currentSelectedText || null,
        }),
      });

      const data = await response.json();

      if (response.ok) {
        const botMessage = {
          text: data.answer,
          sender: 'bot',
          sources: data.sources || [],
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          text: 'Sorry, I encountered an error processing your request.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = {
        text: 'Sorry, I\'m having trouble connecting to the server.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  // Handle Enter key press (without Shift for new line)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Clear chat
  const clearChat = () => {
    setMessages([]);
  };

  return (
    <div className="chat-container" style={{
      height: '100%',
      display: 'flex',
      flexDirection: 'column',
      fontFamily: 'system-ui, -apple-system, sans-serif',
    }}>
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: '15px',
      }}>
        <h3 style={{ margin: 0, fontSize: '1em' }}>🤖 AI Assistant</h3>
        <div style={{ display: 'flex', gap: '8px' }}>
          <select
            value={mode}
            onChange={(e) => setMode(e.target.value)}
            style={{
              padding: '4px 8px',
              fontSize: '12px',
              border: '1px solid #ccc',
              borderRadius: '4px',
              backgroundColor: 'white',
            }}
          >
            <option value="book">Book Q&A</option>
            <option value="selected_text">Context Q&A</option>
          </select>
          <button
            onClick={clearChat}
            style={{
              padding: '4px 8px',
              fontSize: '12px',
              backgroundColor: '#f0f0f0',
              border: '1px solid #ccc',
              borderRadius: '4px',
              cursor: 'pointer',
            }}
          >
            Clear
          </button>
        </div>
      </div>

      <div className="chat-messages" style={{
        flex: 1,
        marginBottom: '15px',
        overflowY: 'auto',
      }}>
        {messages.length === 0 ? (
          <div style={{
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            height: '100%',
            color: '#666',
            fontStyle: 'italic',
          }}>
            <div style={{ textAlign: 'center' }}>
              <p>How can I help you today?</p>
              <p style={{ fontSize: '12px', marginTop: '8px' }}>Ask questions about Physical AI & Humanoid Robotics</p>
            </div>
          </div>
        ) : (
          <div>
            {messages.map((message, index) => (
              <div
                key={index}
                className={`chat-message ${message.sender === 'user' ? 'chat-message-user' : ''}`}
              >
                <div
                  className={`message-bubble ${message.sender === 'user' ? 'message-bubble-user' : 'message-bubble-bot'}`}
                >
                  {message.text}
                  {message.sources && message.sources.length > 0 && (
                    <div className="sources">
                      <strong>Sources:</strong>
                      <ul>
                        {message.sources.map((source, idx) => (
                          <li key={idx}>
                            {source.chapter}
                            {source.section && ` - ${source.section}`}
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div style={{ textAlign: 'left', marginBottom: '15px' }}>
                <div
                  style={{
                    display: 'inline-block',
                    padding: '10px 15px',
                    borderRadius: '18px',
                    backgroundColor: '#e9ecef',
                    color: 'black',
                  }}
                >
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      <div className="chat-input-area">
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book..."
          rows={2}
          className="chat-input"
          disabled={isLoading}
        />
        <button
          onClick={sendMessage}
          disabled={!inputValue.trim() || isLoading}
          className="chat-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div style={{ fontSize: '11px', color: '#888', textAlign: 'center', marginTop: '8px' }}>
        {mode === 'book' ? (
          <p>Mode: Book Q&A</p>
        ) : (
          <p>Mode: Context Q&A</p>
        )}
      </div>
    </div>
  );
};

export default ChatInterface;