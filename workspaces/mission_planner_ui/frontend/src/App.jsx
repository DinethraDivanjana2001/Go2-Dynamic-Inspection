import React, { useState, useEffect } from 'react';
import Viewer3D from './components/Viewer3D';
import LandingPage from './components/LandingPage';
import Login from './components/Login';

function App() {
  const [view, setView] = useState('landing');
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      setIsAuthenticated(true);
    }
  }, []);

  const handleLoginSuccess = () => {
    setIsAuthenticated(true);
    setView('landing');
  };

  const handleLogout = () => {
    localStorage.removeItem('auth_token');
    setIsAuthenticated(false);
    setView('landing');
  };

  // If not authenticated, force login view
  if (!isAuthenticated) {
    return <Login onLoginSuccess={handleLoginSuccess} />;
  }

  return (
    <div className="App w-full h-full relative">
      {/* Global Logout Button */}
      <button
        onClick={handleLogout}
        className="absolute top-4 left-4 z-[9999] px-3 py-1 bg-red-500/80 hover:bg-red-500 text-white text-xs font-bold rounded shadow-lg backdrop-blur"
      >
        LOGOUT
      </button>

      {view === 'landing' ? (
        <LandingPage onOperate={() => setView('viz')} />
      ) : (
        <Viewer3D onBack={() => setView('landing')} />
      )}
    </div>
  );
}

export default App;
