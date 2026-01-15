import React, { useState } from 'react';
import Viewer3D from './components/Viewer3D';
import LandingPage from './components/LandingPage';

function App() {
  const [view, setView] = useState('landing'); // 'landing' | 'viz'

  return (
    <div className="App w-full h-full">
      {view === 'landing' ? (
        <LandingPage onOperate={() => setView('viz')} />
      ) : (
        <Viewer3D onBack={() => setView('landing')} />
      )}
    </div>
  );
}

export default App;
