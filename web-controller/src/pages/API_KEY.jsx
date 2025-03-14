import React from 'react';
import { createRoot } from 'react-dom/client';
import { APIProvider, Map } from '@vis.gl/react-google-maps';

const API_KEY = 'AIzaSyB0yZWHNbvrSKQjMkTGUahIKU7RT0EuvEM';

const App = () => (
  <APIProvider apiKey={API_KEY} libraries={['geometry']}>
    <Map
      style={{ width: '100vw', height: '100vh' }}
      defaultCenter={{ lat: 37.77, lng: -122.22 }}
      defaultZoom={12}
    />
  </APIProvider>
);

const root = createRoot(document.getElementById('app'));
root.render(<App />);