// Video_Camera positions
var lat = 41.693802;
var lon = -91.638302;

// Store (camera id: marker group) pairs
const cameraMarkers = new Map();

// Leaflet map
var map = L.map('map').setView([lat, lon], 15);

L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

// Connect to the WebSocket server
var socket = new WebSocket("ws://127.0.0.1:8765");

socket.onopen = function() {
    console.log("WebSocket connected");
};

socket.onerror = function(err) {
    console.error("WebSocket error:", err);
};

// When a message is received
socket.onmessage = function(event) {
  // Enable for debug
  // console.error(event);

  // Parse incoming JSON data
  var data = JSON.parse(event.data);

  // On a control/reset frame, clear current layer for a camera if it exists as
  // it will be sending its points
  if (data.lat == 0 && data.lon == 0) {
    if (cameraMarkers.has(data.camera_name)) {
      cameraMarkers.get(data.camera_name).clearLayers();
    }
    return;
  }

  // If a markerGroup for this camera has not been added, add one
  if (!cameraMarkers.has(data.camera_name)) {
    cameraMarkers.set(data.camera_name, L.layerGroup().addTo(map));
  }
  
  // Update marker position
  L.marker([data.lat, data.lon]).addTo(cameraMarkers.get(data.camera_name));
};

