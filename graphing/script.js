// Camera positions
var lat = 38.560719;
var lon = -121.480659;

var map = L.map('map').setView([lat, lon], 25);

L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

var marker = L.marker([lat, lon]).addTo(map);

// Connect to the WebSocket server
var socket = new WebSocket("ws://localhost:8765");

socket.onopen = function() {
    console.log("WebSocket connected");
};

socket.onerror = function(err) {
    console.error("WebSocket error:", err);
};

// When a message is received
socket.onmessage = function(event) {
  // Parse incoming JSON data
  var data = JSON.parse(event.data);
  
  // Update marker position
  marker.setLatLng([data.lat, data.lon]);
};

