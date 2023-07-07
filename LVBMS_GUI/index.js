let pcm = new PCM("localhost:41000", on_connected, on_close, on_error);

function on_connected() {
  console.log("Connected to WebSocket");	
};

function on_error() {
  console.log("Error with the WebSocket");
};

function on_close() {
  console.log("WebSocket Close");
}