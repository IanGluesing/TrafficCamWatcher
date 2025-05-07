import asyncio
import websockets
import json
import random

lat = 38.560719
lon = -121.480659

async def send_position(websocket):
    global lat, lon
    while True:
        lat += 0.0001
        lon += 0.0001

        message = json.dumps({"lat": lat, "lon": lon})
        await websocket.send(message)

        await asyncio.sleep(0.1)

async def main():
    try:
        async with websockets.serve(send_position, "localhost", 8765):
            print("WebSocket server running at ws://localhost:8765")
            await asyncio.Future()  # run forever
    except Exception as e:
        print(f"Error starting WebSocket server: {e}")

if __name__ == "__main__":
    asyncio.run(main())

