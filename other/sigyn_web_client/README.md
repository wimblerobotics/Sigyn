# Sigyn Web Client

This package contains a simple browser-based client for visualizing map and costmap data from the `sigyn_websocket` server.

## How to Run

1.  **Ensure the `sigyn_websocket` server is running.**
    ```bash
    ros2 run sigyn_websocket server
    ```

2.  **Serve the web client files.**
    Since this is not a ROS package, you need a simple HTTP server to serve the files to your browser. Python's built-in HTTP server is perfect for this.

    Open a new terminal and run the following command from within the `sigyn_web_client` directory:
    ```bash
    cd /home/ros/sigyn_ws/src/Sigyn/sigyn_web_client
    python3 -m http.server
    ```

3.  **Open the client in your browser.**
    Open your web browser and navigate to:
    [http://localhost:8000](http://localhost:8000)

You should see a canvas displaying the map and costmap data from your robot.

## File Structure

-   `index.html`: The main HTML file.
-   `css/style.css`: Contains the styles for the page.
-   `js/client.js`: The core JavaScript application logic. It connects to the WebSocket server, receives CBOR-encoded and compressed data, and draws it on the HTML canvas.
