document.addEventListener('DOMContentLoaded', () => {
    // Check if required libraries are loaded
    if (typeof CBOR === 'undefined') {
        console.error('CBOR library not loaded!');
        return;
    }
    if (typeof pako === 'undefined') {
        console.error('pako library not loaded!');
        return;
    }
    console.log('All libraries loaded successfully');

    const WS_URL = 'ws://localhost:8765';
    const statusElement = document.getElementById('connection-status');
    const canvas = document.getElementById('map-canvas');
    const ctx = canvas.getContext('2d');

    let socket;
    let mapMetadata = null;
    let costmapMetadata = null;

    const costmapColors = [
        { r: 0, g: 0, b: 0, a: 0 }, // 0: No obstacle
        // ... many colors for intermediate costs
        { r: 254, g: 0, b: 0, a: 255 }, // 253: Inscribed obstacle
        { r: 255, g: 0, b: 255, a: 255 }, // 254: Lethal obstacle
        { r: 0, g: 0, b: 0, a: 255 }  // 255: No information
    ];
    // Pre-fill the costmap color table
    for (let i = 1; i < 253; i++) {
        const r = Math.round((i / 252.0) * 255);
        costmapColors[i] = { r: r, g: 255 - r, b: 0, a: 255 };
    }


    function connect() {
        socket = new WebSocket(WS_URL);

        socket.onopen = () => {
            statusElement.textContent = 'Connected';
            statusElement.style.backgroundColor = 'green';
        };

        socket.onclose = () => {
            statusElement.textContent = 'Disconnected';
            statusElement.style.backgroundColor = 'red';
            setTimeout(connect, 5000); // Try to reconnect after 5 seconds
        };

        socket.onerror = (error) => {
            console.error('WebSocket Error:', error);
            socket.close();
        };

        socket.onmessage = async (event) => {
            try {
                const data = await event.data.arrayBuffer();
                const message = CBOR.decode(data);
                console.log('Received message:', message.t, message.p);
                handleMessage(message.t, message.p);
            } catch (error) {
                console.error('Error processing message:', error);
            }
        };
    }

    function handleMessage(topic, payload) {
        console.log('Handling message type:', topic);
        switch (topic) {
            case 'map':
                console.log('MAP message received!');
                mapMetadata = payload;
                drawMap(payload);
                break;
            case 'global_costmap':
                console.log('GLOBAL_COSTMAP message received!');
                costmapMetadata = payload;
                drawCostmap(payload);
                break;
            case 'global_costmap_update':
                updateCostmap(payload);
                break;
            default:
                console.log('Unhandled message type:', topic);
        }
    }

    function drawMap(map) {
        try {
            console.log('Drawing map:', map.w, 'x', map.h);
            const { w, h, data } = map;
            canvas.width = w;
            canvas.height = h;
            
            console.log('Map data type:', typeof data, 'Length:', data.length);
            console.log('Map first few bytes:', Array.from(data.slice(0, 10)));
            
            // Try different decompression methods
            let inflatedData;
            try {
                // Try raw deflate first
                inflatedData = pako.inflateRaw(new Uint8Array(data));
                console.log('Map raw deflate successful');
            } catch (e1) {
                console.log('Map raw deflate failed:', e1.message);
                try {
                    // Try standard zlib
                    inflatedData = pako.inflate(new Uint8Array(data));
                    console.log('Map standard zlib successful');
                } catch (e2) {
                    console.log('Map standard zlib failed:', e2.message);
                    // Try gzip
                    inflatedData = pako.ungzip(new Uint8Array(data));
                    console.log('Map gzip successful');
                }
            }
            
            console.log('Inflated data length:', inflatedData.length);
            const imageData = ctx.createImageData(w, h);

        for (let i = 0; i < inflatedData.length; i++) {
            const val = inflatedData[i];
            let r, g, b;
            if (val === 100) { // Occupied
                r = g = b = 0;
            } else if (val === 0) { // Free
                r = g = b = 255;
            } else { // Unknown
                r = g = b = 128;
            }
            imageData.data[i * 4] = r;
            imageData.data[i * 4 + 1] = g;
            imageData.data[i * 4 + 2] = b;
            imageData.data[i * 4 + 3] = 255;
        }
        ctx.putImageData(imageData, 0, 0);
        console.log('Map drawn successfully');
        } catch (error) {
            console.error('Error drawing map:', error);
        }
    }

    function drawCostmap(costmap) {
        try {
            console.log('Drawing costmap:', costmap.w, 'x', costmap.h);
            const { w, h, data } = costmap;
            
            // Set canvas size if not already set by map
            if (canvas.width !== w || canvas.height !== h) {
                canvas.width = w;
                canvas.height = h;
            }
            
            console.log('Data type:', typeof data, 'Length:', data.length);
            console.log('First few bytes:', Array.from(data.slice(0, 10)));
            
            // Use standard zlib decompression (we know this works from debugging)
            const inflatedData = pako.inflate(new Uint8Array(data));
            console.log('Costmap inflated data length:', inflatedData.length);
            const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);

        for (let i = 0; i < inflatedData.length; i++) {
            const cost = inflatedData[i];
            const color = costmapColors[cost];
            if (color && color.a > 0) { // Only draw non-transparent pixels if color exists
                imageData.data[i * 4] = color.r;
                imageData.data[i * 4 + 1] = color.g;
                imageData.data[i * 4 + 2] = color.b;
                imageData.data[i * 4 + 3] = color.a;
            } else if (!color) {
                // Handle undefined colors - log first few instances for debugging
                if (i < 10) {
                    console.log(`Unknown cost value at pixel ${i}: ${cost}`);
                }
                // Default to black for unknown costs
                imageData.data[i * 4] = 0;     // r
                imageData.data[i * 4 + 1] = 0; // g
                imageData.data[i * 4 + 2] = 0; // b
                imageData.data[i * 4 + 3] = 255; // a (opaque)
            }
        }
        ctx.putImageData(imageData, 0, 0);
        console.log('Costmap drawn successfully');
        } catch (error) {
            console.error('Error drawing costmap:', error);
        }
    }

    function updateCostmap(update) {
        if (!mapMetadata || !costmapMetadata) return;

        const { x, y, w, h, data } = update;
        const inflatedData = pako.inflate(new Uint8Array(data));
        const imageData = ctx.getImageData(x, y, w, h);

        for (let i = 0; i < inflatedData.length; i++) {
            const cost = inflatedData[i];
            const color = costmapColors[cost];
            if (color && color.a > 0) {
                imageData.data[i * 4] = color.r;
                imageData.data[i * 4 + 1] = color.g;
                imageData.data[i * 4 + 2] = color.b;
                imageData.data[i * 4 + 3] = color.a;
            } else if (!color) {
                // Default to black for unknown costs
                imageData.data[i * 4] = 0;
                imageData.data[i * 4 + 1] = 0;
                imageData.data[i * 4 + 2] = 0;
                imageData.data[i * 4 + 3] = 255;
            }
        }
        ctx.putImageData(imageData, x, y);
    }

    connect();
});
