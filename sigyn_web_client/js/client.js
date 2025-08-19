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
    const costmapCanvas = document.getElementById('costmap-canvas');
    const costmapCtx = costmapCanvas.getContext('2d');

    let lastMapData = null; // Store map data for redrawing

    function scaleCanvas(w, h) {
        const container = canvas.parentElement;
        const containerRect = container.getBoundingClientRect();
        const containerWidth = containerRect.width - 4; // Account for border
        const containerHeight = containerRect.height - 4;

        // Calculate scale to fit container while maintaining aspect ratio
        const scaleX = containerWidth / w;
        const scaleY = containerHeight / h;
        const scale = Math.min(scaleX, scaleY);

        // Only change intrinsic size if it actually changed (changing clears content)
        const sizeChanged = (canvas.width !== w || canvas.height !== h || costmapCanvas.width !== w || costmapCanvas.height !== h);
        if (sizeChanged) {
            canvas.width = w;
            canvas.height = h;
            costmapCanvas.width = w;
            costmapCanvas.height = h;
        }

        // Always update CSS size (does not clear)
        [canvas, costmapCanvas].forEach(canv => {
            canv.style.width = (w * scale) + 'px';
            canv.style.height = (h * scale) + 'px';
        });

        console.log(`Canvas: ${w}x${h}, Container: ${containerWidth}x${containerHeight}, Scale: ${scale.toFixed(2)}, sizeChanged=${sizeChanged}`);
        return sizeChanged;
    }
    let socket;
    let mapMetadata = null;
    let costmapMetadata = null;

    const costmapColors = [];
    // Initialize all 256 color slots to handle any costmap values
    // Based on ROS costmap_2d values: 0=free, 1-97=scaling, 98=inscribed, 99=lethal, 100=no_information
    for (let i = 0; i <= 255; i++) {
        if (i === 0) {
            // Free space - transparent so map shows through
            costmapColors[i] = { r: 0, g: 0, b: 0, a: 0 };
        } else if (i >= 1 && i <= 97) {
            // Cost values 1-97: Blue to cyan to yellow to orange gradient
            const ratio = i / 97.0;
            if (ratio < 0.33) {
                // Blue to cyan (low cost)
                const t = ratio * 3;
                costmapColors[i] = { r: 0, g: Math.round(255 * t), b: 255, a: 150 };
            } else if (ratio < 0.66) {
                // Cyan to yellow (medium cost)
                const t = (ratio - 0.33) * 3;
                costmapColors[i] = { r: Math.round(255 * t), g: 255, b: 255 - Math.round(255 * t), a: 150 };
            } else {
                // Yellow to orange (high cost)
                const t = (ratio - 0.66) * 3;
                costmapColors[i] = { r: 255, g: 255 - Math.round(127 * t), b: 0, a: 150 };
            }
        } else if (i === 98) {
            // Inscribed obstacle - bright orange
            costmapColors[i] = { r: 255, g: 165, b: 0, a: 200 };
        } else if (i === 99) {
            // Lethal obstacle - bright red
            costmapColors[i] = { r: 255, g: 0, b: 0, a: 220 };
        } else if (i === 100) {
            // No information - purple/magenta
            costmapColors[i] = { r: 128, g: 0, b: 128, a: 180 };
        } else if (i >= 101 && i <= 252) {
            // Extended range - red variants
            const ratio = (i - 101) / 151.0;
            costmapColors[i] = { r: 255, g: Math.round(100 * (1 - ratio)), b: Math.round(100 * ratio), a: 200 };
        } else if (i === 253) {
            // Special case - bright red
            costmapColors[i] = { r: 255, g: 0, b: 0, a: 255 };
        } else if (i === 254) {
            // Special case - magenta
            costmapColors[i] = { r: 255, g: 0, b: 255, a: 255 };
        } else { // i === 255
            // Unknown - dark gray
            costmapColors[i] = { r: 64, g: 64, b: 64, a: 150 };
        }
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
                // If we already have costmap data, redraw it on top
                if (costmapMetadata) {
                    setTimeout(() => drawCostmap(costmapMetadata), 10);
                }
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

            const resized = scaleCanvas(w, h);

            // Use standard zlib decompression (we know this works)
            const inflatedData = pako.inflate(new Uint8Array(data));
            console.log('Map inflated data length:', inflatedData.length);

            lastMapData = { w, h, inflatedData }; // Store for redrawing
            drawMapData(w, h, inflatedData);

        } catch (error) {
            console.error('Error drawing map:', error);
        }
    }

    function drawMapData(w, h, inflatedData) {
        const imageData = ctx.createImageData(w, h);

        // Draw each pixel with proper ROS map value interpretation, flipped vertically
        for (let row = 0; row < h; row++) {
            for (let col = 0; col < w; col++) {
                // Original index in the data array
                const dataIndex = row * w + col;
                // Flipped index in the image data (flip vertically)
                const flippedRow = h - 1 - row;
                const pixelIndex = (flippedRow * w + col) * 4;

                // Inflate returns Uint8 (0..255). In OccupancyGrid: -1=unknown, 0=free, 100=occupied.
                // Map -1 to 255 in Uint8, so convert back for logic.
                const raw = inflatedData[dataIndex];
                const val = (raw === 255) ? -1 : raw;

                let r, g, b;
                if (val === -1) {
                    // Unknown space - light gray like RViz (around 205)
                    r = g = b = 205;
                } else if (val === 0) {
                    // Free space - white
                    r = g = b = 255;
                } else if (val === 100) {
                    // Occupied - black
                    r = g = b = 0;
                } else {
                    // Intermediate probabilities 1..99 -> grayscale (white->black)
                    const gray = 255 - Math.round((val / 100) * 255);
                    r = g = b = Math.max(0, Math.min(255, gray));
                }

                imageData.data[pixelIndex] = r;
                imageData.data[pixelIndex + 1] = g;
                imageData.data[pixelIndex + 2] = b;
                imageData.data[pixelIndex + 3] = 255; // Fully opaque
            }
        }
        ctx.putImageData(imageData, 0, 0);
        console.log('Map drawn successfully');
    }

    function drawCostmap(costmap) {
        try {
            console.log('Drawing costmap:', costmap.w, 'x', costmap.h);
            const { w, h, data } = costmap;

            const resized = scaleCanvas(w, h);

            // If size changed, redraw the map layer that was cleared by resizing
            if (resized && lastMapData) {
                drawMapData(lastMapData.w, lastMapData.h, lastMapData.inflatedData);
            }

            // Clear the costmap canvas completely
            costmapCtx.clearRect(0, 0, w, h);

            // Decompress costmap data
            const inflatedData = pako.inflate(new Uint8Array(data));
            console.log('Costmap inflated data length:', inflatedData.length);

            // Analyze cost values for debugging
            const costCounts = {};
            for (let i = 0; i < Math.min(1000, inflatedData.length); i++) {
                const cost = inflatedData[i];
                costCounts[cost] = (costCounts[cost] || 0) + 1;
            }
            console.log('Sample cost values:', Object.keys(costCounts).sort((a,b) => b-a).slice(0, 10));

            // Create costmap image data - start completely transparent
            const costmapImageData = costmapCtx.createImageData(w, h);

            // Draw each pixel based on cost value using exact RViz2 mapping
            // Flip vertically by reading rows in reverse order
            for (let row = 0; row < h; row++) {
                for (let col = 0; col < w; col++) {
                    const dataIndex = row * w + col;
                    const flippedRow = h - 1 - row;
                    const pixelIndex = (flippedRow * w + col) * 4;

                    const cost = inflatedData[dataIndex];

                    if (cost === 0) {
                        // Free space - leave completely transparent (RGBA = 0,0,0,0)
                        // costmapImageData already initialized to zeros
                        continue;
                    } else if (cost >= 1 && cost <= 98) {
                        const v = (255 * cost) / 100;
                        costmapImageData.data[pixelIndex] = v;         // r
                        costmapImageData.data[pixelIndex + 1] = 0;     // g
                        costmapImageData.data[pixelIndex + 2] = 255 - v; // b
                        costmapImageData.data[pixelIndex + 3] = 51;    // a (20% opacity)
                    } else if (cost === 99) {
                        costmapImageData.data[pixelIndex] = 0;       // r
                        costmapImageData.data[pixelIndex + 1] = 255; // g (cyan)
                        costmapImageData.data[pixelIndex + 2] = 255; // b (cyan)
                        costmapImageData.data[pixelIndex + 3] = 51;  // a (20% opacity)
                    } else if (cost === 100) {
                        costmapImageData.data[pixelIndex] = 255;     // r (magenta)
                        costmapImageData.data[pixelIndex + 1] = 0;   // g
                        costmapImageData.data[pixelIndex + 2] = 255; // b (magenta)
                        costmapImageData.data[pixelIndex + 3] = 51;  // a (20% opacity)
                    } else if (cost >= 101 && cost <= 127) {
                        costmapImageData.data[pixelIndex] = 0;       // r
                        costmapImageData.data[pixelIndex + 1] = 255; // g (green)
                        costmapImageData.data[pixelIndex + 2] = 0;   // b
                        costmapImageData.data[pixelIndex + 3] = 51;  // a (20% opacity)
                    } else if (cost >= 128 && cost <= 254) {
                        const yellowComponent = (255 * (cost - 128)) / (254 - 128);
                        costmapImageData.data[pixelIndex] = 255;                    // r (red)
                        costmapImageData.data[pixelIndex + 1] = yellowComponent;    // g
                        costmapImageData.data[pixelIndex + 2] = 0;                  // b
                        costmapImageData.data[pixelIndex + 3] = 51;                 // a (20% opacity)
                    } else { // cost === 255
                        costmapImageData.data[pixelIndex] = 0x70;    // r
                        costmapImageData.data[pixelIndex + 1] = 0x89; // g
                        costmapImageData.data[pixelIndex + 2] = 0x86; // b
                        costmapImageData.data[pixelIndex + 3] = 51;   // a (20% opacity)
                    }
                }
            }

            // Draw the costmap data to the costmap canvas (no global alpha needed)
            costmapCtx.putImageData(costmapImageData, 0, 0);

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
    
    // Handle window resize
    window.addEventListener('resize', () => {
        if (lastMapData) {
            const resized = scaleCanvas(lastMapData.w, lastMapData.h);
            // Redraw everything on resize
            drawMapData(lastMapData.w, lastMapData.h, lastMapData.inflatedData);
            // Costmap will be redrawn on next update
        }
    });
});
