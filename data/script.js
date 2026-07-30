        console.log("SCRIPT START");
        var gateway =  `ws://${window.location.hostname}/ws`;
        var websocket;

        window.onload = initWebSocket;
        
        function initWebSocket() 
        {
            websocket = new WebSocket(gateway);
            
            websocket.onopen = function() {
                document.getElementById("status-dot").style.backgroundColor = "#00e676";
                document.getElementById("status-dot").style.boxShadow = "0 0 8px #00e676";
                document.getElementById("status-text").innerText = "Connected";
            };

            websocket.onclose = function() {
                document.getElementById("status-dot").style.backgroundColor = "#f44336";
                document.getElementById("status-dot").style.boxShadow = "0 0 8px #f44336";
                document.getElementById("status-text").innerText = "Disconnected";
                
                document.getElementById("tele-distance").innerText = "-- mm";
                document.getElementById("tele-servo").innerText = "--°";
                document.getElementById("tele-mode").innerText = "--";
                
                setTimeout(initWebSocket, 2000);
            };

            websocket.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    
                    if (data.distance !== undefined) {
                        document.getElementById("tele-distance").innerText = data.distance + " mm";
                    }
                    if (data.servo !== undefined) {
                        document.getElementById("tele-servo").innerText = data.servo + "°";
                    }
                    if (data.mode !== undefined) {
                        document.getElementById("tele-mode").innerText = data.mode;
                    }
                } 
                catch (e) {
                    console.error("Error parsing JSON in telemetry: ", event.data);
                }
            };
        }
        
        function send(cmd) {
            if (websocket.readyState == WebSocket.OPEN) {
                websocket.send(cmd);
            }
        }
        