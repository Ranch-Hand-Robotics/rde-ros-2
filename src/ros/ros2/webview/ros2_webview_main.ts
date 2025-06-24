// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// VS Code API declarations
declare function acquireVsCodeApi(): any;

// Acquire VS Code API once and cache it
const vscode = acquireVsCodeApi();

namespace ros2monitor {
    function removeAllChildElements(e) {
        while (e.firstChild) {
            e.removeChild(e.firstChild);
        }
    };

    function generateColumnTable(dataArray: any, headers: string[], callback: (data: any, i: number) => string) {
        let t = document.createElement("table");
        let th = document.createElement("thead");
        let headerRow = document.createElement("tr");
        headers.forEach((name, _i) => {
            let h = document.createElement("th");
            h.appendChild(document.createTextNode(name));
            headerRow.appendChild(h);
        });

        th.appendChild(headerRow);
        t.appendChild(th);

        let tb = document.createElement("tbody");
        for (const i in dataArray) {
            const data = dataArray[i];
            const r = document.createElement("tr");
            headers.forEach((name, _i) => {
                let cell = document.createElement("td");
                cell.appendChild(document.createTextNode(callback(data, _i)));
                r.appendChild(cell);
            });
            tb.appendChild(r);
        }
        t.appendChild(tb);

        return t;
    }

    export function initializeRos2Monitor() {
        
        // Initialize menu bar event handlers
        const toggleDaemonBtn = document.getElementById("daemon-toggle-btn") as HTMLButtonElement;
        const daemonStatusMessage = document.getElementById("daemon-status-message") as HTMLSpanElement;
        
        let isDaemonRunning = false;

        toggleDaemonBtn.addEventListener("click", () => {
            //toggleDaemonBtn.disabled = true;
            
            // Use cached vscode API
            if (isDaemonRunning) {
                daemonStatusMessage.textContent = "Stopping daemon...";
                vscode.postMessage({
                    command: 'stopDaemon'
                });
            } else {
                daemonStatusMessage.textContent = "Starting daemon...";
                vscode.postMessage({
                    command: 'startDaemon'
                });
            }
        });

        // handle message passed from extension to webview
        window.addEventListener("message", (event) => {
            const message = event.data;

            // Handle daemon control messages
            if (message.daemonAction) {
                const toggleBtn = document.getElementById("daemon-toggle-btn") as HTMLButtonElement;
                const statusMsg = document.getElementById("daemon-status-message") as HTMLSpanElement;

                switch (message.daemonAction) {
                    case 'starting':
                        statusMsg.textContent = message.message;
                        // Don't update isDaemonRunning yet, wait for 'started'
                        setTimeout(() => {
                            statusMsg.textContent = "";
                        }, 5000);
                        break;
                    case 'started':
                        statusMsg.textContent = message.message;
                        isDaemonRunning = message.isRunning;
                        // Update button immediately to reflect new state
                        toggleBtn.disabled = false;
                        toggleBtn.textContent = isDaemonRunning ? "Stop Daemon" : "Start Daemon";
                        toggleBtn.className = isDaemonRunning ? "menu-button stop" : "menu-button";
                        setTimeout(() => {
                            statusMsg.textContent = "";
                        }, 5000);
                        break;
                    case 'stopping':
                        statusMsg.textContent = message.message;
                        // Don't update isDaemonRunning yet, wait for 'stopped'
                        setTimeout(() => {
                            statusMsg.textContent = "";
                        }, 5000);
                        break;
                    case 'stopped':
                        statusMsg.textContent = message.message;
                        isDaemonRunning = message.isRunning;
                        // Update button immediately to reflect new state
                        toggleBtn.disabled = false;
                        toggleBtn.textContent = isDaemonRunning ? "Stop Daemon" : "Start Daemon";
                        toggleBtn.className = isDaemonRunning ? "menu-button stop" : "menu-button";
                        setTimeout(() => {
                            statusMsg.textContent = "";
                        }, 5000);
                        break;
                    case 'error':
                        statusMsg.textContent = message.message;
                        isDaemonRunning = message.isRunning;
                        toggleBtn.disabled = false;
                        toggleBtn.textContent = isDaemonRunning ? "Stop Daemon" : "Start Daemon";
                        toggleBtn.className = isDaemonRunning ? "menu-button stop" : "menu-button";
                        setTimeout(() => {
                            statusMsg.textContent = "";
                        }, 5000);
                        break;
                }
                return;
            }

            const topicsElement = document.getElementById("topics");
            const servicesElement = document.getElementById("services");

            removeAllChildElements(topicsElement);
            removeAllChildElements(servicesElement);

            if (message.ready) {
                // Update daemon state based on polling result
                if (typeof message.isDaemonRunning === 'boolean') {
                    isDaemonRunning = message.isDaemonRunning;
                    const toggleBtn = document.getElementById("daemon-toggle-btn") as HTMLButtonElement;
                    toggleBtn.textContent = isDaemonRunning ? "Stop Daemon" : "Start Daemon";
                    toggleBtn.className = isDaemonRunning ? "menu-button stop" : "menu-button";
                }

                const nodes = JSON.parse(message.nodes);
                const topics = JSON.parse(message.topics);
                const services = JSON.parse(message.services);

                const nodesHeader = document.createElement("h2");
                nodesHeader.appendChild(document.createTextNode("Nodes"));
                topicsElement.appendChild(nodesHeader);
                topicsElement.appendChild(generateColumnTable(nodes, ["Name"], (data, i) => {
                    return `${data[1]}${data[0]}`;
                }));

                const topicsHeader = document.createElement("h2");
                topicsHeader.appendChild(document.createTextNode("Topics"));
                topicsElement.appendChild(topicsHeader);
                topicsElement.appendChild(generateColumnTable(topics, ["Name", "Type"], (data, i) => {
                    return data[i];
                }));

                const servicesHeader = document.createElement("h2");
                servicesHeader.appendChild(document.createTextNode("Services"));
                servicesElement.appendChild(servicesHeader);
                servicesElement.appendChild(generateColumnTable(services, ["Name", "Type"], (data, i) => {
                    return data[i];
                }));
            }
            else {
                // Update daemon state based on polling result
                if (typeof message.isDaemonRunning === 'boolean') {
                    isDaemonRunning = message.isDaemonRunning;
                    const toggleBtn = document.getElementById("daemon-toggle-btn") as HTMLButtonElement;
                    toggleBtn.textContent = isDaemonRunning ? "Stop Daemon" : "Start Daemon";
                    toggleBtn.className = isDaemonRunning ? "menu-button stop" : "menu-button";
                }
            }
        });
    };
}

window.onload = () => ros2monitor.initializeRos2Monitor();
