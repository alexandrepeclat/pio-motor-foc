const http = require('http');
const os = require('os');

const server = http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/plain' });
    res.end('WebSocket server running\n');
});

// Crée une instance WebSocket
const WebSocket = require('ws');
const wss = new WebSocket.Server({ server });

// const commands = [
// 'L000I1000',
// 'L010I0500', 
// 'L020I1000', 
// 'L030I0500', 
// 'L040I1000', 
// 'L050I0500', 
// 'L040I1000', 
// 'L030I0500', 
// 'L020I1000',
// 'L010I0500',
// 'L099I1000',
// 'L000I1000',
// 'L099I0050',
// 'L000I0050',
// ];

const commands = [
    { action: 'L', axis: 0, value: 24, interval: 79 },
    { action: 'L', axis: 0, value: 86, interval: 147 },
    { action: 'L', axis: 0, value: 30, interval: 118 },
    { action: 'L', axis: 0, value: 82, interval: 97 },
    { action: 'L', axis: 0, value: 27, interval: 117 },
    { action: 'L', axis: 0, value: 81, interval: 137 },
    { action: 'L', axis: 0, value: 21, interval: 118 },
    { action: 'L', axis: 0, value: 83, interval: 138 },
    { action: 'L', axis: 0, value: 21, interval: 158 },
    { action: 'L', axis: 0, value: 84, interval: 131 },
    { action: 'L', axis: 0, value: 21, interval: 110 },
    { action: 'L', axis: 0, value: 87, interval: 130 },
    { action: 'L', axis: 0, value: 42, interval: 150 },
    { action: 'L', axis: 0, value: 99, interval: 123 },
    { action: 'L', axis: 0, value: 40, interval: 142 },
    { action: 'L', axis: 0, value: 99, interval: 114 },
    { action: 'L', axis: 0, value: 39, interval: 133 },
    { action: 'L', axis: 0, value: 87, interval: 191 },
    { action: 'L', axis: 0, value: 39, interval: 77 },
    { action: 'L', axis: 0, value: 91, interval: 104 },
    { action: 'L', axis: 0, value: 36, interval: 124 },
    { action: 'L', axis: 0, value: 83, interval: 144 },
    { action: 'L', axis: 0, value: 83, interval: 116 },
    { action: 'L', axis: 0, value: 18, interval: 15 },
    { action: 'L', axis: 0, value: 80, interval: 89 },
    { action: 'L', axis: 0, value: 20, interval: 155 },
    { action: 'L', axis: 0, value: 20, interval: 2000 },
];

function formatCommand(command) {
    return `${command.action}${command.axis}${command.value}I${command.interval}`;
}

wss.on('connection', (ws) => {
    console.log('Client connected');

    let count = 0;

    function sendNextCommand() {
        if (ws.readyState === WebSocket.OPEN) {
            const command = commands[count];
            const commandString = formatCommand(command);
            const commandBuffer = Buffer.from(commandString, 'utf-8');

            console.log("Sending ", command);
            ws.send(commandBuffer);

            const delay = command.interval;
            count = (count + 1) % commands.length; // Repart à 0 après la dernière commande

            setTimeout(sendNextCommand, delay);
        } else {
            console.log('Connection closed, stopping loop.');
        }
    }

    sendNextCommand();

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

// Démarre le serveur sur le port 1234
server.listen(1234, () => {
    console.log('Server is listening on ws://localhost:1234');
    const ipAddress = getLocalIPAddress();
    console.log(`Server running at http://${ipAddress}:1234/`);
});

// Fonction pour obtenir l'adresse IP par défaut
function getLocalIPAddress() {
    const interfaces = os.networkInterfaces();
    for (let iface in interfaces) {
        for (let alias of interfaces[iface]) {
            if (alias.family === 'IPv4' && !alias.internal) {
                return alias.address; // On retourne la première adresse IPv4 non interne
            }
        }
    }
    return 'IP non trouvée';
}