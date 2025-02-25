const http = require('http');
const os = require('os');

const server = http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/plain' });
    res.end('WebSocket server running\n');
});

// Crée une instance WebSocket
const WebSocket = require('ws');
const wss = new WebSocket.Server({ server });

const commands = [
    'L000I1000',
    'L010I0500', 
    'L020I1000', 
    'L030I0500', 
    'L040I1000', 
    'L050I0500', 
    'L040I1000', 
    'L030I0500', 
    'L020I1000',
    'L010I0500',
    'L000I1000'
];

wss.on('connection', (ws) => {
    console.log('Client connected');
    
    let count = 0;
    const intervalId = setInterval(() => {
        if (count < commands.length && ws.readyState === WebSocket.OPEN) {
            // Crée un Buffer à partir de la commande
            const commandBuffer = Buffer.from(commands[count], 'utf-8');
            // Envoie le Buffer comme Blob
            console.log("Sending ", commands[count]);
            ws.send(commandBuffer);
            count++;
        } else {
            clearInterval(intervalId);
            ws.close(); // Ferme la connexion après l'envoi des 10 messages
        }
    }, 1000); // Envoie un message toutes les secondes

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