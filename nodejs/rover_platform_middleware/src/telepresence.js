
const server = require('http').createServer();
const io = require('socket.io')(server);


io.on('connection', (client) => {

    client.on('receiveImage', (data) => {
        io.sockets.emit('broadcastImage', data);
    });

});

const port = 8000;
io.listen(port);
console.log('listening on port ', port);