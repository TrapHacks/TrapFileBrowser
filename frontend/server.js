var app = require('http').createServer(handler)
var io = require('socket.io')(app);
var fs = require('fs');
var currentFiles = [];
app.listen(8001);

function handler (req, res) {
  fs.readFile(__dirname + '/index.html',
  function (err, data) {
    if (err) {
      res.writeHead(500);
      return res.end('Error loading index.html');
    }

    res.writeHead(200);
    res.end(data);
  });
}

io.on('connection', function (socket) {
  socket.emit('updateDir', currentFiles);
  socket.on('enterFile', function (data) {
    console.log("user entered: " + data);
  });
});

var net = require('net');
var server = net.createServer(function(c) { //'connection' listener
  console.log('client connected');
  c.on('data', function(data) {
    var str = data.toString('ascii').split(" ");
    console.log(str);
    if(str[0] == "0") { //changing directory
      currentFiles = [];
      for(var i = 1; i < str.length; i++) {
        currentFiles[i-1] = str[i];
      }
      console.log("YOOOO");
      io.sockets.emit('updateDir', currentFiles);
    }

    if(str[0] == "1") { //changing selector
      

      io.sockets.emit('updateIndex', parseInt(str[1]));
    }

    c.write("you entered " + data.toString('ascii'));
  });
  c.on('end', function() {
    console.log('client disconnected');
  });
});
server.listen(8124, function() { //'listening' listener
  console.log('server bound');
  
});


