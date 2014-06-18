// to start the server in the command line: node express-server.js
var ok_state = 0;
var fs = require("fs");
var host = "127.0.0.1";
var port = 8092;
var express = require("express");
// serialport
var SerialPort = require("serialport").SerialPort;
var serialPort = new SerialPort("COM3", {
                             // "/dev/cu.usbserial-A603OPML"
  baudrate: 9600
}, false); 

// function to execute callback after data is transmitted, check serialport documentation for more ingo
function writeAndDrain (data, callback) {
  console.log('Sending \''+data+'\'');
  serialPort.write(data, function () {
    serialPort.drain(callback);
  });
}


// server
var app = express();
app.use(express.static(__dirname + '/public'));
app.use(express.static(__dirname + '/public/css'));
app.use(express.static(__dirname + '/public/images'));
app.use(express.static(__dirname + '/public/js'));

app.get("/index", function(req, res){ //root dir
    res.sendfile("public/index.html");
});

app.get("/hardware/:code", function(req, res){ 
    // here the code is sent from the html page to the nodeserver
    // it can be a string or a node buffer; to turn the led im using 0x01, off 0x00 
    var codeToSend = req.params.code;
    // new Buffer([codeToSend])
    if (codeToSend == 1) {
      writeAndDrain('1\r\n', function(err, results){
        console.log('finished sending...');
        if (typeof err != 'undefined') console.log('errors: '+err+'\n');
        console.log('results: '+results);
      });
      res.sendfile("public/on.html");
    } else {
      writeAndDrain('0\r\n', function(err, results){
        console.log('finished sending...');
        if (typeof err != 'undefined') console.log('errors: '+err+'\n');
        console.log('results: '+results);
      });
      res.sendfile("public/off.html");
    }
});

app.get("/draw", function(req, res){
    var x = req.query.x; // UNTESTED
    var y = req.query.y;
    writeAndDrain('G1 X'+x+' Y'+y+'\r\n', function(err, results){
      //console.log('finished sending...');
      if (typeof err != 'undefined') console.log('errors: '+err+'\n');
      //console.log('results: '+results);
    });
    res.sendfile("public/ok.html");
});

app.get("/close", function(req, res){ //root dir
    res.send("server closed... ssh!");
    server.close();
});

serialPort.open(function () {
  console.log('open');
  serialPort.on('data', function(data) {
    var str = data + '';
    //console.log('packet: '+data);
    for (var i=0, chr; i < str.length; i++) {
        chr = str.charAt(i);
        if ((ok_state == 0)
             && (chr == 'o')) {
           ok_state = 1;
        } else if ((ok_state == 1)
                    && (chr == 'k')) {
           ok_state = 2;
        } else if ((ok_state == 2)
                    && (chr.charCodeAt(0) == 13)) {
           ok_state = 3;
        } else if ((ok_state == 3)
                    && (chr.charCodeAt(0) == 10)) {
           console.log('Received OK');
           ok_state = 0;
        } else if ((ok_state == 0)
             && (chr == 'r')) {
           ok_state = 101;
        } else if ((ok_state == 101)
                    && (chr == 'e')) {
           ok_state = 102;
        } else if ((ok_state == 102)
                    && (chr == 'a')) {
           ok_state = 103;
        } else if ((ok_state == 103)
                    && (chr == 'd')) {
           ok_state = 104;
        } else if ((ok_state == 104)
                    && (chr == 'y')) {
           ok_state = 105;
        } else if ((ok_state == 105)
                    && (chr.charCodeAt(0) == 13)) {
          ok_state = 106;
        } else if (((ok_state == 106) || (ok_state == 105))
                    && (chr.charCodeAt(0) == 10)) {
           console.log('Received READY');
          ok_state = 0;
        } else if ((ok_state == 0)
                    && (chr == ';')) { // diagnostics from arduino
           process.stdout.write(chr);
           ok_state = 201;
        } else if ((ok_state == 201)
                    && (chr.charCodeAt(0) >= 32)) {
           process.stdout.write(chr);
        } else if ((ok_state == 201)
                    && (chr.charCodeAt(0) == 13)) {
           ok_state = 202;
        } else if (((ok_state == 202) || (ok_state == 201))
                    && (chr.charCodeAt(0) == 10)) {
           process.stdout.write('\n');
           ok_state = 0;
        } else {
           process.stdout.write('state='+ok_state+'  char: <'
                             +chr.charCodeAt(0)
                             +'> '+chr.toString()+'\n');
        }
    }
  })
});

// start the server	
var server = app.listen(port, function(){
	console.log("Listening on port %d", server.address().port);
});
