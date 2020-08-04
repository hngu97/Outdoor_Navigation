var ros = new ROSLIB.Ros({
  url : 'ws://192.168.1.2:9090'
});

/*var ros = new ROSLIB.Ros({
  url : 'ws://172.20.10.5:9090'
});*/

/*var ros = new ROSLIB.Ros({
  url : 'ws://118.138.68.41:9090'
});*/

ros.on('connection', function() {
  //console.log('Connected to websocket server.');
  document.getElementById("status").innerHTML = "Connected";
  document.getElementById("status").style.color = "#00ff00";
});

ros.on('error', function(error) {
  //console.log('Error connecting to websocket server: ', error);
  document.getElementById("status").innerHTML = "Error";
  document.getElementById("status").style.color = "#ff3300";
});

ros.on('close', function() {
  //console.log('Connection to websocket server closed.');
  document.getElementById("status").innerHTML = "Closed";
  document.getElementById("status").style.color = "#19334d";
});

let filteredLat = 0;
let filteredLng = 0;
let currentLat = 0;
let currentLng = 0;
let origin_lat = 0;
let origin_lng = 0;
let map = {};

origin_lat =  -37.9103184688932;//-37.883738;
origin_lng =  145.13304612603062; //145.161580;
