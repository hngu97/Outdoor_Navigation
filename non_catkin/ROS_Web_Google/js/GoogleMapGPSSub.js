// Subscribing to a Topic
// ----------------------
/*var gpsSub1 = new ROSLIB.Topic({
  ros : ros,
  name : '/fix',
  messageType : 'sensor_msgs/NavSatFix'
});


gpsSub1.subscribe(function(msg) {
  origin_lat = msg.latitude;
  origin_lng = msg.longitude;
  gpsSub1.unsubscribe();
});*/


var gpsSub2 = new ROSLIB.Topic({
  ros : ros,
  name : '/fix',
  messageType : 'sensor_msgs/NavSatFix'
});

var currentGPSMarker = null;
gpsSub2.subscribe(function(msg)
{
  if (currentGPSMarker == null)
  {
    currentGPSMarker = new google.maps.Marker
    ({
      position: new google.maps.LatLng(msg.latitude, msg.longitude),
      label: 'U',
      map: map
    });
  }
  else
  {
    currentGPSMarker.setPosition(new google.maps.LatLng(msg.latitude, msg.longitude));
  }
});


var gpsSub3 = new ROSLIB.Topic({
  ros : ros,
  name : '/gps/filtered',
  messageType : 'sensor_msgs/NavSatFix'
});

var filteredGPSMarker = null;
gpsSub3.subscribe(function(msg)
{
  if (filteredGPSMarker == null)
  {
    filteredGPSMarker = new google.maps.Marker
    ({
      position: new google.maps.LatLng(msg.latitude, msg.longitude),
      /*icon: {
        path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
        rotation: 90,
        scale: 4
      },*/
      label: 'F',
      map: map
    });
  }
  else
  {
    filteredGPSMarker.setPosition(new google.maps.LatLng(msg.latitude, msg.longitude));
  }
});
