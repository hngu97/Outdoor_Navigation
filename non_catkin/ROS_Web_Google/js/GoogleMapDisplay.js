var directionsService = null;
var directionsRenderer = null;
function initMap()
{
  directionsService = new google.maps.DirectionsService();
  directionsRenderer = new google.maps.DirectionsRenderer();
  var monashClayton = new google.maps.LatLng(origin_lat, origin_lng);
  var mapOptions =
  {
    zoom: 19,
    center: monashClayton,
    mapTypeId: 'hybrid'
    /*styles:
    [
      {elementType: 'geometry', stylers: [{color: '#242f3e'}]},
      {elementType: 'labels.text.stroke', stylers: [{color: '#242f3e'}]},
      {elementType: 'labels.text.fill', stylers: [{color: '#746855'}]},
      {
        featureType: 'administrative.locality',
        elementType: 'labels.text.fill',
        stylers: [{color: '#d59563'}]
      },
      {
        featureType: 'poi',
        elementType: 'labels.text.fill',
        stylers: [{color: '#d59563'}]
      },
      {
        featureType: 'poi.park',
        elementType: 'geometry',
        stylers: [{color: '#263c3f'}]
      },
      {
        featureType: 'poi.park',
        elementType: 'labels.text.fill',
        stylers: [{color: '#6b9a76'}]
      },
      {
        featureType: 'road',
        elementType: 'geometry',
        stylers: [{color: '#38414e'}]
      },
      {
        featureType: 'road',
        elementType: 'geometry.stroke',
        stylers: [{color: '#212a37'}]
      },
      {
        featureType: 'road',
        elementType: 'labels.text.fill',
        stylers: [{color: '#9ca5b3'}]
      },
      {
        featureType: 'road.highway',
        elementType: 'geometry',
        stylers: [{color: '#746855'}]
      },
      {
        featureType: 'road.highway',
        elementType: 'geometry.stroke',
        stylers: [{color: '#1f2835'}]
      },
      {
        featureType: 'road.highway',
        elementType: 'labels.text.fill',
        stylers: [{color: '#f3d19c'}]
      },
      {
        featureType: 'transit',
        elementType: 'geometry',
        stylers: [{color: '#2f3948'}]
      },
      {
        featureType: 'transit.station',
        elementType: 'labels.text.fill',
        stylers: [{color: '#d59563'}]
      },
      {
        featureType: 'water',
        elementType: 'geometry',
        stylers: [{color: '#17263c'}]
      },
      {
        featureType: 'water',
        elementType: 'labels.text.fill',
        stylers: [{color: '#515c6d'}]
      },
      {
        featureType: 'water',
        elementType: 'labels.text.stroke',
        stylers: [{color: '#17263c'}]
      }
    ]*/
  }
  map = new google.maps.Map(document.getElementById('map'), mapOptions);
  directionsRenderer.setMap(map);
  map.addListener('click', function(event) {placeInfoWindow(event.latLng);});
};



var infoWindow = null;
function placeInfoWindow(location)
{
  if (infoWindow == null)
  {
    infoWindow = new google.maps.InfoWindow
    ({
      content: location.lat() + ", " + location.lng(),
      position: new google.maps.LatLng(location.lat(), location.lng())
    });
    infoWindow.open(map);
  }
  else
  {
    var contentString = location.lat() + ", " + location.lng();
    infoWindow.setContent(contentString);
    infoWindow.setPosition(location);
  }
}

var visualizedMarkers = [];
var gpsPath = [];
function getPath()
{
  var start = new google.maps.LatLng({lat: Number(document.getElementById('fromLat').value), lng: Number(document.getElementById('fromLng').value)});
  var end = new google.maps.LatLng({lat: Number(document.getElementById('toLat').value), lng: Number(document.getElementById('toLng').value)});
  gpsPath = [];
  var request = {
    origin: start,
    destination: end,
    travelMode: 'BICYCLING',
    unitSystem: google.maps.UnitSystem.METRIC,
    provideRouteAlternatives: false
  };
  directionsService.route(request, function(result, status)
  {
    console.log(status)
    if (status == 'OK')
    {
      for (var i = 0; i < visualizedMarkers.length; i++)
      {
        visualizedMarkers[i].setMap(null);
      }
      visualizedMarkers = [];
      //directionsRenderer.setDirections(result);
      for (var i = 0; i < result.routes[0].overview_path.length; i++)
      {
        var marker = new google.maps.Marker
        ({
          position: result.routes[0].overview_path[i],
          label: i.toString(),
          map: map
        });
        gpsPath.push([result.routes[0].overview_path[i].lat(), result.routes[0].overview_path[i].lng()])
        visualizedMarkers.push(marker);
      }
    }
  });
}



/*function addMarker(location) {
  if (markers.length > 0)
  {
    markers[0].setMap(null);
    markers = [];
  }
  var singleMarker = new google.maps.Marker({
    position: location,
    map: map
  });
  singleMarker.info = new google.maps.InfoWindow({
    content: "sdfsdfs"
  });
  markers.push(singleMarker);
}*/
