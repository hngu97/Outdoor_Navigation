//Constructor for PoseStamped message.
function pathObj(asecs, ansecs, pX, pY, pZ, oX, oY, oZ, oW)
{
	this.header =
	{
		stamp:
	  {
		  secs: asecs,
		  nsecs: ansecs,
	  },
	  	frame_id : "GoogleMapJS",
  	};
	this.pose =
	{
		position:
		{
			x: pX,
			y: pY,
			z: pZ,
		},
  	orientation:
  	{
    	x: oX,
    	y: oY,
    	z: oZ,
    	w: oW,
  	},
	};
}
// We create a Topic object with details of the topic's name and message type.
let pathTopic = new ROSLIB.Topic({
  ros : ros,
  name : '/jsPath',
  messageType : 'nav_msgs/Path'
});

function pathPublisher()
{
  let pathArrObj = [];

  for (let i = 0; i < gpsPath.length; i++)
  {
  	let currentTime = new Date();
  	let secs = Math.floor(currentTime.getTime()/1000);
  	let nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
  	let pX = gpsPath[i][0];
  	let pY = gpsPath[i][1];
  	let currentpathObj = new pathObj(secs, nsecs, pX, pY, 0, 0, 0, 0, 1);
  	pathArrObj.push(currentpathObj);
  }

  let currentTime = new Date();
  let secs = Math.floor(currentTime.getTime()/1000);
  let nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
  let pathMes = new ROSLIB.Message(
    {
      header:
      {
        //seq:, //ROS handle this one
        stamp:
        {
          secs: secs,
          nsecs: nsecs,
        },
        frame_id: "GoogleMapJS",
      },
      poses: pathArrObj,
    });
  // And finally, publish.
  pathTopic.publish(pathMes);
}

function waypointsPublisher()
{
  let pathArrObj = [];

  for (let i = 0; i < waypointList.length; i++)
  {
  	let currentTime = new Date();
  	let secs = Math.floor(currentTime.getTime()/1000);
  	let nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
  	let pX = waypointList[i][0];
  	let pY = waypointList[i][1];
  	let currentpathObj = new pathObj(secs, nsecs, pX, pY, 0, 0, 0, 0, 1);
  	pathArrObj.push(currentpathObj);
  }

  let currentTime = new Date();
  let secs = Math.floor(currentTime.getTime()/1000);
  let nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
  let pathMes = new ROSLIB.Message(
    {
      header:
      {
        //seq:, //ROS handle this one
        stamp:
        {
          secs: secs,
          nsecs: nsecs,
        },
        frame_id: "GoogleMapJS",
      },
      poses: pathArrObj,
    });
  // And finally, publish.
  pathTopic.publish(pathMes);
}

let pathPubTimer = null;
function pathPubTimerHandle()
{
	clearInterval(waypointPubTimer);
	pathPubTimer = setInterval(pathPublisher, 500);
}

let waypointPubTimer = null;
function waypointsPubTimerHandle()
{
	clearInterval(pathPubTimer);
	waypointPubTimer = setInterval(waypointsPublisher, 500);
}

let waypointList = [];
let waypointMarker = [];
function addWaypoint()
{
	let listHTML = "";
	let plotCoordinates = [];
	waypointList.push([Number(document.getElementById('waypointLat').value), Number(document.getElementById('waypointLng').value)]);
	for (let i = 0; i < waypointList.length; i++)
	{
		listHTML += "<form id='waypointCSS'>";
		listHTML += "<input type='text' size='5' value='W" + (i+1) + "'" + "readonly>"
		listHTML += "<input type='text' size='17' value='" + waypointList[i][0] + "'" + "readonly>"
		listHTML += "<input type='text' size='17' value='" + waypointList[i][1] + "'" + "readonly>"
		listHTML += "</form>";
		plotCoordinates.push({lat: waypointList[i][0], lng: waypointList[i][1]})
		let marker = new google.maps.Marker
		({
			position: {lat: waypointList[i][0], lng: waypointList[i][1]},
			label: (i+1).toString(),
			map: map
		});
		waypointMarker.push(marker);
	}

	if (waypointList.length > 1)
	{
		let plotPath = new google.maps.Polyline({
			path: plotCoordinates,
			geodesic: true,
			strokeColor: '#FF0000',
			strokeOpacity: 1.0,
			strokeWeight: 2
		});
		plotPath.setMap(map);
	}
	document.getElementById("waypointList").innerHTML = listHTML;
}
