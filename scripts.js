// ============================= INTRODUCTION

// Name : Autonomous-Car-GPS-Guiding
// Author : Sylvain ARNOUTS
// Mail : sylvain.ar@hotmail.fr
// Date : From May to August 2016
// Link : https://github.com/sylvainar/Autonomous-Car-GPS-Guiding

// This code has been written in order to create a nice interface to interact
// with the autonomous electric car of the UPV ai2 laboratory. It displays a map,
// set a marker on the car's position using gps publishing topic, and allows a user 
// to start the routing by tapping the destination on the touchscreen.

// License : Apache 2.0

// ============================= CONFIGURATION

// You can set here some parameters for the launch.
// In the lab we're using two different GPS, so further in the
// program, we're going to read rosparam to know what topic
// we should listen to and what is the number of cycles
// between each refreshing.

// The name of the GPS publisher name by default
var CONFIG_default_gps_topic_name = '/gps';

// The number of cycles between every marker position reload
var CONFIG_cycles_number = 20;

// We can download the map online on OSM server, but
// it won't work if the car isn't connected to the internet.
// If you downloaded tiles and put it in the car, then you can
// access them in local, or else, connect to server.
// Set this config to "local" or "server".
var CONFIG_tile_source = 'server';

// If you use local tiles, set here the path to it
var CONFIG_tile_local_path = 'UPV/{z}/{x}/{y}.png';

// Network address to ROS server (it can be localhost or an IP)
var CONFIG_ROS_server_URI = 'localhost';


// ============================= FUNCTIONS

// ===> mapInit() : init the map
function mapInit() {
	
	//===> Var init

	// Fetch tiles
	if(CONFIG_tile_source == 'local')
		var tileUrl = CONFIG_tile_local_path;
	if(CONFIG_tile_source == 'server')
		var tileUrl = 'http://{s}.tile.osm.org/{z}/{x}/{y}.png';

	// Set attrib (always !)
	var attrib = 'Map data © OpenStreetMap contributors'; 

	//===> Map loading
	map = L.map('map');
	var osm = L.tileLayer(tileUrl, {
		minZoom: 12, 
		maxZoom: 18,
		attribution: attrib
	}); 
	osm.addTo(map);

	let polylineMeasure = L.control.polylineMeasure ({
		position:'topleft', unit:'metres', showBearings:false, 
		clearMeasurementsOnStop: false, showClearControl: true})
	polylineMeasure.addTo (map);

	L.easyButton('glyphicon-road', function(btn, map){
		swal({
			title: "Where do you want to go ?",
			text: "After closing this popup, click on the place you want to go.",
			type: "info",
			confirmButtonText: "Got it!",
			showCancelButton: true,
			closeOnConfirm: true,
			showLoaderOnConfirm: true,
			allowOutsideClick: false,
		},
		function(isConfirm){
			if (isConfirm) selectionMode = true;
			else selectionMode = false;
		});
	}).addTo(map);

	L.easyButton('glyphicon glyphicon-cog', function(btn, map){
		// TODO : add the possibility to modify params on the run
	}).addTo(map);

	L.easyButton('glyphicon glyphicon-refresh', function(btn, map){
		window.location.reload();
	}).addTo(map);

	markerFinish.addTo(map).setOpacity(0)

	return map;
}

// ============================= SCRIPT

//===> Global variables
var map;
var selectionMode;
var bounds;
var currentPosition = {latitude : 0, longitude : 0};
var startPoint;
var endPoint;
var markerPosition = L.marker([0,0]);
var markerFinish = L.marker([0,0]);
var zoomLevel = 16;
var routeControl;
var loadedMap = false;
var i = 0;
var listenerGPS;

//===> ROS connexion
var ros = new ROSLIB.Ros({
	url : 'ws://'+ CONFIG_ROS_server_URI +':9090'
});

swal({
	title: "Connecting to ROS...",
	showConfirmButton: false,
	closeOnConfirm: false,
	showLoaderOnConfirm: true,
	allowOutsideClick: false,
	allowEscapeKey: false
});

ros.on('connection', function() {
	console.log('Connected to websocket server.');
	swal({
		title: "Waiting...",
		text: "The navigation module can't work without the GPS. Launch the GPS and the module will start automatically.",
		type: "info",
		confirmButtonText: "Reload",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
	swal({
		title: "Error connecting the ROS server",
		text: "Unable to reach ROS server. Is rosbridge launched ?",
		type: "error",
		confirmButtonText: "Retry",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

ros.on('close', function() {
	console.log("Connexion closed.");
	swal({
		title: "Error connecting the ROS server",
		text: "Unable to reach ROS server. Is rosbridge launched ?",
		type: "error",
		confirmButtonText: "Retry",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

//===> Init the routing parameters
var paramStartLat = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/start/latitude'
});
var paramStartLon = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/start/longitude'
});
var paramEndLat = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/destination/latitude'
});
var paramEndLon = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/destination/longitude'
});
var paramEndGoTo = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/destination/goTo'
});
var paramServerType = new ROSLIB.Param({
	ros : ros,
	name : '/routing_machine/server_type'
});

paramStartLat.set(0);
paramStartLon.set(0);	
paramEndLat.set(0);
paramEndLon.set(0);
paramEndGoTo.set(false);

var publisherPath = new ROSLIB.Topic({
	ros : ros,
	name : '/routing_machine/global_waypoints',
	messageType : 'routing_machine/OutputCoords'
});

var path_ = [];
var polyline_;

//===> Init the map and the click listener

mapInit();

map.on('click', function(e) {
	//When a click on the map is detected
	if(selectionMode == true)
	{
		selectionMode = false;
		//First, get the coordinates of the point clicked
		var lat = e.latlng.lat;
		var lon = e.latlng.lng;
		//Place a marker
		markerFinish.setLatLng([lat,lon]);
		markerFinish.setOpacity(1);
		setTimeout(function() {
			swal({
				title: "Is this correct ?",
				text: "Confirm the position to start the navigation.",
				type: "info",
				confirmButtonText: "Yes, let's go !",
				showCancelButton: true,
				closeOnConfirm: true,
				allowOutsideClick: false,
			},
			function(isConfirm){
				if (isConfirm)
				{
						var server_type = 'server';
						//Logging stuff in the console
						console.log('Routing Start !');
						console.log('Start set to : '+ currentPosition.latitude + ' ' + currentPosition.longitude);
						console.log('Destination set to : '+lat + ' ' + lon);
						paramServerType.get(function(value) {
							console.log('Server type : '+value); // local or server (remote)
							server_type = value;
						});

						// Set all the parameters to the destination
						paramStartLat.set(currentPosition.latitude);
						paramStartLon.set(currentPosition.longitude);
						paramEndLat.set(lat);
						paramEndLon.set(lon);
						paramEndGoTo.set(true);// goTo is set to true, that means that their is a new destination to consider.

						// Remote server
						// if (server_type == 'server') {
							var getWpts = new ROSLIB.Service({
								ros: ros,
								name: '/routing_machine/get_wpts',
								serviceType: 'routing_machine/ParseWptsService'
							});

							console.log("Creating the service request (remote OSRM backend)");
							var request = new ROSLIB.ServiceRequest({
								get_wpts: true
							});

							console.log("Calling the service (remote OSRM backend)");
							getWpts.callService(request, function (response) {
								console.log('Result for service call on '
									+ getWpts.name + ' (remote OSRM backend): ' + response.num_wpts + ' waypoints');

								var statusResponse = response.success;
								var latResponse = response.latitude;
								var lonResponse = response.longitude;

								var active_polyline = L.featureGroup().addTo(map);

								//TODO: add removing old paths

								if (statusResponse) {
									path_ = [];

									var polyline = new L.Polyline([], {color: 'red'}, {weight: 1}).addTo(map);
									var polylinePoints = [];
									var indicator = false;

									for (var i = 0; i < response.num_wpts; i++) {
										polylinePoints.push([latResponse[i], lonResponse[i]]);
										polyline.addLatLng(L.latLng(latResponse[i], lonResponse[i]));
										indicator = true;
									}

									var pathMsg = new ROSLIB.Message({
										latitude: latResponse,
										longitude: lonResponse
									});

									publisherPath.publish(pathMsg);
									// console.log(polylinePoints);

									// if (indicator){
									// map.setView(polyline, zoomLevel);
									// }

									//polyline_ = L.polyline(path_, {color: 'red'}, {weight: 1}).addTo(map);
								}

							});
						// }
						// Local server
						// else if (server_type == 'local') {
							var getWptsLocal = new ROSLIB.Service({
								ros: ros,
								name: '/routing_machine/get_wpts_local',
								serviceType: 'routing_machine/RouteService'
							});

							var poseCurrent = new ROSLIB.Message({
								position : {
									y : currentPosition.latitude,
									x : currentPosition.longitude,
									z : 0.0
								},
								orientation : {
									x : 0.0,
									y : 0.0,
									z : 0.0,
									w : 1.0
								}
							});

							var poseGoal = new ROSLIB.Message({
								position : {
									y : lat,
									x : lon,
									z : 0.0
								},
								orientation : {
									x : 0.0,
									y : 0.0,
									z : 0.0,
									w : 1.0
								}
							});

							console.log("Creating the service request (local OSRM backend)");
							var requestLocal = new ROSLIB.ServiceRequest({
								get_wpts : true,
								waypoints : [poseCurrent, poseGoal],
								// radiuses :,
								// bearings :,
								// approaches :,
								// exclude :,
								steps : true,
								// continue_straight :,
								// annotation :,
								// overview :,
								number_of_alternatives : 0
							});

							console.log("Calling the service (local OSRM backend)");
							getWptsLocal.callService(requestLocal, function(response)
							{
								console.log('Result for service call on '
									+ getWptsLocal.name + ' (local OSRM backend)');

								var statusResponseLocal = response.success;
								var latResponseLocal = [];
								var lonResponseLocal = [];

								for (var route in response.routes) {
									console.log(response.routes[route].coordinates);

									for (var coordinate in response.routes[route].coordinates) {
										latResponseLocal.push(response.routes[route].coordinates[coordinate].y);
										lonResponseLocal.push(response.routes[route].coordinates[coordinate].x);
										var ll = L.latLng(response.routes[route].coordinates[coordinate].y, response.routes[route].coordinates[coordinate].x);
										var utm = ll.utm();
									}

									if (statusResponseLocal) {
										path_ = [];

										var polyline = new L.Polyline([], {color: 'red'}, {weight: 1}).addTo(map);
										var indicator = false;

										for (var coordinate in response.routes[route].coordinates) {
											polyline.addLatLng(L.latLng(response.routes[route].coordinates[coordinate].y, response.routes[route].coordinates[coordinate].x));
											indicator = true;
										}

										var pathMsg = new ROSLIB.Message({
											latitude: latResponseLocal,
											longitude: lonResponseLocal
										});

										publisherPath.publish(pathMsg);
									}
								}
							});
						// }
						// else {
						// 	console.log('WRONG SERVER TYPE!');
						// }
					}
					else
					{
						markerFinish.setOpacity(0);
					}
				})}, 1000);
	}
});

//===> Set the GPS listener

//  => Create param with initial value
var paramTopicNameValue = CONFIG_default_gps_topic_name;
var paramNbCyclesValue = CONFIG_cycles_number;

//  => Init the ROS param
var paramTopicName = new ROSLIB.Param({ros : ros, name : '/panel/gps_topic'});
var paramNbCycles = new ROSLIB.Param({ros : ros, name : '/panel/nb_cycles'});



//  => Set the value
paramTopicName.get(function(value) { 
	// If the param isn't created yet, we keep the default value
	if(value != null) 
		paramTopicNameValue = value; 
	else
		paramTopicName.set(paramTopicNameValue);
	
	paramNbCycles.get(function(value) { 
		// If the param isn't created yet, we keep the default value
		if(value != null) 
			paramNbCyclesValue = value; 
		else
		paramNbCycles.set(paramNbCyclesValue);


		// Set the listener informations
		listenerGPS = new ROSLIB.Topic({
			ros : ros,
			name : paramTopicNameValue,
			messageType : 'sensor_msgs/NavSatFix'
		});

		// Set the callback function when a message from /gps is received

		var i = 0;

		listenerGPS.subscribe(function(message) {
			// We have to wait for the GPS before showing the map, because we don't know where we are
			var lat = message.latitude;
			var lon = message.longitude;

			if(loadedMap == false) 
			{
				swal.close();
				// Center the map on the car's position
				map.setView([lat, lon], zoomLevel);
				// Add the marker on the map
				markerPosition.addTo(map);
				// Set the flag to true, so we don't have to load the map again
				loadedMap = true;
			}

			if(i % paramNbCyclesValue == 0)
			{
				// Refresh the global variable with the position
				currentPosition.latitude = lat;
				currentPosition.longitude = lon;
				// Refresh the position of the marker on the map
				markerPosition.setLatLng([lat, lon]);
				// If the marker has went out of the map, we move the map
				bounds = map.getBounds();
				if(!bounds.contains([lat, lon]))
					map.setView([lat, lon], zoomLevel);

				console.log("Update position");
			}

			i++;

		});
	});
});

