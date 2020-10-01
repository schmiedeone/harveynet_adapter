#!/usr/bin/env node

const rosnodejs = require('rosnodejs');
const uuidv4 = require('uuid').v4;

var ioClient = require("socket.io-client");

var Audit = require('entity-diff');

/*
This produces a list of changes like this: [{"key":"engine_off","from":0,"to":1}]
*/
function detectChanges(before, after) {
	const audit = new Audit.Audit();
	const result = audit.diff(before, after);
	console.log('result: %j', result);
	return result;
}

function packIntakeHeight(changeList, harveyMessageChange) {
	switch (harveyMessageChange.to) {
		case -1:
			return changeList + [{'intake_raise': false}, {'intake_lower': true}]
		case 1:
			return changeList + [{'intake_raise': true}, {'intake_lower': false}]
		default:
			return changeList + [{'intake_raise': false}, {'intake_lower': false}]
	}
}
function packPalletTilt(changeList, harveyMessageChange) {
	switch (harveyMessageChange.to) {
		case -1:
			return changeList + [{'pallet_angle_up': false}, {'pallet_angle_down': true}]
		case 1:
			return changeList + [{'pallet_angle_up': true}, {'pallet_angle_down': false}]
		default:
			return changeList + [{'pallet_angle_up': false}, {'pallet_angle_down': false}]
	}
}
function packPalletHeight(changeList, harveyMessageChange) {
	switch (harveyMessageChange.to) {
		case -1:
			return changeList + [{'pallet_raise': false}, {'pallet_lower': true}]
		case 1:
			return changeList + [{'pallet_raise': true}, {'pallet_lower': false}]
		default:
			return changeList + [{'pallet_raise': false}, {'pallet_lower': false}]
	}
}

const monitorToWebMap = {
	'joystick_mode': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'joystick_mode': harveyMessageChange.to}]
		}
	},
	'engine_on': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_on': harveyMessageChange.to}]
		}
	},
	'engine_off': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_off': harveyMessageChange.to}]
		}
	},
	'track_speed_max': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'track_speed_max': harveyMessageChange.to}]
		}
	},
	'front_belt_rpm': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'front_belt_rpm': harveyMessageChange.to}]
		}
	},
	'back_belt_rpm': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'back_belt_rpm': harveyMessageChange.to}]
		}
	},
	'dammer_height': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packIntakeHeight(changeList, harveyMessageChange);
		}
	},
	'pallet_height': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packPalletHeight(changeList, harveyMessageChange);
		}
	},
	'pallet_tilt': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packPalletTilt(changeList, harveyMessageChange);
		}
	},
	'header': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'hmi_connected': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'left_track_speed': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'right_track_speed': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}}
}

function packMonitorChanges(changes) {
	var socketMessages = changes.reduce(
		(accumulator, currentValue) => {
			monitorToWebMap[currentValue.key].addSocketMsg(accumulator, currentValue)
		}
		, []
	)

	return socketMessages;
}

const key_map = {
	'joystick_mode': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'joystick_mode': harveyMessageChange.to}]
		}
	},
	'engine_on': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_on': harveyMessageChange.to}]
		}
	},
	'engine_off': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_off': harveyMessageChange.to}]
		}
	},
	'track_speed_max': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'track_speed_max': harveyMessageChange.to}]
		}
	},
	'front_belt_rpm': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'front_belt_rpm': harveyMessageChange.to}]
		}
	},
	'back_belt_rpm': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'back_belt_rpm': harveyMessageChange.to}]
		}
	},
	'dammer_height': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packIntakeHeight(changeList, harveyMessageChange);
		}
	},
	'pallet_height': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packPalletHeight(changeList, harveyMessageChange);
		}
	},
	'pallet_tilt': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			packPalletTilt(changeList, harveyMessageChange);
		}
	},
	'header': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'hmi_connected': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'left_track_speed': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}},
	'right_track_speed': {'addSocketMsg': function (changeList, harveyMessageChange) {changeList}}
}
function openToolControlTopic(nh, socket) {
	const pub = nh.advertise(`/harvey_controller/hmi_controller`, 'harvey_can/harvey_joy_msg');
	// When we receive a message... note, here we need to map the message to a harvey control message
	socket.on(`harvey-hmi-control`, function(data){
			// Object.keys(data)
			console.log(`harvey-hmi-control: %j`, data);
			const msg = {};

			Object.keys(data)
			msg[data:] =
			pub.publish(msg);
	});
}

function initState(nh, socket) {

}

function openMonitorTopic(nh, socket) {
	nh.subscribe('/harvey_controller/set_state', 'harvey_can/harvey_joy_msg', newMsg => {
		nh.getParam('previous_state')
			.then((val) => {
  			changes = detectChanges(oldMsg, newMsg);
				nh.setParam('previous_state', msg);
				socket.emit(`harvey-hmi-monitor`, packMonitorChanges(changes));
			});
	});
}

function openDisconnectMonitorTopic(nh, socket) {
	const pub = nh.advertise(`/harvey_controller/hmi_disconnected`, 'std_msgs/Bool');
	socket.sockets.on('disconnect',function(){
			pub.publish(true);
	});
}

console.log('starting ros node');

// init adapter node
rosnodejs.initNode('/adapter')
	.then(() => {

		console.log('ros node is initiated');

		const nh = rosnodejs.nh;

		socket = ioClient('http://localhost:3000');

		socket.on('connect', () => {
		    // Display a connected message
				console.log("Connected to server");

				// you can disable a feature by commenting out correspondind line below
				// odomProcedure(nh);
				// cameraProcedure(nh);
				// //turtlebotMoveControlProcedure(nh);
				// //streamMoveControlProcedure(nh);
				// joystickControlProcedure(nh);
				// toolControlProcedure(nh, socket);
		});

		socket.on(`harvey-hmi-control`, function(data, socket){
				// Object.keys(data)
				console.log(`harvey-hmi-control: %j`, data);
				// const msg = { data: data.value };
				// pub.publish(msg);
		});

		socket.on("*",function(event,data) {
    	console.log(event);
    	console.log(data);
		});

		console.log('finished registering listeners');

// 		ocket.on('connect', function(){});
// socket.on('event', function(data){});
// socket.on('disconnect', function(){});


	});
