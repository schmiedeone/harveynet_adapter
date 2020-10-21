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
	'increment': {
		'getHarveyMessage': function (changeList, harveyMessageChange) {
			changeList + [{'joystick_mode': harveyMessageChange.to}]
		}
	},
	'decrement': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_on': harveyMessageChange.to}]
		}
	},
	'set': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'engine_off': harveyMessageChange.to}]
		}
	},
	'unset': {
		'addSocketMsg': function (changeList, harveyMessageChange) {
			changeList + [{'track_speed_max': harveyMessageChange.to}]
		}
	},
}

function getIncrementMessage(msg, channel, value) {
	switch (channel) {
		case 'track_speed_max':
		return {
			...msg,
			...{'track_speed_max_increment': value}
		};
		break;
		case 'front_belt_rpm':
		return {
			...msg,
			...{'front_belt_rpm_increment': value}
		};
		break;
		case 'back_belt_rpm':
		return {
			...msg,
			...{'back_belt_rpm_increment': value}
		};
		break;
	}
}
function getDecrementMessage(msg, channel, value) {
	switch (channel) {
		case 'track_speed_max':
		return {
			...msg,
			...{'track_speed_max_increment': -1*value}
		};
		break;
		case 'front_belt_rpm':
		return {
			...msg,
			...{'front_belt_rpm_increment': -1*value}
		};
		break;
		case 'back_belt_rpm':
		return {
			...msg,
			...{'back_belt_rpm_increment': -1*value}
		};
		break;
	}
}
function getSetMessage(msg, channel) {
	switch (channel) {
		case 'engine_on':
		return {
			...msg,
			...{'engine_on': true}
		};
		break;
		case 'engine_off':
		return {
			...msg,
			...{'engine_off': true}
		};
		break;
		case 'joystick_mode':
		return {
			...msg,
			...{'joystick_mode': true}
		};
		break;
		case 'pallet_raise':
		return {
			...msg,
			...{'pallet_height': 1}
		};
		break;
		case 'pallet_lower':
		return {
			...msg,
			...{'pallet_height': -1}
		};
		break;
		case 'pallet_angle_up':
		return {
			...msg,
			...{'pallet_tilt': 1}
		};
		break;
		case 'pallet_angle_down':
		return {
			...msg,
			...{'pallet_tilt': -1}
		};
		break;
		case 'intake_raise':
		return {
			...msg,
			...{'dammer_height': 1}
		};
		break;
		case 'intake_lower':
		return {
			...msg,
			...{'dammer_height': -1}
		};
		break;
	}
}
function getUnsetMessage(msg, channel) {
	switch (channel) {
		case 'engine_on':
		return {'engine_on': false};
		break;
		case 'engine_off':
		return {'engine_off': false};
		break;
		case 'joystick_mode':
		return {'joystick_mode': false};
		break;
		case 'pallet_raise':
		case 'pallet_lower':
		return {'pallet_height': 0};
		break;
		case 'pallet_angle_up':
		case 'pallet_angle_down':
		return {'pallet_tilt': 0};
		break;
		case 'intake_raise':
		case 'intake_lower':
		return {'dammer_height': 0};
		break;
	}
}

function openToolControlTopic(nh, socket) {
	const harvey_can = rosnodejs.require('harvey_can');
	console.log('harvey_can fetched on the fly: ', harvey_can);
	const harvey_joy_msg = harvey_can.msg.harvey_joy_msg;
	console.log('harvey_joy_msg fetched on the fly: ', harvey_joy_msg);
	// const StringMsg = std_msgs.msg.String;
	const pub = nh.advertise(`/harvey_controller/hmi_controller`, harvey_joy_msg);

	// When we receive a message... note, here we need to map the message to a harvey control message
	socket.on(`harvey-hmi-control`, function(data){
		// Object.keys(data)
		console.log(`harvey-hmi-control: %j`, data);
		const msg = new harvey_joy_msg();

		if ('increment' in data) {
			pub.publish(getIncrementMessage(msg, data['increment'][0], data['increment'][1]));
		}

		if ('decrement' in data) {
			console.log(`harvey-hmi-control: %j`, 'decrement');
			pub.publish(getDecrementMessage(msg, data['decrement'][0], data['decrement'][1]));
		}

		if ('set' in data) {
			console.log(`harvey-hmi-control: %j`, 'set');
			pub.publish(getSetMessage(msg, data['set'][0]));
		}

		if ('unset' in data) {
			console.log(`harvey-hmi-control: %j`, 'unset');
			pub.publish(getUnsetMessage(msg, data['unset'][0]));
		}
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
rosnodejs.initNode('/adapter', {onTheFly: true})
.then(() => {

	console.log('ros node is initiated');

	const nh = rosnodejs.nh;

	socket = ioClient('http://localhost:3000');

	socket.on('connect', () => {
		// Display a connected message
		console.log("Connected to server");
		// const pub = nh.advertise(`/harvey_controller/hmi_controller`, 'harvey_can/harvey_joy_msg');

		// pub.publish({'engine_on': false})

		// you can disable a feature by commenting out correspondind line below
		// odomProcedure(nh);
		// cameraProcedure(nh);
		// //turtlebotMoveControlProcedure(nh);
		// //streamMoveControlProcedure(nh);
		// joystickControlProcedure(nh);
		openToolControlTopic(nh, socket);
	});

	// socket.on(`harvey-hmi-control`, function(data, socket){
	// 	const msg = new harvey_joy_msg({ joystick_mode: true });
	// 	console.log(`harvey-hmi-control: %j`, data);
	// 	pub.publish(msg);
	// });
	//
	// socket.on("*",function(event,data) {
	// 	console.log(event);
	// 	console.log(data);
	// });

	console.log('finished registering listeners');

	// 		ocket.on('connect', function(){});
	// socket.on('event', function(data){});
	// socket.on('disconnect', function(){});


});

// Mistakes I found:
// I need to use 'on the fly' mode so that I am no longer required to do 'catkin_make'
// I need to still test if this works without sourcing the workspace at all
// I cannot just send a partial message, I actually need to use the message constructor, and send the message class
