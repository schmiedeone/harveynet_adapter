#!/usr/bin/env node
const server = require('http').Server();
const { Server } = require('socket.io');
const io = new Server(server, {
  cors: {
    origin: '*',
  },
});

const uuidv4 = require('uuid').v4;

class HarveyNetAdapter {
  constructor(start_socket = true, start_ros_node = false, debug_messages = []) {
    this.start_socket = start_socket;
    this.start_ros_node = start_ros_node;
    this.socket = 'NONE';
    this.node_handle = 'NONE';
    this.debug_messages = debug_messages;
    this.socketListeners = 'NONE';
    this.connectionPublisher = 'NONE';
    this.controlMessagePublisher = 'NONE';
  }

  registerSocketHandlers() {
    if (this.socket !== 'NONE') {
      this.registerControlMessageListener();
    }
  }

  registerControlMessageListener() {
    // When we receive a message... note, here we need to map the message to a harvey control message
    const classHandle = this;
    this.socket.on(`harvey-hmi-control`, function(data){
      // Object.keys(data)
      if (classHandle.debug_messages.includes('socket-messages')) {
        console.log(`harvey-hmi-control: %j`, data);
      }
      classHandle.handleControlMessage(data);
    });
  }

  registerROSHandlers() {
    if (this.node_handle !== 'NONE') {
      this.registerConnectionHandler();
      this.registerControlMessageHandler();
    }
  }

  registerConnectionHandler() {
    const std_msgs = rosnodejs.require('std_msgs');
    const boolMsg = std_msgs.msg.Bool;
    const connectPub = this.node_handle.advertise(`/harvey_controller/hmi_connected`, boolMsg);
    this.connectionPublisher = connectPub;
  }

  registerControlMessageHandler() {
    const hmi_controller = rosnodejs.require('hmi_controller');
    const harvey_hmi_msg = hmi_controller.msg.harvey_hmi_msg;
    const pub = this.node_handle.advertise(`/harvey_controller/hmi_controller`, harvey_hmi_msg);
    this.controlMessagePublisher = pub;
  }

  handleControlMessage(data) {
    console.log('data in handler', data);
    console.log('this.debug_messages', this.debug_messages);
    if (this.node_handle !== 'NONE' && this.controlMessagePublisher !== 'NONE') {
      const hmi_controller = rosnodejs.require('hmi_controller');
      const harvey_hmi_msg = hmi_controller.msg.harvey_hmi_msg;
      const msg = harvey_hmi_msg();

      if ('increment' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'increment');
        }
        this.controlMessagePublisher.publish(getIncrementMessage(msg, data['increment'][0], data['increment'][1]));
      }

      if ('decrement' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'decrement');
        }
        this.controlMessagePublisher.publish(getDecrementMessage(msg, data['decrement'][0], data['decrement'][1]));
      }

      if ('set' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'set');
        }
        this.controlMessagePublisher.publish(getSetMessage(msg, data['set'][0]));
      }

      if ('unset' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'unset');
        }
        this.controlMessagePublisher.publish(getUnsetMessage(msg, data['unset'][0]));
      }
    } else {
      console.warn(`Skipping control message publishing: \n node_handle: ${this.node_handle}  \n controlMessagePublisher: ${this.controlMessagePublisher}`);
    }
  }

  handleConnectionMessage(message) {
    if (this.node_handle !== 'NONE' && this.connectionPublisher !== 'NONE') {
      this.connectionPublisher.publish(message);
    }
  }

}

var Audit = require('entity-diff');

let requiredEnv = [
  'SOCKET_HOST'
];
// let unsetEnv = requiredEnv.filter((env) => !(typeof process.env[env] !== 'undefined'));

// if (unsetEnv.length > 0) {
//  throw new Error("Required ENV variables are not set: [" + unsetEnv.join(', ') + "]");
// }
const harveyNetAdapter = new HarveyNetAdapter(true, process.env.START_ROS === "true", ['action_types', 'socket-messages'])

const START_SOCKET = true;
const START_ROS = process.env.START_ROS === "true";

if (START_ROS) {
  const rosnodejs = require('rosnodejs');
  // init adapter node
  rosnodejs.initNode('/adapter', {onTheFly: true})
  .then(() => {
  	console.log('ros node is initiated');
  	const nh = rosnodejs.nh;
    harveyNetAdapter.node_handle = nh;
    harveyNetAdapter.registerROSHandlers();
  	// console.log('finished registering listeners');
    // console.log(harveyNetAdapter);
  });
}

var counter = 0;
while (START_ROS && harveyNetAdapter.node_handle !== 'NONE' && counter < 10) {
  setTimeout(function() {
    counter = counter +1;
    console.log('waiting');
  }, 200
  );
  if (counter === 10) {
    throw 'Timeout reached and node_handle not set';
  }
}

// Instantiate the socket and listen
io.on('connection', socket => {
  harveyNetAdapter.socket = socket;
  harveyNetAdapter.handleConnectionMessage({data: true});
  harveyNetAdapter.registerSocketHandlers();
  if (harveyNetAdapter.debug_messages.includes('classes')) {
    console.log(harveyNetAdapter);
  }
  socket.on("disconnect", (reason) => {
    harveyNetAdapter.socket = 'NONE';
    harveyNetAdapter.handleConnectionMessage({data: false});
    if (harveyNetAdapter.debug_messages.includes('classes')) {
      console.log(harveyNetAdapter);
    }
  });
});

// Instantiate the ROS stuff if needed

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
		default:
		throw `Channel - ${channel} - not recognized!`;
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
		default:
			throw `Channel - ${channel} - not recognized!`;
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
    case 'track_width_widen':
		return {
			...msg,
			...{'track_width': 1}
		};
		break;
    case 'track_width_narrow':
		return {
			...msg,
			...{'track_width': -1}
		};
		break;
		default:
		throw `Channel - ${channel} - not recognized!`;
		break;
	}
}
function getUnsetMessage(msg, channel) {
	switch (channel) {
		case 'engine_on':
    return {
			...msg,
			...{'engine_on': false}
		};
		break;
		case 'engine_off':
    return {
			...msg,
			...{'engine_off': false}
		};
		return {'engine_off': false};
		break;
		case 'joystick_mode':
    return {
			...msg,
			...{'joystick_mode': false}
		};
		break;
		case 'pallet_raise':
		case 'pallet_lower':
    return {
			...msg,
			...{'pallet_height': 0}
		};
    break;
		case 'pallet_angle_up':
		case 'pallet_angle_down':
    return {
			...msg,
			...{'pallet_tilt': 0}
		};
		break;
		case 'intake_raise':
		case 'intake_lower':
    return {
			...msg,
			...{'dammer_height': 0}
		};
		break;
    case 'track_width_widen':
		case 'track_width_narrow':
    return {
			...msg,
			...{'track_width': 0}
		};
		break;
		default:
		throw `Channel - ${channel} - not recognized!`;
		break;
	}
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

const port = process.env.PORT || 3001;
server.listen(port, () => {
  console.log('harvey-socket-io-server listening on port [3001]...');
});


// Removing this for now because it gets a bit too complicated otherwise
// function openConnectionErrorHandler(socket) {
//   socket.on('connect_error', (error) => {
//     console.log(error);
//     // server_pub.publish({data: false});
//   });
// }

// Mistakes I found:
// I need to use 'on the fly' mode so that I am no longer required to do 'catkin_make'
// I need to still test if this works without sourcing the workspace at all
// I cannot just send a partial message, I actually need to use the message constructor, and send the message class
