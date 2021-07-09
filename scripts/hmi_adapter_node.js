#!/usr/bin/env node
const server = require('http').Server();
const { Server } = require('socket.io');
const rosnodejs = require('rosnodejs');

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
    this.publishCounter = 1;
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
      this.registerStatusMessageHandler();
    }
  }

  registerConnectionHandler() {
    console.log("Registering Connection Handler");
    const std_msgs = rosnodejs.require('std_msgs');
    const boolMsg = std_msgs.msg.Bool;
    const connectPub = this.node_handle.advertise(`/harvey_controller/hmi_connected`, boolMsg);
    this.connectionPublisher = connectPub;
  }

  registerControlMessageHandler() {
    console.log("Registering Connection Control Message Handler");
    const harvey_controller = rosnodejs.require('harvey_controller');
    const harvey_hmi_msg = harvey_controller.msg.harvey_hmi_msg;
    const pub = this.node_handle.advertise(`/harvey_controller/hmi_controller`, harvey_hmi_msg);
    this.controlMessagePublisher = pub;
  }

  registerStatusMessageHandler() {
    console.log("Registering Connection Status Message Handler");
    this.node_handle.subscribe('/harvey_controller/hmi_controller_state', 'harvey_controller/hmi_controller_state', statusMessage => {
      this.handleStatusMessage(statusMessage)
    });
  }

  handleStatusMessage(statusMessage) {
    if(this.socket !== 'NONE') {
      this.socket.emit('harvey-hmi-monitor', packStatusMessage(statusMessage))
    } else {
      console.log("Skipping emit, socket is not defined");
    }
  }

  handleIncrementMessage(data) {
    switch (data['increment'][0]) {

  		case 'track_speed_max':

        const trackSpeedClient = harveyNetAdapter.node_handle.serviceClient(
          '/increment_track_speed',
          'harvey_controller/increment'
        );

        var result = trackSpeedClient.call({
          request_id: "one",
          increment_by: data['increment'][1],
          channel: data['increment'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "track_speed",
              "display_name": "Track Speed",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });

  		break;
  		case 'front_belt_rpm':

        const frontBeltClient = harveyNetAdapter.node_handle.serviceClient(
          '/increment_front_belt',
          'harvey_controller/increment'
        );

        var result = frontBeltClient.call({
          request_id: "one",
          increment_by: data['increment'][1],
          channel: data['increment'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "front_belt",
              "display_name": "Front Belt",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });
  		break;

  		case 'back_belt_rpm':

        const backBeltClient = harveyNetAdapter.node_handle.serviceClient(
          '/increment_front_belt',
          'harvey_controller/increment'
        );

        var result = backBeltClient.call({
          request_id: "one",
          increment_by: data['increment'][1],
          channel: data['increment'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "back_belt",
              "display_name": "Back Belt",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });
  		break;
  		default:
  		throw `Channel - ${channel} - not recognized!`;
  		break;
  	}
  }

  handleDecrementMessage(data) {
    switch (data['decrement'][0]) {

  		case 'track_speed_max':

        const trackSpeedClient = harveyNetAdapter.node_handle.serviceClient(
          '/decrement_track_speed',
          'harvey_controller/decrement'
        );

        var result = trackSpeedClient.call({
          request_id: "one",
          decrement_by: data['decrement'][1],
          channel: data['decrement'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "track_speed",
              "display_name": "Track Speed",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });

  		break;
  		case 'front_belt_rpm':

        const frontBeltClient = harveyNetAdapter.node_handle.serviceClient(
          '/decrement_front_belt',
          'harvey_controller/decrement'
        );

        var result = frontBeltClient.call({
          request_id: "one",
          decrement_by: data['decrement'][1],
          channel: data['decrement'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "front_belt",
              "display_name": "Front Belt",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });
  		break;

  		case 'back_belt_rpm':

        const backBeltClient = harveyNetAdapter.node_handle.serviceClient(
          '/decrement_front_belt',
          'harvey_controller/decrement'
        );

        var result = backBeltClient.call({
          request_id: "one",
          decrement_by: data['decrement'][1],
          channel: data['decrement'][0]
        })
        .then((result) => {
          this.socket.emit('harvey-hmi-monitor', {
            "machine_tools": [{
              "id": "back_belt",
              "display_name": "Back Belt",
              "type": "ANALOG",
              "target": result.new_value,
              "min_limit": 255,
              "max_limit": 0,
              "scaling_factor": (100/255),
              // "actual": 0, I think this is optional
              "unit": "%",
              "disabled": false
            }]
          })

        })
        .catch((error) => {
          console.error('error', error);
        });

  		break;
  		default:
  		throw `Channel - ${channel} - not recognized!`;
  		break;
  	}
  }

  handleSetTriState(data) {
    console.log("handleSetTriState");
    switch (data['set'][0]) {
      case 'default_automation':

        const autoMaticModeClient = harveyNetAdapter.node_handle.serviceClient(
          '/activate_default_automatic_mode',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "default_automation",
            "display_name": "Default Automation",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = autoMaticModeClient.call({
          request_id: "one",
          set_to: true,
          channel: "default_automation"
        })
        .then((result) => {
          console.log("result - handleSetTriState - default_automation : ", result);
          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "default_automation",
              "display_name": "Lane Hold Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });

  		break;

      case 'lane_hold_assist':

        const laneHoldClient = harveyNetAdapter.node_handle.serviceClient(
          '/activate_lane_hold_assist',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "lane_hold_assist",
            "display_name": "Lane Hold Assist",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = laneHoldClient.call({
          request_id: "one",
          set_to: true,
          channel: "lane_hold_assist"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "lane_hold_assist",
              "display_name": "Lane Hold Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });

  		break;
      case 'soil_flow_track_assist':

        const soilFlowTrackAssist = harveyNetAdapter.node_handle.serviceClient(
          '/activate_soil_flow_track_assist',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "soil_flow_track_assist",
            "display_name": "Soil-Flow Track Assist",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = soilFlowTrackAssist.call({
          request_id: "one",
          set_to: true,
          channel: "soil_flow_track_assist"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "soil_flow_track_assist",
              "display_name": "Soil-Flow Track Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });

  		break;
      case 'soil_flow_belt_assist':
        throw `Belt based soil flow control not implimented`;
  		break;
      case 'height_assist':
        const heightHoldClient = harveyNetAdapter.node_handle.serviceClient(
          '/activate_height_assist',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "height_assist",
            "display_name": "Height Assist",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = heightHoldClient.call({
          request_id: "one",
          set_to: true,
          channel: "height_assist"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "height_assist",
              "display_name": "Height Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
  		break;
      case 'speed_assist':
        throw `Speed assist not implimented`;
  		break;
      case 'hmi_active':

        console.log("activate HMI")

        const hmiClient = harveyNetAdapter.node_handle.serviceClient(
          '/activate_hmi',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "hmi_active",
            "display_name": "HMI",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = hmiClient.call({
          request_id: "one",
          set_to: true,
          channel: "hmi_active"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "hmi_active",
              "display_name": "HMI",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
      break;
      case 'machine_lights':
        const machineLightsClient = harveyNetAdapter.node_handle.serviceClient(
          '/activate_machine_lights',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "machine_lights",
            "display_name": "Machine Lights",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = machineLightsClient.call({
          request_id: "one",
          set_to: true,
          channel: "machine_lights"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "machine_lights",
              "display_name": "Machine Lights",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
      break;
      case 'machine_paused':
        const machinePause = harveyNetAdapter.node_handle.serviceClient(
          '/pause_machine',
          'harvey_controller/set_boolean'
        );

        var result = machinePause.call({
          request_id: "one",
          set_to: false,
          channel: "machine_paused"
        })
        .then((result) => {

          console.log("machine pausing done");

          this.socket.emit('harvey-hmi-monitor', {
            "machine_paused": result.new_value
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
  		break;
  		default:
  		throw `Channel - ${channel} - not a tri-state channel!`;
  		break;
  	}

  }

  handleUnsetTriState(data) {
    console.log("data", data);
    switch (data['unset'][0]) {

      case 'default_automation':

        const autoMaticModeClient = harveyNetAdapter.node_handle.serviceClient(
          '/deactivate_default_automatic_mode',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "default_automation",
            "display_name": "Default Automation",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = autoMaticModeClient.call({
          request_id: "one",
          set_to: true,
          channel: "default_automation"
        })
        .then((result) => {
          console.log("result - handleUnsetTriState - default_automation : ", result);
          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "default_automation",
              "display_name": "Default Automation",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });

      break;

      case 'lane_hold_assist':

        const laneHoldClient = harveyNetAdapter.node_handle.serviceClient(
          '/deactivate_lane_hold_assist',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "lane_hold_assist",
            "display_name": "Lane Hold Assist",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = laneHoldClient.call({
          request_id: "one",
          set_to: true,
          channel: "lane_hold_assist"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "lane_hold_assist",
              "display_name": "Lane Hold Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
      });

  		break;
      case 'soil_flow_track_assist':
        const soilFlowTrackAssist = harveyNetAdapter.node_handle.serviceClient(
          '/deactivate_soil_flow_track_assist',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "soil_flow_track_assist",
            "display_name": "Soil-Flow Track Assist",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = soilFlowTrackAssist.call({
          request_id: "one",
          set_to: true,
          channel: "soil_flow_track_assist"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "soil_flow_track_assist",
              "display_name": "Soil-Flow Track Assist",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
  		break;
      case 'soil_flow_belt_assist':
  		break;
      case 'height_assist':
      const heightHoldClient = harveyNetAdapter.node_handle.serviceClient(
        '/deactivate_height_assist',
        'harvey_controller/set_boolean'
      );

      this.socket.emit('harvey-hmi-monitor', {
        "toggle_modes": [{
          "id": "height_assist",
          "display_name": "Height Assist",
          "type": "TRI-STATE",
          "high_state": false,
          "low_state": false,
          "disabled": false
        }]
      })

      var result = heightHoldClient.call({
        request_id: "one",
        set_to: false,
        channel: "height_assist"
      })
      .then((result) => {

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "height_assist",
            "display_name": "Height Assist",
            "type": "TRI-STATE",
            "high_state": result.new_value === true,
            "low_state": result.new_value !== true,
            "disabled": false
          }]
        })

        if (result.status != 'SUCCEEDED') {
          // TODO send the error message to the front-end
        }

      })
      .catch((error) => { // TODO handle the errors
        console.error('error', error);
      });
  		break;
      case 'speed_assist':
  		break;
      case 'hmi_active':
        const hmiClient = harveyNetAdapter.node_handle.serviceClient(
          '/deactivate_hmi',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "hmi_active",
            "display_name": "HMI",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = hmiClient.call({
          request_id: "one",
          set_to: false,
          channel: "hmi_active"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "hmi_active",
              "display_name": "HMI",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
      break;
      case 'machine_lights':
        const machineLightsClient = harveyNetAdapter.node_handle.serviceClient(
          '/deactivate_machine_lights',
          'harvey_controller/set_boolean'
        );

        this.socket.emit('harvey-hmi-monitor', {
          "toggle_modes": [{
            "id": "machine_lights",
            "display_name": "Machine Lights",
            "type": "TRI-STATE",
            "high_state": false,
            "low_state": false,
            "disabled": false
          }]
        })

        var result = machineLightsClient.call({
          request_id: "one",
          set_to: false,
          channel: "machine_lights"
        })
        .then((result) => {

          this.socket.emit('harvey-hmi-monitor', {
            "toggle_modes": [{
              "id": "machine_lights",
              "display_name": "Machine Lights",
              "type": "TRI-STATE",
              "high_state": result.new_value === true,
              "low_state": result.new_value !== true,
              "disabled": false
            }]
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
  		break;
      case 'machine_paused':
        const machinePause = harveyNetAdapter.node_handle.serviceClient(
          '/unpause_machine',
          'harvey_controller/set_boolean'
        );

        console.log("cancelling pause state");

        var result = machinePause.call({
          request_id: "one",
          set_to: true,
          channel: "machine_paused"
        })
        .then((result) => {

          console.log("cancelled pause state");

          this.socket.emit('harvey-hmi-monitor', {
            "machine_paused": result.new_value
          })

          if (result.status != 'SUCCEEDED') {
            // TODO send the error message to the front-end
          }

        })
        .catch((error) => { // TODO handle the errors
          console.error('error', error);
        });
  		break;
  		default:
  		throw `Channel - ${channel} - not a tri-state channel!`;
  		break;
  	}
  }

  handleControlMessage(data) {
    // console.log('data in handler', data);
    // console.log('this.debug_messages', this.debug_messages);
    if (this.node_handle !== 'NONE' && this.controlMessagePublisher !== 'NONE') {
      const harvey_controller = rosnodejs.require('harvey_controller');
      const harvey_hmi_msg = harvey_controller.msg.harvey_hmi_msg;
      const msg = harvey_hmi_msg();
      const triStateModes = [
        'lane_hold_assist',
        'soil_flow_track_assist',
        'soil_flow_belt_assist',
        'height_assist',
        'speed_assist',
        'hmi_active',
        'machine_lights',
        'machine_paused',
        'default_automation'
      ];

      if ('increment' in data) {
        this.handleIncrementMessage(data);
      }

      if ('decrement' in data) {
        this.handleDecrementMessage(data);
      }

      if ('set' in data) {
        if (triStateModes.includes(data['set'][0])) {
          console.log("Using service");
          this.handleSetTriState(data);
        } else {
          console.log("data", data);
          console.log("Kicking it old-school");
          if (this.debug_messages.includes('action_types')) {
            console.log(`harvey-hmi-control: %j`, 'set');
          }
          this.controlMessagePublisher.publish(getSetMessage(msg, data['set'][0]));
        }
      }

      if ('unset' in data) {
        if (triStateModes.includes(data['unset'][0])) {
          this.handleUnsetTriState(data);
        } else {
          if (this.debug_messages.includes('action_types')) {
            console.log(`harvey-hmi-control: %j`, 'unset');
          }
          this.controlMessagePublisher.publish(getUnsetMessage(msg, data['unset'][0]));
        }
      }

      if ('load-file' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'unset');
        }
        this.controlMessagePublisher.publish(getLoadFileMessage(msg, data['load-file'][0]));
      }

      if ('save-file' in data) {
        if (this.debug_messages.includes('action_types')) {
          console.log(`harvey-hmi-control: %j`, 'unset');
        }
        this.controlMessagePublisher.publish(getSaveFileMessage(msg, data['save-file'][0]));
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
const START_ROS = (process.env.START_ROS === "true");
const harveyNetAdapter = new HarveyNetAdapter(true, START_ROS, ['action_types', 'socket-messages'])

const START_SOCKET = true;

if (START_ROS) {
  // init adapter node
  rosnodejs.initNode('/adapter', {onTheFly: true})
  .then(() => {
  	console.log('ros node is initiated');
  	const nh = rosnodejs.nh;
    harveyNetAdapter.node_handle = nh;
    harveyNetAdapter.registerROSHandlers();
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
  if (harveyNetAdapter.socket === 'NONE') {
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
  } else {
    console.log('Ignoring socket connection as one is already being assigned')
  }
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
    case 'lane_hold_assist':
    return {
			...msg,
			...{'toggle_lanefollow_mode': true}
		};
		break;
    case 'soil_flow_track_assist':
    return {
			...msg,
			...{'toggle_soilflow_track_control': true}
		};
		break;
    case 'soil_flow_belt_assist':
    return {
			...msg,
			...{'toggle_soilflow_belt_control': true}
		};
		break;
    case 'height_assist':
    return {
			...msg,
			...{'toggle_intake_height_control': true}
		};
		break;
    case 'speed_assist':
    return {
			...msg,
			...{'toggle_speed_control': true}
		};
		break;
    case 'hmi_active':
    return {
			...msg,
			...{'toggle_hmi_mode': true}
		};
    break;
    case 'machine_lights':
    return {
			...msg,
			...{'toggle_machine_lights': true}
		};
		break;
    case 'machine_paused':
    return {
			...msg,
			...{'pause_operation': true}
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
    case 'lane_hold_assist':
    return {
			...msg,
			...{'toggle_lanefollow_mode': true}
		};
		break;
    case 'soil_flow_track_assist':
    return {
			...msg,
			...{'toggle_soilflow_track_control': true}
		};
		break;
    case 'soil_flow_belt_assist':
    return {
			...msg,
			...{'toggle_soilflow_belt_control': true}
		};
		break;
    case 'height_assist':
    return {
			...msg,
			...{'toggle_intake_height_control': true}
		};
		break;
    case 'speed_assist':
    return {
			...msg,
			...{'toggle_speed_control': true}
		};
		break;
    case 'hmi_active':
    return {
			...msg,
			...{'toggle_hmi_mode': true}
		};
    break;
    case 'machine_lights':
    return {
			...msg,
			...{'toggle_machine_lights': true}
		};
		break;
    case 'machine_paused':
    return {
			...msg,
			...{'pause_operation': false}
		};
		break;
		default:
		throw `Channel - ${channel} - not recognized!`;
		break;
	}
}
function getLoadFileMessage(msg, name) {
  return {
    ...msg,
    ...{'load_config': true, 'config_filepath': name}
  };
}
function getSaveFileMessage(msg, name) {
  return {
    ...msg,
    ...{'save_config': true, 'config_filepath': name}
  };
}

function packStatusMessage(statusMessage) {
  console.log("statusMessage", statusMessage);
  return {
  "hmi_active": !statusMessage.hmi_locked,
  "machine_paused": statusMessage.machine_paused || false,
  "error_message": statusMessage.error_string,
  "warning_message": statusMessage.warn_string,
  "info_message": statusMessage.info_string, // TODO deal with empty strings
  "machine_tools": [
    // If you are ready to build it, you can actually hide items that are in the design but not in this list
    {
      "id": "track_speed",
      "display_name": "Track Speed",
      "type": "ANALOG",
      "target": statusMessage.target_track_speed,
      "min_limit": 255,
      "max_limit": 0,
      "scaling_factor": (100/255),
      // "actual": ((statusMessage.left_track_curpwm + statusMessage.right_track_curpwm) / 2),
      "unit": "%",
      "disabled": false
    },
    {
      "id": "back_belt",
      "display_name": "Sorting Belt",
      "type": "ANALOG",
      "target": statusMessage.target_belt_speed,
      "min_limit": 255,
      "max_limit": 0,
      "scaling_factor": (100/255),
      // "actual": statusMessage.back_belt_curpwm,
      "unit": "%",
      "disabled": false
    },
    {
      "id": "front_belt",
      "display_name": "Sieving Belt",
      "type": "ANALOG",
      "target": statusMessage.target_belt_speed,
      "min_limit": 255,
      "max_limit": 0,
      "scaling_factor": (100/255),
      // "actual": statusMessage.front_belt_curpwm,
      "unit": "%",
      "disabled": false
    },
    {
      "id": "track_width",
      "display_name": "Track Width",
      "type": "TRI-STATE",
      "high_state": (statusMessage.track_width_curpwm > 0),
      "low_state": (statusMessage.track_width_curpwm < 0),
      "disabled": false
    },
    {
      "id": "intake_height",
      "display_name": "Intake Height",
      "type": "TRI-STATE",
      "high_state": (statusMessage.dammer_height_curpwm > 0),
      "low_state": (statusMessage.dammer_height_curpwm < 0),
      "disabled": false
    },
    {
      "id": "pallet_height",
      "display_name": "Pallet Height",
      "type": "TRI-STATE",
      "high_state": (statusMessage.pallet_height_curpwm > 0),
      "low_state": (statusMessage.pallet_height_curpwm < 0),
      "disabled": false
    },
    {
      "id": "pallet_tilt",
      "display_name": "Pallet Tilt",
      "type": "TRI-STATE",
      "high_state": (statusMessage.pallet_tilt_curpwm > 0),
      "low_state": (statusMessage.pallet_tilt_curpwm < 0),
      "disabled": false
    }
  ],
  "home": [
    {
      "id": "machine_lights",
      "display_name": "Machine Lights",
      "type": "TRI-STATE",
      "high_state": (statusMessage.machine_lights_on === true),
      "low_state": (statusMessage.machine_lights_on !== true),
      "disabled": false
    }//,
    // {
    //   "id": "on_active",
    //   "display_name": "ON",
    //   // Note that in the future some of these tri-states have the potential
    //   // to become analog states like the ones above
    //   "type": "BOOLEAN",
    //   // This can be used to highlight the button or something to show that the
    //   // cylinders are moving
    //   "value": true | false
    // },
    // {
    //   "id": "off_active",
    //   "display_name": "OFF",
    //   // Note that in the future some of these tri-states have the potential
    //   // to become analog states like the ones above
    //   "type": "BOOLEAN",
    //   // This can be used to highlight the button or something to show that the
    //   // cylinders are moving
    //   "value": true | false
    // }
  ],
  "toggle_modes": [
    // If you are ready to build it, you can actually hide items that are in the design but not in this list
    {
      "id": "default_automation",
      "display_name": "Default Automation",
      "type": "TRI-STATE",
      "high_state": (statusMessage.default_automatic_mode === true),
      "low_state": (statusMessage.default_automatic_mode !== true),
      "disabled": false
    },
    {
      "id": "lane_hold_assist",
      "display_name": "Follow Lane",
      "type": "TRI-STATE",
      "high_state": (statusMessage.lane_follow_mode === true),
      "low_state": (statusMessage.lane_follow_mode !== true),
      "disabled": false
    },
    {
      "id": "soil_flow_track_assist",
      "display_name": "SFC - Track",
      "type": "TRI-STATE",
      // If both states are false consider it to be unknown and show a loading state
      "high_state": (statusMessage.soilflow_track_control === true),
      "low_state": (statusMessage.soilflow_track_control !== true),
      "disabled": false
    },
    {
      "id": "height_assist",
      "display_name": "Hold Height",
      "type": "TRI-STATE",
      // If both states are false consider it to be unknown and show a loading state
      "high_state": (statusMessage.intake_height_control === true),
      "low_state": (statusMessage.intake_height_control !== true),
      "disabled": false
    }
  ]
}
}

const port = process.env.PORT || 3001;
server.listen(port, () => {
  console.log(`harvey-socket-io-server listening on port [${port}]...`);
});
