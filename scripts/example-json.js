

{

  "hmi_active": true | false, // maps to lock button

  "machine_paused": true | false,// maps to lock button

  "error_message": "Some message that can be displayed to the user" | null,

  "info_message": "Some message that can be displayed to the user" | null,

  "warning_message": "Some message that can be displayed to the user" | null,

  "machine_tools": [

    // If you are ready to build it, you can actually hide items that are in the design but not in this list

    {

      "id": "track_speed",

      "display_name": "Track Speed",

      "type": "ANALOG",

      "target": -1 to +1,

      "min_limit": -1 to +1,

      "max_limit": -1 to +1,

      // The normalized values need to be multiplied by the scaling factor to

      // get to the display value

      "scaling_factor": 0 to infinity,

      "actual": -1 to +1,

      // "%", "km/h", "m/s" - do not bind yourself to these, they will change for different tools

      "unit": "%",

      "disabled": true | false

    },

    {

      "id": "back_belt",

      "display_name": "Sorting Belt",

      "type": "ANALOG",

      "target": -1 to +1,

      "min_limit": -1 to +1,

      "max_limit": -1 to +1,

      // The normalized values need to be multiplied by the scaling factor to

      // get to the display value

      "scaling_factor": 0 to infinity,

      "actual": -1 to +1,

      // "%", "km/h", "m/s" - do not bind yourself to these, they will change for different tools

      "unit": "%",

      "disabled": true | false

    },

    {

      "id": "front_belt",

      "display_name": "Sieving Belt",

      "type": "ANALOG",

      "target": -1 to +1,

      "min_limit": -1 to +1,

      "max_limit": -1 to +1,

      // The normalized values need to be multiplied by the scaling factor to

      // get to the display value

      "scaling_factor": 0 to infinity,

      "actual": -1 to +1,

      // "%", "km/h", "m/s" - do not bind yourself to these, they will change for different tools

      "unit": "%",

      "disabled": true | false

    },

    {

      "id": "intake_height",

      "display_name": "Intake Height",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "TRI-STATE",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "intake_height",

      "display_name": "Intake Height",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "TRI-STATE",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "pallet_height",

      "display_name": "Pallet Height",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "TRI-STATE",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "intake_height",

      "display_name": "Pallet Tilt",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "TRI-STATE",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    }

  ],

  "home": [

    {

      "id": "on_active",

      "display_name": "ON",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "BOOLEAN",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "value": true | false

    },

    {

      "id": "off_active",

      "display_name": "OFF",

      // Note that in the future some of these tri-states have the potential

      // to become analog states like the ones above

      "type": "BOOLEAN",

      // This can be used to highlight the button or something to show that the

      // cylinders are moving

      "value": true | false

    }

  ]

  "toggle_modes": [

    // If you are ready to build it, you can actually hide items that are in the design but not in this list

    {

      "id": "lane_hold_assist",

      "display_name": "Follow Lane",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "soil_flow_belt_assist",

      "display_name": "SFC - Belt",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "soil_flow_track_assist",

      "display_name": "SFC - Track",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "height_assist",

      "display_name": "Hold Height",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    },

    {

      "id": "speed_assist",

      "display_name": "Hold Speed",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    }

    ,

    {

      "id": "machine_lights",

      "display_name": "Machine Lights",

      "type": "TRI-STATE",

      // If both states are false consider it to be unknown and show a loading state

      "high_state": true | false,

      "low_state": true | false,

      "disabled": true | false

    }

  ]

}
