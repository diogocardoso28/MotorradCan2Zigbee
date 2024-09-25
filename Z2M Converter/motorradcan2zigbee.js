const {} = require('zigbee-herdsman-converters/lib/modernExtend');
const {commandsLevelCtrl} = require('zigbee-herdsman-converters/lib/modernExtend');

// Add the lines below
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const utils = require('zigbee-herdsman-converters/lib/utils');
const globalStore = require('zigbee-herdsman-converters/lib/store');
const e = exposes.presets;
const ea = exposes.access;


// Enum for Button Actions
const ButtonActions = {
    0: 'UP',
    1: 'LONG_UP',
    2: 'DOWN',
    3: 'LONG_DOWN',
    4: 'NONE',
};

// Function to bind to endpoint 10
const bind = async (endpoint, target, clusters) => {
    for (const cluster of clusters) {
        await endpoint.bind(cluster, target);
    }
};

// From Zigbee converter for handling button actions
const fzLocal = {
    motorrad_can_buttons: {
        cluster: 'genMultistateValue',
        type: ['readResponse', 'attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            // Read the button action from presentValue
            const clicks = msg.data['presentValue'];
            const action = ButtonActions[clicks] ? ButtonActions[clicks] : `UNKNOWN_${clicks}`;
            return {
                action: action,
            };
        },
    },
};

// Definition for Zigbee device
const definition = {
    zigbeeModel: ['MCANZIG'], // Zigbee model of the device
    model: 'MotorradCan2Zigbee', // Your device model
    vendor: 'Cardoso Tech', // Vendor name
    description: 'Motorrad Can 2 Zigbee Adapter', // Description
    fromZigbee: [fzLocal.motorrad_can_buttons], // Use the custom converter
    configure: async (device, coordinatorEndpoint, logger) => {
        // Bind the genMultistateValue cluster to endpoint 10
        const endpoint = device.getEndpoint(10);
        await bind(endpoint, coordinatorEndpoint, ['genMultistateValue']);
        await reporting.bind(endpoint, coordinatorEndpoint, ['genMultistateValue']); // Bind for reporting

        // // Setup reporting for the `presentValue` attribute of the genMultistateValue cluster
        // await reporting.presentValue(endpoint, {min: 1, max: 600, change: 1});
    },
    exposes: [
        e.action(['UP', 'LONG_UP', 'DOWN', 'LONG_DOWN', 'NONE']), // Exposes the available actions
    ],
    extend: [],
};


module.exports = definition;
