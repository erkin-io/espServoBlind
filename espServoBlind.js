const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

const tz_custom = {
    tuning_sliders: {
        key: ['servo_min_us', 'servo_max_us', 'move_speed_ms', 'idle_timeout_ms', 'invert_direction'],
        convertSet: async (entity, key, value, meta) => {
            const attrMap = {
                'servo_min_us': { id: 0x0001, type: 0x21 },
                'servo_max_us': { id: 0x0002, type: 0x21 },
                'move_speed_ms': { id: 0x0003, type: 0x21 },
                'idle_timeout_ms': { id: 0x0004, type: 0x21 }
            };

            const mapping = attrMap[key];
            const payload = {};

            payload[mapping.id] = { value: value, type: mapping.type };
            await entity.write(0xFF00, payload);

            return { state: { [key]: value } };
        },
    },

    identify_button: {
        key: ['identify_action'],
        convertSet: async (entity, key, value, meta) => {
            if (value === 'IDENTIFY') {
                await entity.command('genIdentify', 'identify', { identifytime: 3 }, meta.options);
            }
        },
    }
};

const definition = {
    zigbeeModel: ['espServoBlind'],
    model: 'espServoBlind',
    vendor: 'erkin.io',
    description: 'Universal Zigbee PWM servo controller with adjustable min/max pulse range, controllable as a 0–100% position slider from your smart home dashboard. Source code: https://github.com/erkin-io/espServoBlind',

    fromZigbee: [fz.cover_position_tilt],

    toZigbee: [
        tz.cover_state,
        tz.cover_position_tilt,
        tz_custom.tuning_sliders,
        tz_custom.identify_button
    ],

    exposes: [
        e.cover_position(),

        e.enum('identify_action', ea.SET, ['IDENTIFY']).withDescription('Blink the LED to identify the device'),

        exposes.numeric('servo_min_us', ea.SET)
            .withValueMin(500).withValueMax(2500)
            .withDescription('Calibration: Minimum Servo Pulse (μs)')
            .withCategory('config'),

        exposes.numeric('servo_max_us', ea.SET)
            .withValueMin(500).withValueMax(2500)
            .withDescription('Calibration: Maximum Servo Pulse (μs)')
            .withCategory('config'),

        exposes.numeric('move_speed_ms', ea.SET)
            .withValueMin(0).withValueMax(100)
            .withDescription('Speed: Milliseconds delay per 1% step (0 = Instant)')
            .withCategory('config'),

        exposes.numeric('idle_timeout_ms', ea.SET)
            .withValueMin(0).withValueMax(10000)
            .withDescription('Power: Cut PWM power after X idle milliseconds (0 = Never cut)')
            .withCategory('config'),
    ],

    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['closuresWindowCovering']);
        await reporting.currentPositionLiftPercentage(endpoint);
    },
};

module.exports = definition;
