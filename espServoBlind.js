const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

const CUSTOM_CLUSTER = 0xFF00;
const ZCL_UINT16 = 0x21;

const tuningAttrs = {
    servo_min_us: 0x0001,
    servo_max_us: 0x0002,
    move_speed_ms: 0x0003,
    idle_timeout_ms: 0x0004,
};

const diagnosticAttrs = {
    last_reset_reason: 0x0010,
    boot_count: 0x0011,
    device_temperature: 0x0012,
};

const CHIP_TEMP_UNKNOWN = -32768;

// esp_reset_reason_t, in ESP-IDF order
const resetReasons = [
    'unknown', 'power_on', 'external_pin', 'software', 'crash', 'interrupt_watchdog', 'task_watchdog',
    'other_watchdog', 'deep_sleep', 'brownout', 'sdio', 'usb', 'jtag', 'efuse', 'power_glitch', 'cpu_lockup',
];

const attrValue = (data, id) => (data[id] !== undefined ? data[id] : data[`${id}`]);

const fz_custom = {
    // Turns reads/reports of the custom cluster into state, so HA shows the real values
    tuning: {
        cluster: `${CUSTOM_CLUSTER}`,
        type: ['readResponse', 'attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            for (const [key, id] of Object.entries(tuningAttrs)) {
                const value = attrValue(msg.data, id);
                if (value !== undefined) result[key] = value;
            }
            const reason = attrValue(msg.data, diagnosticAttrs.last_reset_reason);
            if (reason !== undefined) result.last_reset_reason = resetReasons[reason] || `code_${reason}`;
            const boots = attrValue(msg.data, diagnosticAttrs.boot_count);
            if (boots !== undefined) result.boot_count = boots;
            const temp = attrValue(msg.data, diagnosticAttrs.device_temperature);
            if (temp !== undefined && temp !== CHIP_TEMP_UNKNOWN) result.device_temperature = Math.round(temp / 10) / 10;
            return result;
        },
    },
};

const tz_custom = {
    tuning_sliders: {
        key: Object.keys(tuningAttrs),
        convertSet: async (entity, key, value, meta) => {
            await entity.write(CUSTOM_CLUSTER, { [tuningAttrs[key]]: { value: value, type: ZCL_UINT16 } });
            return { state: { [key]: value } };
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CUSTOM_CLUSTER, [tuningAttrs[key]]);
        },
    },

    diagnostics: {
        key: Object.keys(diagnosticAttrs),
        convertGet: async (entity, key, meta) => {
            await entity.read(CUSTOM_CLUSTER, [diagnosticAttrs[key]]);
        },
    },

    identify_button: {
        key: ['identify_action'],
        convertSet: async (entity, key, value, meta) => {
            if (value === 'IDENTIFY') {
                await entity.command('genIdentify', 'identify', { identifytime: 3 }, meta.options);
            }
        },
    },
};

const definition = {
    zigbeeModel: ['espServoBlind'],
    model: 'espServoBlind',
    vendor: 'erkin.io',
    description: 'Hardware Privacy Camera Cover & Universal Servo Blind. Code: https://github.com/erkin-io/espServoBlind',

    // Firmware 1.1.0+ accepts Zigbee OTA updates (manufacturer 0x131B, image type 0x5342)
    ota: true,

    fromZigbee: [fz.cover_position_tilt, fz_custom.tuning],

    toZigbee: [
        tz.cover_state,
        tz.cover_position_tilt,
        tz_custom.tuning_sliders,
        tz_custom.diagnostics,
        tz_custom.identify_button,
    ],

    exposes: [
        e.cover_position(),

        e.enum('identify_action', ea.SET, ['IDENTIFY']).withDescription('Blink the LED to identify the device'),

        exposes.numeric('servo_min_us', ea.ALL)
            .withValueMin(500).withValueMax(2500)
            .withDescription('Calibration: pulse (μs) at 0%. Changing it does not move the servo; the next command uses it.')
            .withCategory('config'),

        exposes.numeric('servo_max_us', ea.ALL)
            .withValueMin(500).withValueMax(2500)
            .withDescription('Calibration: pulse (μs) at 100%. Changing it does not move the servo; the next command uses it.')
            .withCategory('config'),

        exposes.numeric('move_speed_ms', ea.ALL)
            .withValueMin(0).withValueMax(100)
            .withDescription('Speed: milliseconds per 1% of travel (0 = servo full speed)')
            .withCategory('config'),

        exposes.numeric('idle_timeout_ms', ea.ALL)
            .withValueMin(0).withValueMax(10000)
            .withDescription('Power: cut the servo signal this long after reaching the position (0 = hold forever)')
            .withCategory('config'),

        exposes.text('last_reset_reason', ea.STATE_GET)
            .withDescription('Why the device last restarted (power_on, brownout, crash, task_watchdog, software, …)')
            .withCategory('diagnostic'),

        exposes.numeric('boot_count', ea.STATE_GET)
            .withDescription('Number of starts since the firmware was first flashed')
            .withCategory('diagnostic'),

        exposes.numeric('device_temperature', ea.STATE_GET)
            .withUnit('°C')
            .withDescription('Temperature of the ESP32-C6 chip (firmware 1.1.0+), updated every 5 minutes. Rough, runs above room temperature.')
            .withCategory('diagnostic'),
    ],

    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['closuresWindowCovering']);
        await reporting.currentPositionLiftPercentage(endpoint);
        await endpoint.read('closuresWindowCovering', ['currentPositionLiftPercentage']);
        await endpoint.read(CUSTOM_CLUSTER, Object.values(tuningAttrs));
        await endpoint.read(CUSTOM_CLUSTER, Object.values(diagnosticAttrs));
    },
};

module.exports = definition;
