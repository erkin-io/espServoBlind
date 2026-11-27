// Zigbee2MQTT external converter for espServoBlind (https://github.com/erkin-io/espServoBlind).
// For firmware 1.3.0 and later. It also works with older firmware: the feedback items then stay empty.
//
// Install: Zigbee2MQTT > Settings > Dev console > External converters, as "espServoBlind.mjs"
// (or copy the file to data/external_converters/). Remove the older "espServoBlind.js" first.
import * as fz from 'zigbee-herdsman-converters/converters/fromZigbee';
import * as tz from 'zigbee-herdsman-converters/converters/toZigbee';
import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';

const e = exposes.presets;
const ea = exposes.access;

const CUSTOM_CLUSTER = 0xFF00;
const ZCL_UINT8 = 0x20;
const ZCL_UINT16 = 0x21;

// Settings stored in the device (16 bit)
const tuningAttrs = {
    servo_min_us: 0x0001,
    servo_max_us: 0x0002,
    move_speed_ms: 0x0003,
    idle_timeout_ms: 0x0004,
    test_pulse_us: 0x0007,
};

const ATTR_FEEDBACK = 0x0005;
const ATTR_COMMAND = 0x0006;

// Buttons: each writes one command number
const commands = {
    calibrate: 1,
    set_open_position: 2,
    set_closed_position: 3,
};

const ATTR_RESET_REASON = 0x0010;
const ATTR_BOOT_COUNT = 0x0011;
const ATTR_CHIP_TEMP = 0x0012;
const ATTR_STATUS = 0x0013;
const ATTR_FEEDBACK_MV = 0x0014;
const ATTR_FEEDBACK_POSITION = 0x0015;
const calibrationAttrs = {
    calibration_low_us: 0x0016,
    calibration_low_mv: 0x0017,
    calibration_high_us: 0x0018,
    calibration_high_mv: 0x0019,
};

// Read-only values: key -> attribute, for "get" requests
const readOnlyAttrs = {
    last_reset_reason: ATTR_RESET_REASON,
    boot_count: ATTR_BOOT_COUNT,
    device_temperature: ATTR_CHIP_TEMP,
    status: ATTR_STATUS,
    feedback_voltage: ATTR_FEEDBACK_MV,
    feedback_position: ATTR_FEEDBACK_POSITION,
    ...calibrationAttrs,
};

const CHIP_TEMP_UNKNOWN = -32768;
const POSITION_UNKNOWN = 255;
const RESET_REASON_STORAGE_ERASED = 128;

// esp_reset_reason_t, in ESP-IDF order
const resetReasons = [
    'unknown', 'power_on', 'external_pin', 'software', 'crash', 'interrupt_watchdog', 'task_watchdog',
    'other_watchdog', 'deep_sleep', 'brownout', 'sdio', 'usb', 'jtag', 'efuse', 'power_glitch', 'cpu_lockup',
];

const statuses = ['ok', 'moving', 'blocked', 'no_feedback', 'moved_by_hand', 'calibrating', 'test'];

const FEEDBACK_NOTE = 'Needs a servo with a feedback wire.';

const attrValue = (data, id) => (data[id] !== undefined ? data[id] : data[`${id}`]);

// Everything the device stores or measures, in groups small enough for one Zigbee frame each
const readGroups = [
    [...Object.values(tuningAttrs), ATTR_FEEDBACK],
    [ATTR_RESET_REASON, ATTR_BOOT_COUNT, ATTR_CHIP_TEMP, ATTR_STATUS, ATTR_FEEDBACK_MV, ATTR_FEEDBACK_POSITION],
    Object.values(calibrationAttrs),
];

// Older firmware does not have all of these, and a device that is away can't answer: neither
// may stop the rest
const readAll = async (endpoint) => {
    try {
        await endpoint.read('closuresWindowCovering', ['currentPositionLiftPercentage']);
    } catch (error) { /* not reachable now */ }
    for (const group of readGroups) {
        try {
            await endpoint.read(CUSTOM_CLUSTER, group);
        } catch (error) { /* not reachable, or older firmware */ }
    }
};

const fzCustom = {
    // Turns reads and reports of the custom cluster into state, so the values shown are the
    // ones the device really uses
    espServoBlind: {
        cluster: `${CUSTOM_CLUSTER}`,
        type: ['readResponse', 'attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            for (const [key, id] of Object.entries({...tuningAttrs, ...calibrationAttrs})) {
                const value = attrValue(msg.data, id);
                if (value !== undefined) result[key] = value;
            }
            const feedback = attrValue(msg.data, ATTR_FEEDBACK);
            if (feedback !== undefined) result.feedback = feedback ? 'ON' : 'OFF';
            const status = attrValue(msg.data, ATTR_STATUS);
            if (status !== undefined) result.status = statuses[status] || `code_${status}`;
            const mv = attrValue(msg.data, ATTR_FEEDBACK_MV);
            if (mv !== undefined) result.feedback_voltage = Math.round(mv / 10) / 100;
            const position = attrValue(msg.data, ATTR_FEEDBACK_POSITION);
            if (position !== undefined) result.feedback_position = position === POSITION_UNKNOWN ? null : position;
            const reason = attrValue(msg.data, ATTR_RESET_REASON);
            if (reason !== undefined) {
                result.last_reset_reason = reason === RESET_REASON_STORAGE_ERASED ? 'storage_erased' : (resetReasons[reason] || `code_${reason}`);
            }
            const boots = attrValue(msg.data, ATTR_BOOT_COUNT);
            if (boots !== undefined) result.boot_count = boots;
            const temp = attrValue(msg.data, ATTR_CHIP_TEMP);
            if (temp !== undefined && temp !== CHIP_TEMP_UNKNOWN) result.device_temperature = Math.round(temp / 10) / 10;
            return result;
        },
    },
};

const tzCustom = {
    tuning: {
        key: Object.keys(tuningAttrs),
        convertSet: async (entity, key, value, meta) => {
            await entity.write(CUSTOM_CLUSTER, {[tuningAttrs[key]]: {value: value, type: ZCL_UINT16}});
            // The device answers with the value it saved (limited to its range)
            return {state: {[key]: value}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CUSTOM_CLUSTER, [tuningAttrs[key]]);
        },
    },

    feedback: {
        key: ['feedback'],
        convertSet: async (entity, key, value, meta) => {
            const on = `${value}`.toUpperCase() === 'ON' || value === true;
            await entity.write(CUSTOM_CLUSTER, {[ATTR_FEEDBACK]: {value: on ? 1 : 0, type: ZCL_UINT8}});
            return {state: {feedback: on ? 'ON' : 'OFF'}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CUSTOM_CLUSTER, [ATTR_FEEDBACK]);
        },
    },

    commands: {
        key: Object.keys(commands),
        convertSet: async (entity, key, value, meta) => {
            await entity.write(CUSTOM_CLUSTER, {[ATTR_COMMAND]: {value: commands[key], type: ZCL_UINT8}});
        },
    },

    readOnly: {
        key: Object.keys(readOnlyAttrs),
        convertGet: async (entity, key, meta) => {
            await entity.read(CUSTOM_CLUSTER, [readOnlyAttrs[key]]);
        },
    },

    identify: {
        key: ['identify_action'],
        convertSet: async (entity, key, value, meta) => {
            await entity.command('genIdentify', 'identify', {identifytime: 3}, meta.options);
        },
    },
};

export default {
    zigbeeModel: ['espServoBlind'],
    model: 'espServoBlind',
    vendor: 'erkin.io',
    description: 'Hardware Privacy Camera Cover & Universal Servo Blind. Code: https://github.com/erkin-io/espServoBlind',

    // Zigbee OTA updates (manufacturer 0x131B, image type 0x5342)
    ota: true,

    fromZigbee: [fz.cover_position_tilt, fzCustom.espServoBlind],

    toZigbee: [
        tz.cover_state,
        tz.cover_position_tilt,
        tzCustom.tuning,
        tzCustom.feedback,
        tzCustom.commands,
        tzCustom.readOnly,
        tzCustom.identify,
    ],

    exposes: [
        // ---- 1. Control ----
        e.cover_position(),

        // ---- 2. Status (read only) ----
        exposes.enum('status', ea.STATE_GET, statuses)
            .withLabel('Status')
            .withDescription('ok, moving, or a problem. With feedback: "blocked" = the cover did not reach the position and the motor ' +
                'was stopped; "moved_by_hand" = the position changed with the motor off; "no_feedback" = no signal on the feedback wire.')
            .withCategory('diagnostic'),

        exposes.numeric('feedback_position', ea.STATE_GET)
            .withUnit('%')
            .withLabel('Real position')
            .withDescription(`Position measured on the feedback wire. ${FEEDBACK_NOTE}`)
            .withCategory('diagnostic'),

        exposes.numeric('feedback_voltage', ea.STATE_GET)
            .withUnit('V')
            .withLabel('Feedback voltage')
            .withDescription(`Voltage on the feedback wire (GPIO 4). ${FEEDBACK_NOTE}`)
            .withCategory('diagnostic'),

        exposes.numeric('device_temperature', ea.STATE_GET)
            .withUnit('°C')
            .withLabel('Chip temperature')
            .withDescription('Temperature of the ESP32-C6 chip, updated every 5 minutes. Rough, runs above room temperature.')
            .withCategory('diagnostic'),

        exposes.text('last_reset_reason', ea.STATE_GET)
            .withLabel('Last restart reason')
            .withDescription('Why the device last restarted (power_on, brownout, crash, software, …). "storage_erased" = the saved ' +
                'settings were damaged and are back to defaults.')
            .withCategory('diagnostic'),

        exposes.numeric('boot_count', ea.STATE_GET)
            .withLabel('Restart count')
            .withDescription('Number of starts since the firmware was first flashed')
            .withCategory('diagnostic'),

        // ---- 3. Setup ----
        exposes.numeric('servo_min_us', ea.ALL)
            .withValueMin(500).withValueMax(2500).withUnit('µs')
            .withLabel('Open position (0 %)')
            .withDescription('Servo pulse at 0 %. Changing it does not move the servo; the next command uses it.')
            .withCategory('config'),

        exposes.numeric('servo_max_us', ea.ALL)
            .withValueMin(500).withValueMax(2500).withUnit('µs')
            .withLabel('Closed position (100 %)')
            .withDescription('Servo pulse at 100 %. Changing it does not move the servo; the next command uses it.')
            .withCategory('config'),

        exposes.numeric('move_speed_ms', ea.ALL)
            .withValueMin(0).withValueMax(1000).withUnit('ms')
            .withLabel('Speed')
            .withDescription('Milliseconds for each 1 % of travel. Larger = slower. 0 = the full speed of the servo.')
            .withCategory('config'),

        exposes.numeric('idle_timeout_ms', ea.ALL)
            .withValueMin(0).withValueMax(10000).withUnit('ms')
            .withLabel('Motor-off delay')
            .withDescription('The servo signal stops this long after the position is reached. 0 = hold forever. Minimum 500.')
            .withCategory('config'),

        exposes.binary('feedback', ea.ALL, 'ON', 'OFF')
            .withLabel('Position feedback')
            .withDescription(`Use the feedback wire of the servo: real position, "blocked" and "moved by hand" detection. ${FEEDBACK_NOTE} ` +
                'Leave OFF for a servo without it. Run "Calibrate" once after you switch it on.')
            .withCategory('config'),

        exposes.enum('calibrate', ea.SET, ['calibrate'])
            .withLabel('Calibrate')
            .withDescription(`The servo moves slowly to both ends of its travel and learns its feedback voltages. About 40 seconds. ${FEEDBACK_NOTE}`)
            .withCategory('config'),

        exposes.enum('set_open_position', ea.SET, ['set'])
            .withLabel('Set open position here')
            .withDescription(`Move the cover by hand to the open position, then press. Saves it as "Open position". ${FEEDBACK_NOTE}`)
            .withCategory('config'),

        exposes.enum('set_closed_position', ea.SET, ['set'])
            .withLabel('Set closed position here')
            .withDescription(`Move the cover by hand to the closed position, then press. Saves it as "Closed position". ${FEEDBACK_NOTE}`)
            .withCategory('config'),

        exposes.numeric('test_pulse_us', ea.ALL)
            .withValueMin(0).withValueMax(2600).withUnit('µs')
            .withLabel('Test pulse')
            .withDescription('For tests: drives the servo slowly to this pulse width and holds it (at most 5 minutes). 0 = motor off. ' +
                'The position in % is not valid during a test.')
            .withCategory('config'),

        exposes.numeric('calibration_low_us', ea.STATE_GET).withUnit('µs').withLabel('Calibration: low end')
            .withDescription('Shortest pulse the servo follows (from "Calibrate")').withCategory('diagnostic'),
        exposes.numeric('calibration_low_mv', ea.STATE_GET).withUnit('mV').withLabel('Calibration: low end voltage')
            .withDescription('Feedback voltage at the low end').withCategory('diagnostic'),
        exposes.numeric('calibration_high_us', ea.STATE_GET).withUnit('µs').withLabel('Calibration: high end')
            .withDescription('Longest pulse the servo follows (from "Calibrate")').withCategory('diagnostic'),
        exposes.numeric('calibration_high_mv', ea.STATE_GET).withUnit('mV').withLabel('Calibration: high end voltage')
            .withDescription('Feedback voltage at the high end').withCategory('diagnostic'),

        exposes.enum('identify_action', ea.SET, ['IDENTIFY'])
            .withLabel('Identify')
            .withDescription('Blink the LED to identify the device')
            .withCategory('config'),
    ],

    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['closuresWindowCovering']);
        await reporting.currentPositionLiftPercentage(endpoint);
        await readAll(endpoint);
    },

    // The device is asked for all its values when it comes (back) onto the network and when
    // Zigbee2MQTT starts, so nothing stays "unknown" until it happens to change.
    // Works with both forms of this hook: (event) in current Zigbee2MQTT, (type, data, device) before.
    onEvent: async (first, second, third) => {
        const type = typeof first === 'string' ? first : first?.type;
        const device = typeof first === 'string' ? third : first?.data?.device;
        if ((type !== 'deviceAnnounce' && type !== 'start') || !device) return;
        const endpoint = device.getEndpoint(1);
        if (endpoint) await readAll(endpoint);
    },
};
