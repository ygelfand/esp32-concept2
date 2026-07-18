import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_POWER,
    ICON_HEART_PULSE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_EMPTY,
    UNIT_SECOND,
    UNIT_WATT,
)

from . import CONF_CONCEPT2_ID, Concept2Component

DEPENDENCIES = ["concept2"]

CONF_DISTANCE = "distance"
CONF_PACE = "pace"
CONF_POWER = "power"
CONF_STROKE_RATE = "stroke_rate"
CONF_STROKE_COUNT = "stroke_count"
CONF_HEART_RATE = "heart_rate"
CONF_CALORIES = "calories"
CONF_ELAPSED_TIME = "elapsed_time"
CONF_DRAG_FACTOR = "drag_factor"
CONF_FLYWHEEL_RPM = "flywheel_rpm"

# key -> (schema, C++ setter name)
SENSORS = {
    CONF_DISTANCE: (
        sensor.sensor_schema(
            unit_of_measurement="m",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        "set_distance_sensor",
    ),
    CONF_PACE: (
        sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            accuracy_decimals=0,
            icon="mdi:speedometer",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_pace_sensor",
    ),
    CONF_POWER: (
        sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_power_sensor",
    ),
    CONF_STROKE_RATE: (
        sensor.sensor_schema(
            unit_of_measurement="spm",
            accuracy_decimals=0,
            icon="mdi:rowing",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_stroke_rate_sensor",
    ),
    CONF_STROKE_COUNT: (
        sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:counter",
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        "set_stroke_count_sensor",
    ),
    CONF_HEART_RATE: (
        sensor.sensor_schema(
            unit_of_measurement="bpm",
            accuracy_decimals=0,
            icon=ICON_HEART_PULSE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_heart_rate_sensor",
    ),
    CONF_CALORIES: (
        sensor.sensor_schema(
            unit_of_measurement="kcal",
            accuracy_decimals=0,
            icon="mdi:fire",
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        "set_calories_sensor",
    ),
    CONF_ELAPSED_TIME: (
        sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_elapsed_time_sensor",
    ),
    CONF_DRAG_FACTOR: (
        sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            icon="mdi:water",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_drag_factor_sensor",
    ),
    CONF_FLYWHEEL_RPM: (
        sensor.sensor_schema(
            unit_of_measurement="rpm",
            accuracy_decimals=0,
            icon="mdi:fan",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        "set_flywheel_sensor",
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CONCEPT2_ID): cv.use_id(Concept2Component),
        **{cv.Optional(key): schema for key, (schema, _) in SENSORS.items()},
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CONCEPT2_ID])
    for key, (_, setter) in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(parent, setter)(sens))
