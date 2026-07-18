import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import esp32, esp32_ble, light
from esphome.const import CONF_ID, CONF_NAME, CONF_PORT
from esphome.core import CORE

CODEOWNERS = ["@ygelfand"]
# esp32_ble owns the BLE stack; esp32_ble_server provides the GATTS event-handler
# registry we hook into; mdns is needed for DirCon discovery; sensor is
# auto-loaded so the optional HA sensor platform is available.
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["esp32_ble", "esp32_ble_server", "mdns"]
MULTI_CONF = False

concept2_ns = cg.esphome_ns.namespace("concept2")
Concept2Component = concept2_ns.class_("Concept2Component", cg.PollingComponent)

CONF_CONCEPT2_ID = "concept2_id"
CONF_BLE = "ble"
CONF_DIRCON = "dircon"
CONF_ENABLED = "enabled"
CONF_STATUS_LIGHT = "status_light"
CONF_PAUSE_BUTTON = "pause_button"
CONF_VBUS_PIN = "vbus_pin"
CONF_SLEEP_TIMEOUT = "sleep_timeout"
CONF_AUTOSLEEP_ON_IDLE = "autosleep_on_idle"

DIRCON_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_PORT, default=36866): cv.port,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Concept2Component),
        cv.GenerateID(esp32_ble.CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
        cv.Optional(CONF_NAME, default="Concept2 Rower"): cv.string,
        cv.Optional(CONF_BLE, default=True): cv.boolean,
        cv.Optional(CONF_DIRCON, default={}): DIRCON_SCHEMA,
        cv.Optional(CONF_STATUS_LIGHT): cv.use_id(light.LightState),
        cv.Optional(CONF_PAUSE_BUTTON): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_VBUS_PIN): pins.gpio_output_pin_schema,
        cv.Optional(
            CONF_SLEEP_TIMEOUT, default="2min"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_AUTOSLEEP_ON_IDLE, default=True): cv.boolean,
    }
).extend(cv.polling_component_schema("100ms"))


def _validate_framework(config):
    if CORE.using_arduino:
        raise cv.Invalid(
            "The 'concept2' component requires the esp-idf framework "
            "(esp32: framework: type: esp-idf). It uses the ESP-IDF USB host stack."
        )
    return config


FINAL_VALIDATE_SCHEMA = _validate_framework


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_device_name(config[CONF_NAME]))
    cg.add(var.set_enable_ble(config[CONF_BLE]))

    if config[CONF_BLE]:
        # Hook esp32_ble's GATTS dispatcher: it will call
        # Concept2Component::gatts_event_handler() for every GATTS event.
        ble_parent = await cg.get_variable(config[esp32_ble.CONF_BLE_ID])
        esp32_ble.register_gatts_event_handler(ble_parent, var)

    dircon = config[CONF_DIRCON]
    cg.add(var.set_dircon_enabled(dircon[CONF_ENABLED]))
    cg.add(var.set_dircon_port(dircon[CONF_PORT]))

    cg.add(var.set_sleep_timeout(config[CONF_SLEEP_TIMEOUT]))
    cg.add(var.set_autosleep_on_idle(config[CONF_AUTOSLEEP_ON_IDLE]))

    if CONF_STATUS_LIGHT in config:
        status_light = await cg.get_variable(config[CONF_STATUS_LIGHT])
        cg.add(var.set_status_light(status_light))

    if CONF_PAUSE_BUTTON in config:
        button = await cg.gpio_pin_expression(config[CONF_PAUSE_BUTTON])
        cg.add(var.set_pause_button(button))

    if CONF_VBUS_PIN in config:
        vbus = await cg.gpio_pin_expression(config[CONF_VBUS_PIN])
        cg.add(var.set_vbus_pin(vbus))

    # The USB host stack lives in the built-in ESP-IDF `usb_host` component.
    # Newer ESPHome prunes unreferenced built-in IDF components; keep it in.
    if hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("usb_host")
