import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import CONF_CONCEPT2_ID, Concept2Component, concept2_ns

DEPENDENCIES = ["concept2"]

Concept2Switch = concept2_ns.class_("Concept2Switch", switch.Switch)

CONFIG_SCHEMA = switch.switch_schema(Concept2Switch).extend(
    {
        cv.GenerateID(CONF_CONCEPT2_ID): cv.use_id(Concept2Component),
    }
)


async def to_code(config):
    var = await switch.new_switch(config)
    parent = await cg.get_variable(config[CONF_CONCEPT2_ID])
    cg.add(var.set_parent(parent))
    cg.add(parent.set_active_switch(var))
