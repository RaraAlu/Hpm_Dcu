"""Strict YAML input and field validation shared by Profile tools."""
import re
from pathlib import Path

import yaml


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, expected, where):
    require(type(value) is dict, f"{where}: expected object")
    require(set(value) == set(expected), f"{where}: missing or unknown keys")


def uint(value, bits, where, minimum=0):
    require(type(value) is int and minimum <= value < (1 << bits),
            f"{where}: expected unsigned {bits}-bit integer >= {minimum}")


def identifier(value, where):
    require(type(value) is str and re.fullmatch(r"[a-z][a-z0-9_]{0,63}", value) is not None,
            f"{where}: invalid identifier")



class BenchLoader(yaml.SafeLoader):
    """A single explicit data tree, without YAML overrides or implicit numeric surprises."""

    def compose_node(self, parent, index):
        event = self.peek_event()
        require(not isinstance(event, yaml.AliasEvent) and event.anchor is None,
                "YAML anchors and aliases are not supported")
        return super().compose_node(parent, index)

    def construct_object(self, node, deep=False):
        prefix = "tag:yaml.org,2002:"
        require(node.tag in {prefix + kind for kind in ("map", "seq", "str", "int", "bool")},
                f"unsupported YAML type: {node.tag}")
        if node.tag in (prefix + "bool", prefix + "int"):
            require(isinstance(node, yaml.ScalarNode), "YAML numbers and booleans must be scalars")
        if node.tag == prefix + "bool":
            require(node.value in ("true", "false"), "YAML booleans must be true or false")
        elif node.tag == prefix + "int":
            require(re.fullmatch(r"(?:0|-?[1-9][0-9]*|0x[0-9a-fA-F]+)", node.value) is not None,
                    "YAML integers must be decimal or 0x hexadecimal, without leading zeros")
        return super().construct_object(node, deep=deep)

    def construct_mapping(self, node, deep=False):
        require(isinstance(node, yaml.MappingNode), "expected YAML mapping")
        result = {}
        for key_node, value_node in node.value:
            name = self.construct_object(key_node, deep=deep)
            require(type(name) is str, "YAML keys must be strings")
            require(name not in result, f"duplicate YAML key: {name}")
            result[name] = self.construct_object(value_node, deep=deep)
        return result


def load_yaml(path):
    try:
        return yaml.load(Path(path).read_text(encoding="utf-8"), Loader=BenchLoader)
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML: {error}") from error
