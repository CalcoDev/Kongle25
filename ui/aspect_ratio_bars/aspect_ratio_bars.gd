# TODO(calco): Implement this lol
@tool
extends Control

@export var aspect_ratio_w: float = 4
@export var aspect_ratio_h: float = 3

@export_tool_button("Setup Aspect Ratio") var setup_ar_action := _setup_ar_action

func _setup_ar_action() -> void:
    # TODO(calco): Also do some editor buisness so we can CTRL-Z the things
    setup_ar()

func setup_ar() -> void:
    _left_container().size_flags_stretch_ratio = 1.0
    _right_container().size_flags_stretch_ratio = 1.0
    _center_container().size_flags_stretch_ratio = 1.0

func _left_container() -> CenterContainer:
    return $"HBoxContainer/Left"

func _center_container() -> CenterContainer:
    return $"HBoxContainer/Center"

func _right_container() -> CenterContainer:
    return $"HBoxContainer/Right"