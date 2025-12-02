extends Node2D

@export var foot_height := 30.0
@export var inner_foot_width := 20.0
@export var outer_foot_width := 30.0

class Segment:
    var pos := Vector2.ZERO
    var len := 0.0

    func _init(pos: Vector2, len: float) -> void:
        self.pos = pos
        self.len = len

var segments: Array[Segment] = []
var base_pos := Vector2(320, 240)
var target_pos := Vector2.ZERO

func _ready() -> void:
    const sgm_cnt := 20
    for i in sgm_cnt:
        var norm := (i as float) / (sgm_cnt as float)
        var len := base_pos.distance_to(target_pos) / (sgm_cnt as float)
        var sgm := Segment.new(base_pos.lerp(target_pos, norm), len)
        segments.append(sgm)

func _process(_delta: float) -> void:
    # global_position = get_global_mouse_position()
    target_pos = get_global_mouse_position()
    
    # forward
    var next := segments[segments.size() - 1]
    next.pos = target_pos
    for i in range(segments.size() - 2, 0 - 1, -1):
        var curr := segments[i]
        var dir := (next.pos - curr.pos).normalized() * curr.len
        curr.pos = next.pos - dir
        next = curr
    
    # back
    var prev := segments[0]
    prev.pos = base_pos
    for i in range(1, segments.size(), 1):
        var curr := segments[i]
        var dir := (curr.pos - prev.pos).normalized() * prev.len
        curr.pos = prev.pos + dir
        prev = curr

    queue_redraw()

func _draw() -> void:
    const r := 7.0
    const o := 2.0
    # draw_circle(Vector2.ZERO, r + o, Color.WHITE)
    # draw_circle(Vector2.ZERO, r, Color.RED)

    const r1 := 3.0
    # # draw_circle(Vector2(inner_foot_width)
    # for l: Vector2 in get_leg_array():
    #     draw_circle(l, r1 + o, Color.WHITE)
    #     draw_circle(l, r1, Color.CYAN)

    for i in len(segments) - 1:
        draw_line(segments[i].pos, segments[i+1].pos, Color.GREEN, 2.0)

    const r2 := 3.0
    for segm in segments:
        draw_circle(segm.pos, r2 + o, Color.WHITE)
        draw_circle(segm.pos, r2, Color.RED)

        

func get_leg_array() -> Array[Vector2]:
    return [get_outer_left_leg(), get_inner_left_leg(), get_inner_right_leg(), get_outer_right_leg()]

func get_outer_left_leg() -> Vector2:
    return Vector2(outer_foot_width, foot_height)

func get_inner_left_leg() -> Vector2:
    return Vector2(inner_foot_width, foot_height)

func get_inner_right_leg() -> Vector2:
    return Vector2(-inner_foot_width, foot_height)

func get_outer_right_leg() -> Vector2:
    return Vector2(-outer_foot_width, foot_height)

# func get_physical_leg_swipe(target_pos: Vector2) -> Vector2:
#     pass