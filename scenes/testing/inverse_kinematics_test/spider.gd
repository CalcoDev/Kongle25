extends Node2D

@export var foot_height := 30.0
@export var inner_foot_width := 20.0
@export var outer_foot_width := 30.0

class Segment:
    var pos := Vector2.ZERO
    var length := 0.0

    @warning_ignore("shadowed_variable")
    func _init(pos: Vector2, length: float) -> void:
        self.pos = pos
        self.length = length

@export var coll_radius := 60.0
var base_pos := Vector2(320, 240)

@export var short_leg_length := 30.0
@export var long_leg_length := 30.0

# var segments: Array[Segment] = []

var outer_left_leg_actual := Vector2.ZERO
var inner_left_leg_actual := Vector2.ZERO
var inner_right_leg_actual := Vector2.ZERO
var outer_right_leg_actual := Vector2.ZERO

var outer_left_leg_segments: Array[Segment] = []
var inner_left_leg_segments: Array[Segment] = []
var inner_right_leg_segments: Array[Segment] = []
var outer_right_leg_segments: Array[Segment] = []

func _ready() -> void:
    
    # var p := base_pos.lerp(get_outer_left_leg(), 0.5)
    outer_left_leg_segments.append(Segment.new(base_pos, short_leg_length))
    outer_left_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_outer_left_leg().x, 0.2), base_pos.y - 10.0), long_leg_length))
    outer_left_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_outer_left_leg().x, 0.6), base_pos.y - 20.0), long_leg_length))
    
    inner_left_leg_segments.append(Segment.new(base_pos, short_leg_length))
    inner_left_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_inner_left_leg().x, 0.2), base_pos.y - 10.0), long_leg_length))
    inner_left_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_inner_left_leg().x, 0.6), base_pos.y - 20.0), long_leg_length))
    
    inner_right_leg_segments.append(Segment.new(base_pos, short_leg_length))
    inner_right_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_inner_right_leg().x, 0.2), base_pos.y - 10.0), long_leg_length))
    inner_right_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_inner_right_leg().x, 0.6), base_pos.y - 20.0), long_leg_length))
    
    outer_right_leg_segments.append(Segment.new(base_pos, short_leg_length))
    outer_right_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_outer_right_leg().x, 0.2), base_pos.y - 10.0), long_leg_length))
    outer_right_leg_segments.append(Segment.new(Vector2(lerp(base_pos.x, get_outer_right_leg().x, 0.6), base_pos.y - 20.0), long_leg_length))

    # const sgm_cnt := 20
    # for i in sgm_cnt:
    #     var norm := (i as float) / (sgm_cnt as float)
    #     var length := base_pos.distance_to(target_pos) / (sgm_cnt as float)
    #     var sgm := Segment.new(base_pos.lerp(target_pos, norm), length)
    #     segments.append(sgm)
    pass


# var collided_colliders: Array[]

func _process(_delta: float) -> void:
    base_pos = get_global_mouse_position()

    # target_pos = get_global_mouse_position()
    # forward
    var next := outer_left_leg_segments[outer_left_leg_segments.size() - 1]
    next.pos = outer_left_leg_actual
    for i in range(outer_left_leg_segments.size() - 2, 0 - 1, -1):
        var curr := outer_left_leg_segments[i]
        var dir := (next.pos - curr.pos).normalized() * curr.length
        curr.pos = next.pos - dir
        next = curr
    # back
    var prev := outer_left_leg_segments[0]
    prev.pos = base_pos
    for i in range(1, outer_left_leg_segments.size(), 1):
        var curr := outer_left_leg_segments[i]
        var dir := (curr.pos - prev.pos).normalized() * prev.length
        curr.pos = prev.pos + dir
        prev = curr

    # target_pos = get_global_mouse_position()
    # forward
    next = inner_left_leg_segments[inner_left_leg_segments.size() - 1]
    next.pos = inner_left_leg_actual
    for i in range(inner_left_leg_segments.size() - 2, 0 - 1, -1):
        var curr := inner_left_leg_segments[i]
        var dir := (next.pos - curr.pos).normalized() * curr.length
        curr.pos = next.pos - dir
        next = curr
    # back
    prev = inner_left_leg_segments[0]
    prev.pos = base_pos
    for i in range(1, inner_left_leg_segments.size(), 1):
        var curr := inner_left_leg_segments[i]
        var dir := (curr.pos - prev.pos).normalized() * prev.length
        curr.pos = prev.pos + dir
        prev = curr

    # target_pos = get_global_mouse_position()
    # forward
    next = inner_right_leg_segments[inner_right_leg_segments.size() - 1]
    next.pos = inner_right_leg_actual
    for i in range(inner_right_leg_segments.size() - 2, 0 - 1, -1):
        var curr := inner_right_leg_segments[i]
        var dir := (next.pos - curr.pos).normalized() * curr.length
        curr.pos = next.pos - dir
        next = curr
    # back
    prev = inner_right_leg_segments[0]
    prev.pos = base_pos
    for i in range(1, inner_right_leg_segments.size(), 1):
        var curr := inner_right_leg_segments[i]
        var dir := (curr.pos - prev.pos).normalized() * prev.length
        curr.pos = prev.pos + dir
        prev = curr

    # target_pos = get_global_mouse_position()
    # forward
    next = outer_right_leg_segments[outer_right_leg_segments.size() - 1]
    next.pos = outer_right_leg_actual
    for i in range(outer_right_leg_segments.size() - 2, 0 - 1, -1):
        var curr := outer_right_leg_segments[i]
        var dir := (next.pos - curr.pos).normalized() * curr.length
        curr.pos = next.pos - dir
        next = curr
    # back
    prev = outer_right_leg_segments[0]
    prev.pos = base_pos
    for i in range(1, outer_right_leg_segments.size(), 1):
        var curr := outer_right_leg_segments[i]
        var dir := (curr.pos - prev.pos).normalized() * prev.length
        curr.pos = prev.pos + dir
        prev = curr

    queue_redraw()

func _physics_process(delta: float) -> void:
    var v := Vector2.ZERO

    delta *= 2.0
    const le_dist := 30.0
    
    v = get_closest_collision_point_from_target(base_pos, base_pos + get_outer_left_leg(), coll_radius)
    if v != Vector2.INF:
        outer_left_leg_actual = outer_left_leg_actual.lerp(v, delta)
        if outer_left_leg_actual.distance_to(v) >= le_dist and randi() % 4 == 0:
            outer_left_leg_actual = v
    v = get_closest_collision_point_from_target(base_pos, base_pos + get_inner_left_leg(), coll_radius)
    if v != Vector2.INF:
        inner_left_leg_actual = inner_left_leg_actual.lerp(v, delta)
        if inner_left_leg_actual.distance_to(v) >= le_dist and randi() % 4 == 1:
            inner_left_leg_actual = v
    v = get_closest_collision_point_from_target(base_pos, base_pos + get_inner_right_leg(), coll_radius)
    if v != Vector2.INF:
        inner_right_leg_actual = inner_right_leg_actual.lerp(v, delta)
        if inner_right_leg_actual.distance_to(v) >= le_dist and randi() % 4 == 2:
            inner_right_leg_actual = v
    v = get_closest_collision_point_from_target(base_pos, base_pos + get_outer_right_leg(), coll_radius)
    if v != Vector2.INF:
        outer_right_leg_actual = outer_right_leg_actual.lerp(v, delta)
        if outer_right_leg_actual.distance_to(v) >= le_dist and randi() % 4 == 3:
            outer_right_leg_actual = v

func _draw() -> void:
    draw_circle(base_pos, coll_radius, Color8(0, 255, 255, 30))

    const r := 7.0
    const o := 2.0
    draw_circle(base_pos, r + o, Color.WHITE)
    draw_circle(base_pos, r, Color.RED)

    const r1 := 3.0
    draw_circle(base_pos + get_outer_left_leg(), r1 + o, Color.WHITE)
    draw_circle(base_pos + get_outer_left_leg(), r1, Color.CYAN)
    # for l in get_leg_array():
    #     draw_circle(base_pos + l, r1 + o, Color.WHITE)
    #     draw_circle(base_pos + l, r1, Color.CYAN)
    
    draw_circle(-global_position + outer_left_leg_actual, 5.0, Color.ORANGE)
    # draw_circle(-global_position + inner_left_leg_actual, 5.0, Color.ORANGE)
    # draw_circle(-global_position + inner_right_leg_actual, 5.0, Color.ORANGE)
    # draw_circle(-global_position + outer_right_leg_actual, 5.0, Color.ORANGE)

    for i in outer_left_leg_segments.size() - 1:
        draw_line(outer_left_leg_segments[i].pos, outer_left_leg_segments[i+1].pos, Color.GREEN, 3.0)
    const r2 := 3.0
    for segm in outer_left_leg_segments:
        draw_circle(segm.pos, r2 + o, Color.WHITE)
        draw_circle(segm.pos, r2, Color.REBECCA_PURPLE)

    for i in inner_left_leg_segments.size() - 1:
        draw_line(inner_left_leg_segments[i].pos, inner_left_leg_segments[i+1].pos, Color.GREEN, 3.0)
    for segm in inner_left_leg_segments:
        draw_circle(segm.pos, r2 + o, Color.WHITE)
        draw_circle(segm.pos, r2, Color.REBECCA_PURPLE)

    for i in inner_right_leg_segments.size() - 1:
        draw_line(inner_right_leg_segments[i].pos, inner_right_leg_segments[i+1].pos, Color.GREEN, 3.0)
    for segm in inner_right_leg_segments:
        draw_circle(segm.pos, r2 + o, Color.WHITE)
        draw_circle(segm.pos, r2, Color.REBECCA_PURPLE)

    for i in outer_right_leg_segments.size() - 1:
        draw_line(outer_right_leg_segments[i].pos, outer_right_leg_segments[i+1].pos, Color.GREEN, 3.0)
    for segm in outer_right_leg_segments:
        draw_circle(segm.pos, r2 + o, Color.WHITE)
        draw_circle(segm.pos, r2, Color.REBECCA_PURPLE)


func get_leg_array() -> Array[Vector2]:
    return [get_outer_left_leg(), get_inner_left_leg(), get_inner_right_leg(), get_outer_right_leg()]

func get_outer_left_leg() -> Vector2:
    return Vector2(-outer_foot_width, foot_height)

func get_inner_left_leg() -> Vector2:
    return Vector2(-inner_foot_width, foot_height)

func get_inner_right_leg() -> Vector2:
    return Vector2(inner_foot_width, foot_height)

func get_outer_right_leg() -> Vector2:
    return Vector2(outer_foot_width, foot_height)

func get_closest_collision_point_from_target(base_position: Vector2, target_position: Vector2, radius: float, collision_mask: int = 0x7FFFFFFF) -> Vector2:
    var space := get_world_2d().direct_space_state

    var params := PhysicsShapeQueryParameters2D.new()
    var shape_rid := PhysicsServer2D.circle_shape_create()
    PhysicsServer2D.shape_set_data(shape_rid, radius)
    params.transform = Transform2D(0.0, base_position)
    params.shape_rid = shape_rid
    params.collision_mask = collision_mask

    var da := space.intersect_shape(params)
    PhysicsServer2D.free_rid(shape_rid)

    var min_dist := 999999999.9
    var min_pos := Vector2.INF # something base case I guess

    for r in da:
        var point := _closest_point_on_shape_local(r["rid"], r["shape"], target_position, true, true, base_pos)
        var dist := point.distance_squared_to(target_position)
        if dist < min_dist:
            min_dist = dist
            min_pos = point

    return min_pos


func _closest_point_on_shape_local(owner_rid: RID, shape_idx: int, local_point: Vector2, edge: bool, edge_los: bool, edge_los_point: Vector2) -> Vector2:
    var shape_rid := PhysicsServer2D.body_get_shape(owner_rid, shape_idx)
    var shape_type := PhysicsServer2D.shape_get_type(shape_rid)
    var shape_data = PhysicsServer2D.shape_get_data(shape_rid)

    var shape_transform := PhysicsServer2D.body_get_shape_transform(owner_rid, shape_idx)
    var body_transform := PhysicsServer2D.body_get_direct_state(owner_rid).transform
    var final_transform := body_transform * shape_transform
    
    match shape_type:
        # RectangleShape2D:
        #     var extents = shape.extents
        #     return Vector2(
        #         clampf(local_point.x, -extents.x, extents.x),
        #         clampf(local_point.y, -extents.y, extents.y)
        #     )
        # CircleShape2D:
        #     var r := (shape as CircleShape2D).radius
        #     var dist := local_point.lengthgth()
        #     if dist <= r or dist == 0.0:
        #         # Inside circle or at center → point itself is valid
        #         return local_point
        #     return local_point.normalized() * r
        # CapsuleShape2D:
        #     var r := (shape as CapsuleShape2D).radius
        #     var h := (shape as CapsuleShape2D).height * 0.5
        #     # Capsule axle is along Y in Godot
        #     var top_center := Vector2(0, -h)
        #     var bottom_center := Vector2(0, h)
        #     # Project onto vertical axis to see if we're in cylindrical region
        #     var y := clampf(local_point.y, -h, h)
        #     var cyl_center := Vector2(0, y)
        #     var v := local_point - cyl_center
        #     var d := v.lengthgth()
        #     if d <= r or d == 0.0:
        #         return local_point  # Inside capsule
        #     # Outside – clamp to top/bottom semicircles
        #     if local_point.y < -h:
        #         v = local_point - top_center
        #         d = v.lengthgth()
        #         return top_center + v.normalized() * r
        #     elif local_point.y > h:
        #         v = local_point - bottom_center
        #         d = v.lengthgth()
        #         return bottom_center + v.normalized() * r
        #     else:
        #         # Lateral surface
        #         return cyl_center + v.normalized() * r
        PhysicsServer2D.SHAPE_CONVEX_POLYGON:
            var pts: PackedVector2Array = shape_data.duplicate() # todo(calco): maybe dont dupplicate lol
            for i in pts.size():
                pts[i] = pts[i] * final_transform.affine_inverse()
            # 1) If point is inside, we can just return it
            # (Simple even-odd or sign test; here even-odd on x axis)
            if not edge:
                var inside := false
                var j := pts.size() - 1
                for i in pts.size():
                    var pi := pts[i]
                    var pj := pts[j]
                    var intersects := ((pi.y > local_point.y) != (pj.y > local_point.y)) \
                        and (local_point.x < (pj.x - pi.x) * (local_point.y - pi.y) / (pj.y - pi.y + 0.000001) + pi.x)
                    if intersects:
                        inside = not inside
                    j = i
                if inside:
                    return local_point
            # 2) Otherwise, closest on polygon perimeter
            var closest := pts[0]
            var best_dist_sq := INF
            for i in pts.size():
                var a := pts[i]
                var b := pts[(i + 1) % pts.size()]
                var candidate := Geometry2D.get_closest_point_to_segment(local_point, a, b)
                var d_sq := candidate.distance_squared_to(local_point)
                if d_sq < best_dist_sq:
                    if edge_los:
                        var dss := get_world_2d().direct_space_state
                        var params := PhysicsRayQueryParameters2D.new()
                        params.from = candidate + (edge_los_point - candidate).normalized() * 2.0
                        params.to = edge_los_point
                        params.hit_from_inside = true
                        var res := dss.intersect_ray(params)
                        if res:
                            continue
                    best_dist_sq = d_sq
                    closest = candidate
            return closest
        PhysicsServer2D.SHAPE_CONCAVE_POLYGON:
            # Treat each segment independently; similar to convex but no inside test
            var segments: PackedVector2Array = shape_data.duplicate()
            var closest := local_point
            var best_dist_sq := INF
            for i in range(0, segments.size(), 2):
                var a := segments[i]
                var b := segments[i + 1]
                var candidate := Geometry2D.get_closest_point_to_segment(local_point, a, b)
                var d_sq := candidate.distance_squared_to(local_point)
                if d_sq < best_dist_sq:
                    best_dist_sq = d_sq
                    closest = candidate
            return closest
        # SegmentShape2D:
        #     var a := (shape as SegmentShape2D).a
        #     var b := (shape as SegmentShape2D).b
        #     return Geometry2D.get_closest_point_to_segment(local_point, a, b)
        _:
            # Fallback – if we don't know, just return the point (may be inside)
            return local_point
