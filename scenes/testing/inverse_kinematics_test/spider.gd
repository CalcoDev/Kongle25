extends Node2D

@export var foot_height := 30.0
@export var inner_foot_width := 20.0
@export var outer_foot_width := 30.0

@export var coll_radius := 60.0
var base_pos := Vector2(320, 240)

@export var short_leg_length := 30.0
@export var long_leg_length := 30.0

@export var foot_step_height := 20.0

class Leg:
    var root := Vector2.ZERO

    var effector := Vector2.ZERO
    var effector_target := Vector2.ZERO
    var _last_effector_target := Vector2.ZERO
    var _prev_last_effector_target := Vector2.ZERO

    var joints: Array[Vector2] = []
    var _distance_contstraints: Array[float] = []
    # var _angle_constraints: Array[float] = []

    @warning_ignore("shadowed_variable")
    func _init(root: Vector2) -> void:
        self.root = root
    
    func fabrik() -> void:
        _fabrik_forward()
        _fabrik_backward()
    
    func _fabrik_forward() -> void:
        # var next := outer_left_leg_segments[outer_left_leg_segments.size() - 1]
        # next.pos = outer_left_leg_actual
        # for i in range(outer_left_leg_segments.size() - 2, 0 - 1, -1):
        #     var curr := outer_left_leg_segments[i]
        #     var dir := (next.pos - curr.pos).normalized() * curr.length
        #     curr.pos = next.pos - dir
        #     next = curr
        joints[joints.size() - 1] = effector
        for i in range(joints.size() - 2, 0 - 1, -1):
            var next := joints[i + 1]
            var curr := joints[i]

            var dir := (curr - next).normalized()
            var dist := _distance_contstraints[i]

            joints[i] = next + dir * dist

    
    func _fabrik_backward() -> void:
        # var prev := outer_left_leg_segments[0]
        # prev.pos = base_pos
        # for i in range(1, outer_left_leg_segments.size(), 1):
        #     var curr := outer_left_leg_segments[i]
        #     var dir := (curr.pos - prev.pos).normalized() * prev.length
        #     curr.pos = prev.pos + dir
        #     prev = curr
        joints[0] = root
        for i in range(1, joints.size()):
            var prev := joints[i - 1]
            var curr := joints[i]

            var dir := (curr - prev).normalized()
            var dist := _distance_contstraints[i - 1]

            joints[i] = prev + dir * dist
    
    func update_target_effector(max_dist: float) -> void:
        var dist := _last_effector_target.distance_squared_to(effector_target)
        if dist > max_dist * max_dist: # or _effector_target_time > max_effector_time:
            _prev_last_effector_target = _last_effector_target
            _last_effector_target = effector_target
        # if dist > max_dist * max_dist * 4.0:
            # effector = _last_effector_target
    
    func update_effector(delta: float, step_height: float, lerp_speed: float) -> void:
        var total_dist := absf(_prev_last_effector_target.x - _last_effector_target.x) * 1.0
        var curr_dist := absf(_prev_last_effector_target.x - effector.x)
        var progress := clampf(inverse_lerp(0, total_dist, curr_dist), 0.0, 1.0)
        if is_nan(progress):
            return
        var height_offset = sin(PI * progress) * step_height

        effector = effector.slerp(_last_effector_target + Vector2.UP * height_offset, delta * lerp_speed)
    
    func is_moving(tolerance: float = 0.05) -> bool:
        var mi := minf(_prev_last_effector_target.x, _last_effector_target.x)
        var ma := maxf(_prev_last_effector_target.x, _last_effector_target.x)
        if mi > effector.x or ma < effector.x:
            return true

        var total_dist := absf(_prev_last_effector_target.x - _last_effector_target.x) * 1.0
        var curr_dist := absf(_prev_last_effector_target.x - effector.x)
        var progress := clampf(inverse_lerp(0, total_dist, curr_dist), 0.0, 1.0)
        return not abs(progress - 1.0) < tolerance

    func draw(n: Node2D) -> void:
        _draw_outlined_circ(n, root, 2.0, Color.WHITE, 1.0, Color.BLACK)
        # _draw_outlined_circ(n, effector_target, 1.0, Color.GREEN)
        _draw_outlined_circ(n, _prev_last_effector_target, 1.0, Color.PURPLE)
        _draw_outlined_circ(n, _last_effector_target, 1.0, Color.RED)
        _draw_outlined_circ(n, effector, 1.0, Color.YELLOW)

        const LINE_COLOR := Color.RED
        const LINE_WIDTH := 2.0
        for i in range(0, joints.size() - 1):
            n.draw_line(-n.global_position + joints[i], -n.global_position + joints[i+1], LINE_COLOR, LINE_WIDTH)
    
    func _draw_outlined_circ(n: Node2D, p: Vector2, w: float, wc: Color, ow: float = 0.0, oc: Color = Color.BLACK) -> void:
        n.draw_circle(-n.global_position + p, w + ow, oc)
        n.draw_circle(-n.global_position + p, w, wc)

var legs: Dictionary[Leg, Array] = {}
var moving_section := -1
var _prev_moving_section := -1

func _ready() -> void:
    legs = {
        Leg.new(base_pos): [-outer_foot_width, 0],
        # Leg.new(base_pos): [-inner_foot_width, 1],
        # Leg.new(base_pos): [-inner_foot_width / 2, 2],
        # Leg.new(base_pos): [-inner_foot_width / 2, 3],
        # Leg.new(base_pos): [inner_foot_width, 0],
        Leg.new(base_pos): [outer_foot_width, 1],
    }

    var lk: Array[Leg] = legs.keys()
    lk[0].joints.append_array([base_pos, base_pos + Vector2(-outer_foot_width * 0.5, -10.0), base_pos + Vector2(-outer_foot_width, foot_height)])
    lk[0]._distance_contstraints.append_array([20, 30, -1])
    # lk[1].joints.append_array([base_pos, base_pos + Vector2(-inner_foot_width * 0.5, -10.0), base_pos + Vector2(-inner_foot_width, foot_height)])
    # lk[1]._distance_contstraints.append_array([10, 20, 30])
    # lk[2].joints.append_array([base_pos, base_pos + Vector2(inner_foot_width * 0.5, -10.0), base_pos + Vector2(inner_foot_width, foot_height)])
    # lk[2]._distance_contstraints.append_array([10, 20, 30])
    lk[1].joints.append_array([base_pos, base_pos + Vector2(outer_foot_width * 0.5, -10.0), base_pos + Vector2(outer_foot_width, foot_height)])
    lk[1]._distance_contstraints.append_array([20, 30, -1])

func _process(delta: float) -> void:
    base_pos = get_global_mouse_position()

    var reset_moving := true
    if moving_section != -1:
        for leg in legs:
            if legs[leg][1] == moving_section:
                if leg.is_moving(0.5):
                    reset_moving = false
                    break
    if reset_moving:
        _prev_moving_section = moving_section
        moving_section = -1

    # print(legs.keys()[0].joints)
    
    var llss: Array[Leg] = legs.keys().duplicate()
    llss.shuffle()
    for leg in llss:
        leg.root = base_pos

        var section: int = legs[leg][1]
        if moving_section == -1 and section != _prev_moving_section:
            moving_section = section

        if moving_section == section:
            leg.update_target_effector(20.0)
        
        leg.update_effector(delta, foot_step_height, 8.0)
        leg.fabrik()
        # leg

    queue_redraw()

func _physics_process(_delta: float) -> void:
    for leg in legs:
        var target = get_closest_collision_point_from_target(base_pos, base_pos + Vector2(legs[leg][0], foot_height), coll_radius)
        if target != Vector2.INF:
            leg.effector_target = target

func _draw() -> void:
    draw_circle(base_pos, coll_radius, Color8(0, 255, 255, 30))

    draw_circle(base_pos, 2.0 + 1.0, Color.BLACK)
    draw_circle(base_pos, 2.0, Color.WHITE)

    for leg in legs:
        leg.draw(self)

    # const r2 := 3.0
    # for i in outer_left_leg_segments.size() - 1:
    #     draw_line(outer_left_leg_segments[i].pos, outer_left_leg_segments[i+1].pos, Color.GREEN, 3.0)
    # for segm in outer_left_leg_segments:
    #     draw_circle(segm.pos, r2 + o, Color.WHITE)
    #     draw_circle(segm.pos, r2, Color.REBECCA_PURPLE)


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
