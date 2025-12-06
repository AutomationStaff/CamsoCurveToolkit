# 	MIT License
#---------------------------------------------------------------------------------------------
# 	Copyright (c) 2025 Camshaft Software LLC
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE#  SOFTWARE.
#---------------------------------------------------------------------------------------------

bl_info = {
	"name": "Camso Curve Toolkit",
	"description": "Tools for building and editing curves",
	"author": "Sergey Arkhipov",
	"version": (1, 1, 3),
	"blender": (4, 5, 0),
	"location": "Sidebar -> Camso Curve Toolkit",
	"url": "https://github.com/AutomationStaff/CamsoCurveToolkit",
	"wiki_url": "https://github.com/AutomationStaff/CamsoCurveToolkit/wiki/",
	"category": "3D View"
}

import bpy
import bmesh
import os
from math import isclose, pi, cos, sin, radians
import mathutils.geometry
from mathutils import Vector, Matrix, Quaternion, Euler, Color, kdtree
import bpy_extras
from bpy_extras import view3d_utils
import gpu
from gpu_extras.batch import batch_for_shader
from bpy.utils import register_class, unregister_class
from enum import Enum
from bpy.types import Panel, Menu, Operator
import bpy.utils.previews

# CURVE OPS #####################################################################

class BT_BezierCurve:
	def __init__(self, arg):
		if isinstance(arg, bpy.types.Object) and arg.type == 'CURVE':
			curve = arg
			self.points = []
			src_points = curve.data.splines[0].bezier_points
			
			for point in src_points:
				self.points.append([
					to_world(self, curve.matrix_world, point.handle_left),
					to_world(self, curve.matrix_world, point.co),
					to_world(self, curve.matrix_world, point.handle_right)
					])

			points = curve.data.splines[0].bezier_points
			if len(points) == 2:
				self.p0_co            = to_world(self, curve.matrix_world, points[0].co)
				self.p0_handle_left   = to_world(self, curve.matrix_world, points[0].handle_left)
				self.p0_handle_right  = to_world(self, curve.matrix_world, points[0].handle_right)
				self.p1_co            = to_world(self, curve.matrix_world, points[1].co)
				self.p1_handle_left   = to_world(self, curve.matrix_world, points[1].handle_left)
				self.p1_handle_right  = to_world(self, curve.matrix_world, points[1].handle_right)

			# self.resolution = curve.data.splines[0].resolution_u
			# self.matrix_world = curve.matrix_world
			self.is_valid = True
		
		elif isinstance(arg, list):
			src_points = arg
			self.points = []            

			for point in src_points:
				self.points.append(point)

			# self.matrix_world = Matrix()
			# self.resolution = 12    
			self.is_valid = True

		elif isinstance(arg, tuple) and len(arg) == 6:
			points = arg
			
			self.points = [
				[
				 points[1],
				 points[0],
				 points[2]
				],
				[
				 points[4],
				 points[3],
				 points[5]
				]
			]

			self.p0_co           = points[0]
			self.p0_handle_left  = points[1]
			self.p0_handle_right = points[2]
			self.p1_co           = points[3]
			self.p1_handle_left  = points[4]
			self.p1_handle_right = points[5]

			# self.resolution = 12
			# self.matrix_world = Matrix()
			self.is_valid = True
		
		elif arg is None:
			self.points = []
			# self.resolution = 12
			# self.matrix_world = Matrix()
			self.is_valid = True

	def build(self, context, resolution, name, *, is_set_pivot=True):
		bezier = add_bezier(self, context, resolution, name)
		spline = bezier.data.splines[0]
		points = spline.bezier_points
		bezier.data.splines[0].bezier_points.add(len(self.points)-1)

		for index, point in enumerate(points):
			point.handle_left = self.points[index][0]
			point.co = self.points[index][1]
			point.handle_right = self.points[index][2]
		
		spline.resolution_u = resolution
		bezier.color = context.scene.bt_color		

		bezier.data.bevel_depth = context.scene.bt_pipe_radius
		bezier.data.bevel_resolution = context.scene.bt_pipe_resolution
		bezier.data.extrude = context.scene.bt_band_width	

		if is_set_pivot:
			set_pivot(bezier, bezier.matrix_world@points[0].co)
		# set_handle_type(self, bezier, 'ALIGNED')
		# bezier.matrix_world = self.matrix_world  
		return bezier

	is_valid = False
	points = []
	# matrix_world = Matrix()
	# resolution = 0

class BT_Cursor:
	def get_nearest_target_point_screen(self, cursor, screen_list):
		if len(screen_list) == 0:
			return Vector((0.0, 0.0))

		dist = dict()
		for point in screen_list:
			dist[get_distance(point, cursor)] = point

		if (len(dist)):
			return dist.get(min(dist))

		return Vector()

	def get_nearest_target_point_world(self, cursor, screen_world_map):
		if len(screen_world_map) == 0:
			return Vector() 
		dist = dict()
		for point in screen_world_map.keys():
			dist[get_distance(point, cursor)] = point

		if (len(dist)):
			return dist.get(min(dist))

		return Vector()

	def is_cusror_in_radius_range_from_nearest_point(self, cursor, point, radius):
		cursor_to_point_dist = get_distance(cursor, point)
		return cursor_to_point_dist <= radius

class BT_Draw(Operator, BT_Cursor):
	def __init__(self, *args, **kwargs):
		bpy.types.Operator.__init__(self, *args, **kwargs)
		self.points = []                
		self.target_handler = None # handler for drawing target points      
		self.control_point_handlers = []
		self.line_2d_handler = None # handler for drawing a screen bezier line
		self.curve_2d_handler =  None # handler for drawing a screen bezier curve
		self.snap_points = set() # all points that can be used for snapping
		self.snap_target = None 
		self.RADIUS = 25 # maximum distance between cursor and snap target
		self.radius = []
		self.rotation = Quaternion()
		self.matrix = Matrix()
		self.rotation_mode = 'EULER'
		self.keymap_strings = {	
			'LMB': '[LMB]: Add a new point    ',
			'LMB_CLICK_DRAG': '[LMB]+[CLICK_DRAG]: Move active point/handle    ',
			'CTRL_LMB': '[CTRL]+[LMB]: Snap to curves    ',
			'CTRL_SHIFT_LMB': '[CTRL]+[SHIFT]+[LMB]: Snap to mesh vertices    ',
			'SHIFT_LMB': '[SHIFT]+[LMB]: Snap to mesh surface    ',
			'TAB': '[TAB]: Reverse    ',
			'ENTER': '[ENTER]: Confirm and Exit'
			}

	def snap_to_verts(self, context, cursor): # vertex snap
		hit_result = scene_ray_cast(self, context, cursor)  
		ray_origin = get_view_vector_and_ray_origin(self, context, cursor)[1]
		
		if hit_result[0]:
			obj = hit_result[4]
			
			if obj.type != 'MESH':
				return None

			face = hit_result[3]                
			matrix = hit_result[5]
			location =  to_local(self, matrix, hit_result[1]) # Scene.ray_cast returns location in world coordinates, not in object space
			obj_eval = obj.evaluated_get(context.evaluated_depsgraph_get())         
			
			verts = obj_eval.data.vertices
			
			if not len(verts) > 0:
				return None
			
			kd_tree = kdtree.KDTree(len(verts))
			for index, vert in enumerate(verts):
				kd_tree.insert(vert.co.copy().freeze(), index)
			kd_tree.balance()
			
			nearest_vertex = matrix @ kd_tree.find(location)[0]
			return nearest_vertex

		return None 

	def remove_target_handler(self):
		remove_gpu_draw_handler(self, self.target_handler)
		self.target_handler = None

	def remove_control_point_handlers(self):
		for index, handler in enumerate(self.control_point_handlers):
			remove_gpu_draw_handler(self, handler)
			self.control_point_handlers[index] = None

	def remove_line_2d_handler(self):
		remove_gpu_draw_handler(self, self.line_2d_handler)
		self.line_2d_handler = None

	def remove_curve_2d_handler(self):
		remove_gpu_draw_handler(self, self.curve_2d_handler)
		self.curve_2d_handler = None

	def init_draw(self, context):
		wm = context.window_manager
		if wm.bt_modal_on != 'NONE':
			return False

		if not is_single_view3d(self, context):
			return False

		context.window.cursor_set('NONE')
		self.snap_points = snap_get_points(self, context)
		self.resolution = context.scene.bt_resolution
		km=self.keymap_strings
		context.workspace.status_text_set(km['LMB'] + km['CTRL_LMB'] + km['CTRL_SHIFT_LMB'] + km['SHIFT_LMB'] + km['ENTER'])

		return True

class BT_DrawBezierLine(BT_Draw):
	bl_idname = "curve.bt_draw_bezier_line"
	bl_label = 'Bézier Line'
	bl_description = "Build a new Bézier Line"
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):		
		wm = context.window_manager
		return wm.bt_modal_on != 'BT_LINE'

	def __init__(self, *args, **kwargs):
		BT_Draw.__init__(self, *args, **kwargs)

	def build_curve(self, context):
		curve = BT_BezierCurve([
			[self.points[0][1], self.points[0][1],  self.points[0][1]],
			[self.points[1][1], self.points[1][1],  self.points[1][1]]
			])

		bezier_line = curve.build(context, self.resolution, 'BézierLine')		
		set_handle_type(self, bezier_line, 'AUTO')
		set_handle_type(self, bezier_line, 'ALIGNED')

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not BT_Draw.init_draw(self, context):
			return{'CANCELLED'}

		context.window_manager.bt_modal_on = 'BT_LINE'
		context.window_manager.modal_handler_add(self)
		return{'RUNNING_MODAL'}

	def add_point(self, context, event):
		cursor = get_cursor(self, event)
		view_vector, ray_origin = get_view_vector_and_ray_origin(self, context, cursor)
		direction = view_vector.normalized()

		if event.ctrl: # snap to mesh verts
			if event.shift:				
				vert_co = self.snap_to_verts(context, cursor)

				if vert_co is not None:									
					target = vector_3d_to_screen(self, context, vert_co)
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
						self.points.append((target, vert_co))
						self.remove_target_handler()
						self.remove_line_2d_handler()
						self.target_handler = draw_target(self, context, target)
						if len(self.points) > 0:
							self.line_2d_handler = draw_2d_polyline(self, context, (self.points[0][0], target), ((0,1),))
						update_viewport(self, context)
						self.snap_points.add(vert_co.freeze())

			elif len(self.snap_points) > 0: # snap to curves and empties
				screen_world_map = get_screen_world_map(self, context, self.snap_points)	
				target = snap_get_target(self, context, cursor, screen_world_map)								
				if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
					nearest_world = screen_world_map.get(target)
					self.remove_target_handler()
					self.remove_line_2d_handler()
					self.target_handler = draw_target(self, context, target)

					if len(self.points) > 0:
						self.line_2d_handler = draw_2d_polyline(self, context, (self.points[0][0], target), ((0,1),))

					update_viewport(self, context)
					self.points.append((target, nearest_world))
					self.snap_points.add(nearest_world.freeze())

		elif event.shift:
			point = project(self, context, cursor, on_mesh=True)
			self.points.append(point)
			self.snap_points.add(point[1].freeze())
		else:
			point = project(self, context, cursor, on_mesh=False)
			self.points.append(point)
			self.snap_points.add(point[1].freeze())

	def modal(self, context, event):
		if len(self.points) == 0 and (event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE')):
			self.remove_target_handler()
			self.remove_line_2d_handler()
			update_viewport(self, context)
			return {'PASS_THROUGH'}

		elif event.type == 'MIDDLEMOUSE':
			self.remove_target_handler()
			self.remove_line_2d_handler()
			update_viewport(self, context)
			return {'PASS_THROUGH'}
		
		elif len(self.points) == 0 and event.type in ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
			self.remove_target_handler()
			self.remove_line_2d_handler()
			update_viewport(self, context)			
			return {'PASS_THROUGH'}

		# add point
		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			self.add_point(context, event)
			obj = context.object
			if len(self.points) == 2:
				if obj is not None and obj.select_get():
					context.object.select_set(False)
				self.build_curve(context)
				self.remove_target_handler()
				self.remove_line_2d_handler()
				update_viewport(self, context)
				self.points.clear()
				context.object.select_set(True)
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'
				return {'FINISHED'}

			return {'RUNNING_MODAL'}

		# update on mouse move
		elif event.type == 'MOUSEMOVE':     
			cursor = Vector((event.mouse_region_x, event.mouse_region_y))           
			if len(self.points) != 2:
				self.remove_target_handler()
				self.remove_line_2d_handler()
				update_viewport(self, context)
				self.target_handler = draw_target(self, context, cursor)
				if len(self.points) == 1:
					self.line_2d_handler = draw_2d_polyline(self, context, (self.points[0][0], cursor), ((0,1),))
			return {'RUNNING_MODAL'}

		# finish
		elif event.type in {'RET', 'ESC'} and event.value == 'PRESS':
			self.remove_target_handler()
			self.remove_line_2d_handler()
			update_viewport(self, context)
			self.points.clear()
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			return {'FINISHED'}		

		return {'RUNNING_MODAL'}

class BT_DrawBezierCurve(BT_Draw):
	bl_idname = "curve.bt_draw_bezier_curve"
	bl_label = 'Polyline'
	bl_description = "Build a new Polyline"
	bl_options = {'REGISTER', 'UNDO'}
	spline_type: bpy.props.StringProperty(default='BEZIER', options={'SKIP_SAVE'})

	curve = None
	new_spline = False
	active_tool = ''

	@classmethod
	def poll(cls, context):
		wm = context.window_manager
		return wm.bt_modal_on not in {'BT_CURVE', 'BT_POLYLINE'}

	def __init__(self, *args, **kwargs):
		BT_Draw.__init__(self, *args, **kwargs)
	
	def get_points(self, curve):
		return curve.data.splines[0].bezier_points if is_bezier(curve) else curve.data.splines[0].points

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not BT_Draw.init_draw(self, context):
			return{'CANCELLED'}

		if context.object is not None and context.object.type == 'CURVE' and context.mode == 'EDIT_CURVE' and len(self.get_points(context.object)) > 1:
			self.curve = context.object
			if self.curve.data.splines[0].type != self.spline_type:
				self.report({'ERROR'}, "Spline types don't match. Can't add " + self.spline_type + " points to a " + self.curve.data.splines[0].type + " spline.")
				return {'CANCELLED'}
		else:
			self.curve = self.add_curve(context, self.spline_type, 'BézierCurve' if self.spline_type == 'BEZIER' else 'Polyline')
			self.get_points(self.curve)[0].hide = True
			self.new_spline = True

		wm = context.window_manager		
		wm.bt_modal_on = 'BT_CURVE' if self.spline_type=='BEZIER' else 'BT_POLYLINE'
		
		bpy.ops.object.mode_set(mode='EDIT')
		
		if self.curve is None:
			self.report({'ERROR'}, 'Can\'t build a new curve' )
			return {"CANCELLED"}	

		self.curve.color = context.scene.bt_color
		context.window.cursor_modal_set('CROSS')

		context.window_manager.modal_handler_add(self)

		km=self.keymap_strings
		context.workspace.status_text_set(km['LMB'] + km['CTRL_LMB'] + km['CTRL_SHIFT_LMB'] + km['SHIFT_LMB'] + km['TAB'] + km['ENTER'])
			
		return {"RUNNING_MODAL"}

	def add_curve(self, context, spline_type, name):
		curve_data = bpy.data.curves.new(name=name, type='CURVE')
		curve_data.dimensions = '3D'
		spline = curve_data.splines.new(self.spline_type)
		
		spline.resolution_u = context.scene.bt_resolution
		
		curve = bpy.data.objects.new(name, curve_data)
		context.scene.collection.objects.link(curve)
		context.view_layer.objects.active = curve

		curve.data.bevel_depth = context.scene.bt_pipe_radius
		curve.data.bevel_resolution = context.scene.bt_pipe_resolution
		curve.data.extrude = context.scene.bt_band_width

		return curve

	def add_point(self, context, event):
		cursor = get_cursor(self, event)	   
		point = None

		if event.ctrl: # snap to mesh verts
			if event.shift:
				vert_co = self.snap_to_verts(context, cursor)
				if vert_co is not None:
					target = vector_3d_to_screen(self, context, vert_co)
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
						point = vert_co
		
			elif len(self.snap_points) > 0: # snap to curves and empties
				screen_world_map = get_screen_world_map(self, context, self.snap_points)
				if len(screen_world_map) > 0:   
					target = snap_get_target(self, context, cursor, screen_world_map).copy().freeze()
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
						nearest_world = screen_world_map.get(target)
						if nearest_world is not None:
							point = nearest_world

		elif event.shift:
			point = project(self, context, cursor, on_mesh=True)[1]
		
		else:
			point = project(self, context, cursor, on_mesh=False)[1]

		if point is not None:
			self.snap_points.add(point.freeze())
			point = to_local(self, self.curve.matrix_world, point)
			points = self.get_points(self.curve)
			if points:
				if self.spline_type == 'BEZIER':
					if self.new_spline:
						# to start, we just move the single point to the cursor position
						points[0].co = point
						points[0].handle_left = point
						points[0].handle_right = point
						points[0].hide = False
						self.new_spline = False						
						points[0].handle_right_type='VECTOR'
						points[0].handle_left_type='VECTOR'			

					else:
						# now we can add a new point
						points.add(1)
						points[-1].co = point						
						points[-1].handle_left_type = 'VECTOR'
						points[-1].handle_right_type = 'VECTOR'
					
				
				elif self.spline_type == 'POLY':
					if self.new_spline:                     
						points[0].co = point.to_4d()            
						points[0].hide = False
						self.new_spline = False						
					else:
						points.add(1)
						points[-1].co = point.to_4d()

	def modal(self, context, event):
		try:
			# if event.type not in {'TIMER_REPORT', 'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'}:
			# 	print(event.type, event.value)

			points = self.get_points(self.curve)
			cursor = get_cursor(self, event)
	 
			if event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE'):
				self.remove_line_2d_handler()
				update_viewport(self, context)
				return {'PASS_THROUGH'}

			elif event.type in ('MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
				self.remove_line_2d_handler()
				update_viewport(self, context)
				return {'PASS_THROUGH'}

			elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':			
				self.add_point(context, event)
				self.remove_line_2d_handler()
				update_viewport(self, context)
				return {'PASS_THROUGH'}

			elif event.type == 'MOUSEMOVE':
				if len(points) >= 1 and not self.new_spline:
					self.remove_line_2d_handler()
					self.line_2d_handler = draw_2d_polyline(self, context, (vector_3d_to_screen(self, context, self.curve.matrix_world@(points[-1].co)), cursor), ((0,1),))
					update_viewport(self, context)
				return {'RUNNING_MODAL'}

			elif event.type == 'TAB' and event.value == 'PRESS':
				self.remove_line_2d_handler()
				reverse_curve(self, self.curve)
				self.line_2d_handler = draw_2d_polyline(self, context, (vector_3d_to_screen(self, context, self.curve.matrix_world@(points[-1].co)), cursor), ((0,1),))
				update_viewport(self, context)

			elif event.type in {'RET', 'ESC'} and event.value == 'PRESS':				
				curve = self.curve
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'	
				context.window.cursor_modal_set('DEFAULT')
				update_object_edit_object(context)		
				if not len(self.get_points(curve)) > 1:
					bpy.data.curves.remove(curve.data)
					curve = None
					return {'FINISHED'}

				context.view_layer.objects.active = curve
				curve.select_set(True)
				set_pivot(curve, curve.matrix_world@self.get_points(curve)[0].co.xyz)
				self.remove_line_2d_handler()
				update_viewport(self, context)
				return {'FINISHED'}		
			
		except:
			self.remove_line_2d_handler()
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'	
			context.window.cursor_modal_set('DEFAULT')
			update_object_edit_object(context)
			update_viewport(self, context)	
			return {'CANCELLED'}

		return {'RUNNING_MODAL'}

class BT_DrawPolyBezier(BT_Draw):
	bl_idname = "curve.bt_draw_polybezier"
	bl_label = "Poly Bézier Curve"
	bl_description = "Poly Bézier Min is constructed by passing through 4 given points of a polyline. Poly Bézier Max building is continuous until finished"
	bl_options = {'REGISTER', 'UNDO'}
	to_bezier: bpy.props.IntProperty(min=0, max=2, default=0, options={'HIDDEN', 'SKIP_SAVE'})
	is_parabola: bpy.props.BoolProperty(options={'HIDDEN', 'SKIP_SAVE'})
	
	@classmethod
	def poll(cls, context):
		wm = context.window_manager
		return wm.bt_modal_on not in {'BT_POLYCURVE_PARABOLA', 'BT_POLYCURVE_MIN', 'BT_POLYCURVE_MAX'}
	
	def __init__(self, *args, **kwargs):
		BT_Draw.__init__(self, *args, **kwargs)
	
	def invoke(self, context, event):
		if bpy.ops.object.select_all.poll():
			bpy.ops.object.select_all(action='DESELECT')

		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not BT_Draw.init_draw(self, context):
			return{'CANCELLED'}
		
		wm = context.window_manager
		if self.to_bezier == 1:
			wm.bt_modal_on = 'BT_POLYCURVE_MIN'
		elif self.to_bezier == 2:
			wm.bt_modal_on = 'BT_POLYCURVE_MAX'
		elif self.is_parabola:
			wm.bt_modal_on = 'BT_POLYCURVE_PARABOLA'			

		context.window_manager.modal_handler_add(self)

		return{'RUNNING_MODAL'}	

	def draw_screen_polyline(self, context, cursor):
		points = [point[0] for point in self.points if point is not None]
		index_buffer = []

		if len(points) == 1:
			self.remove_curve_2d_handler()
			self.curve_2d_handler = draw_2d_polyline(self, context, [points[0], cursor], ((0,1),))

		elif len(points) > 1:
			i = 0
			while i != len(points):
				index_buffer.append((i, i + 1))
				i += 1 % len(points)

			self.remove_curve_2d_handler()
			points.append(cursor)
		
			self.curve_2d_handler = draw_2d_polyline(self, context, points, index_buffer)
	
	def add_point(self, context, event):
		cursor = get_cursor(self, event)
		view_vector, ray_origin = get_view_vector_and_ray_origin(self, context, cursor)
		direction = view_vector.normalized()		

		if event.ctrl: # snap to mesh verts
			if event.shift:
				vert_co = self.snap_to_verts(context, cursor)
				if vert_co is not None:
					target = vector_3d_to_screen(self, context, vert_co)
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):               
						self.points.append((target, vert_co))
						self.remove_target_handler()
						self.target_handler = draw_target(self, context, target)
						self.draw_screen_polyline(context, target)
						update_viewport(self, context)
						self.snap_points.add(vert_co.freeze())
		
			elif len(self.snap_points) > 0: # snap to curves and empties    
				screen_world_map = get_screen_world_map(self, context, self.snap_points)                
				if len(screen_world_map) > 0:  
					target = snap_get_target(self, context, cursor, screen_world_map).copy().freeze()
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
						nearest_world = screen_world_map.get(target)
						self.points.append((target, nearest_world))
						self.remove_target_handler()
						self.target_handler = draw_target(self, context, target)
						self.draw_screen_polyline(context, target)
						update_viewport(self, context)
						self.snap_points.add(nearest_world.freeze())
		
		elif event.shift:
			point = project(self, context, cursor, on_mesh=True)
			self.points.append(point)
			self.snap_points.add(point[1].freeze())		

		else:
			point = project(self, context, cursor, on_mesh=False)
			self.points.append(point)
			self.snap_points.add(point[1].freeze())

	def modal(self, context, event):
		if len(self.points) == 0 and (event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE')):
			return {'PASS_THROUGH'}

		elif event.type == 'MIDDLEMOUSE':
			return {'PASS_THROUGH'}

		elif len(self.points) == 0 and event.type in ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
			return {'PASS_THROUGH'}

		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			self.add_point(context, event)

			if self.is_parabola and len(self.points) == 3:
				self.remove_target_handler()
				self.remove_curve_2d_handler()
				update_viewport(self, context)
				add_polyline(self, context, [point[1] for point in self.points], 'Polyline')
				bpy.ops.object.bt_convert(keep_all_points=False, resolution=context.scene.bt_resolution, type='Bezier')						
				self.points.clear()
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'
				return {'FINISHED'}
			
			if len(self.points) == 4 and self.to_bezier == 1: # bulid a short polybezier and finish			
				self.remove_target_handler()
				self.remove_curve_2d_handler()
				update_viewport(self, context)
				add_polyline(self, context, [point[1] for point in self.points], 'Polyline')
				bpy.ops.object.bt_convert(keep_all_points=False, resolution=context.scene.bt_resolution, type='Bezier')
				self.points.clear()
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'
				return {'FINISHED'}

			return {'RUNNING_MODAL'}

		elif event.type == 'MOUSEMOVE':
			cursor = Vector((event.mouse_region_x, event.mouse_region_y))           
			self.remove_target_handler()                        
			self.target_handler = draw_target(self, context, cursor)
			self.draw_screen_polyline(context, cursor)
			return {'RUNNING_MODAL'}

		elif event.type in {'RET', 'ESC'} and event.value == 'PRESS':
			self.remove_target_handler()
			self.remove_curve_2d_handler()
			update_viewport(self, context)
			if len(self.points) > 2:
				add_polyline(self, context, [point[1] for point in self.points], 'Polyline')
				match(self.to_bezier):
					case 0:
						pass
					case 1:
						bpy.ops.object.bt_convert(keep_all_points=False, resolution=context.scene.bt_resolution, type='Bezier')
					case 2:
						bpy.ops.object.bt_convert(keep_all_points=True, resolution=context.scene.bt_resolution, type='Bezier', handle_type='ALIGNED')
			self.points.clear()
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			return {'FINISHED'}			

		return {'RUNNING_MODAL'}

class BT_DrawPolylineCircle(BT_Draw):
	bl_idname = "curve.bt_draw_polyline_circle"
	bl_label = "Polyline Circle"
	bl_description = "Build a new Polyline Circle"
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):
		wm = context.window_manager
		return wm.bt_modal_on != 'BT_POLYCIRCLE'

	def __init__(self, *args, **kwargs):
		BT_Draw.__init__(self, *args, **kwargs)	
	
	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not BT_Draw.init_draw(self, context):
			return{'CANCELLED'}
	
		context.window_manager.bt_modal_on = 'BT_POLYCIRCLE'
		context.window_manager.modal_handler_add(self)

		return{'RUNNING_MODAL'}

	def build_polyline_circle(self, context):
		if len(self.radius) == 2:           
			radius = self.radius[0][0] - self.radius[1][0]

			matrix = Matrix.Rotation(radians(360/self.resolution), 2)
			for i in range(0, self.resolution+1):
				radius.rotate(matrix)
				self.points.append(project(self, context, self.radius[0][0] - radius))

			polyline_circle = add_polyline(self, context, [point[1] for point in self.points], 'PolylineCircle', is_closed=True)			
			polyline_circle.location = self.radius[0][1]
			polyline_circle.color = context.scene.bt_color
			
			return set(point[1].copy().freeze() for point in self.points)
		
		else:
			return set()

	def draw_screen_polyline_circle(self, context, cursor):
		if len(self.radius) > 0:
			points = []
			
			radius = self.radius[0][0].copy() - cursor
			matrix = Matrix.Rotation(radians(360/self.resolution), 2)
			
			if matrix is not None:
				for i in range(0, self.resolution+1):
					radius.rotate(matrix)
					points.append(
							self.radius[0][0] - radius
						)

				index_buffer = []
				i = 0               
				while i != len(points):             
					index_buffer.append((i, i + 1))
					i += 1 % len(points)

			self.remove_curve_2d_handler()
			self.curve_2d_handler = draw_2d_polyline(self, context, points, index_buffer[:-1])
			update_viewport(self, context)

	def add_point(self, context, event):      
		cursor = get_cursor(self, event)
		view_vector, ray_origin = get_view_vector_and_ray_origin(self, context, cursor)
		direction = view_vector.normalized()            

		if event.ctrl: # snap to mesh verts
			if event.shift:
				vert_co = self.snap_to_verts(context, cursor)
				if vert_co is not None:
					target = vector_3d_to_screen(self, context, vert_co)
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
						self.radius.append((target, vert_co))

						self.remove_target_handler()
						self.target_handler = draw_target(self, context, target)
						self.draw_screen_polyline_circle(context, target)
						update_viewport(self, context)
		
			elif len(self.snap_points) > 0: # snap to curves and empties
				screen_world_map = get_screen_world_map(self, context, self.snap_points)				
				target = snap_get_target(self, context, cursor, screen_world_map).freeze()
				if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
					nearest_world = screen_world_map.get(target)
					self.radius.append((target, nearest_world))

					self.remove_target_handler()
					self.target_handler = draw_target(self, context, target)
					self.draw_screen_polyline_circle(context, target)
					update_viewport(self, context)

		elif event.shift:
			self.draw_screen_polyline_circle(context, cursor)
			point = project(self, context, cursor, on_mesh=True)
			self.radius.append(point)

		else:
			self.draw_screen_polyline_circle(context, cursor)
			point = project(self, context, cursor, on_mesh=False)
			self.radius.append(point)

	def modal(self, context, event):        
		if len(self.points) == 0 and (event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE')):
			return {'PASS_THROUGH'}

		elif event.type == 'MIDDLEMOUSE':
			return {'PASS_THROUGH'} 

		elif len(self.points) == 0 and event.type in ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
			return {'PASS_THROUGH'}

		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			self.add_point(context, event)
			if len(self.radius) == 2:
				self.remove_target_handler()
				self.remove_curve_2d_handler()
				update_viewport(self, context)              
				new_snap_points = self.build_polyline_circle(context)
				self.snap_points = self.snap_points.union(new_snap_points)
				self.points.clear()
				self.radius.clear()
				update_viewport(self, context)
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'
				return {'FINISHED'}

			return {'RUNNING_MODAL'}

		elif event.type == 'MOUSEMOVE' and len(self.radius) != 2:
			cursor = Vector((event.mouse_region_x, event.mouse_region_y))
			self.remove_target_handler()                
			self.target_handler = draw_target(self, context, cursor)
			self.remove_curve_2d_handler()
			self.draw_screen_polyline_circle(context, cursor)
			update_viewport(self, context)
			return {'RUNNING_MODAL'}

		elif event.type in {'RET'} and event.value == 'PRESS':         
			self.add_point(context, event)
			if len(self.radius) == 2:
				self.build_polyline_circle(context)
			self.remove_target_handler()
			self.remove_curve_2d_handler()
			update_viewport(self, context)
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			return {'FINISHED'}

		elif event.type in {'ESC'} and event.value == 'PRESS':
			self.remove_target_handler()
			self.remove_curve_2d_handler()
			update_viewport(self, context)
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			return {'FINISHED'}		

		return {'RUNNING_MODAL'}

class BT_DrawPolylineRectangle(BT_Draw):
	bl_idname = "curve.bt_draw_polyline_rectangle"
	bl_label = "Polyline Rectangle"
	bl_description = "Build a new Polyline Rectangle"
	bl_options = {'REGISTER', 'UNDO'}

	diagonal = []

	@classmethod
	def poll(cls, context):
		wm = context.window_manager
		return wm.bt_modal_on != 'BT_POLYRECTANGLE'

	def __init__(self, *args, **kwargs):
		BT_Draw.__init__(self, *args, **kwargs)

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not BT_Draw.init_draw(self, context):
			return{'CANCELLED'}
		
		context.window_manager.bt_modal_on = 'BT_POLYRECTANGLE'
		context.window_manager.modal_handler_add(self)

		return{'RUNNING_MODAL'}

	def build_polyline_rectangle(self, context):
		if len(self.diagonal) == 2:
			diagonal = self.diagonal
			p0 = Vector((diagonal[0][0]))
			p2 = Vector((diagonal[1][0]))
			p1 = Vector((p2.x, p0.y))
			p3 = Vector((p0.x, p2.y))
	
			points = [p0, p1, p2, p3, p0]

			for point in points:
				self.points.append(project(self, context, point))   

			polyline_rectangle = add_polyline(self, context, [point[1] for point in self.points], 'PolylineRectangle', is_closed=True)
			polyline_rectangle.location = diagonal[0][1].lerp(diagonal[1][1], 0.5)
			polyline_rectangle.color = context.scene.bt_color
			
			return set(point[1].copy().freeze() for point in self.points)

		return set()

	def draw_screen_polyline_rectangle(self, context, cursor):
		if len(self.diagonal) == 1:     
			diagonal = self.diagonal
			p0 = Vector((diagonal[0][0]))
			p2 = cursor
			p1 = Vector((p2.x, p0.y))
			p3 = Vector((p0.x, p2.y))
	
			points = [p0, p1, p2, p3]

			index_buffer = [(0, 1), (1, 2), (2, 3), (3, 0)]

			self.remove_curve_2d_handler()
			self.curve_2d_handler = draw_2d_polyline(self, context, points, index_buffer)
			update_viewport(self, context)

	def add_point(self, context, event):
		cursor = get_cursor(self, event)
		view_vector, ray_origin = get_view_vector_and_ray_origin(self, context, cursor)		
		direction = view_vector.normalized()

		if event.ctrl: # snap to mesh verts
			if event.shift:
				vert_co = self.snap_to_verts(context, cursor)
				if vert_co is not None:
					target = vector_3d_to_screen(self, context, vert_co)
					if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target, self.RADIUS):            
						self.diagonal.append((target, vert_co))

						self.remove_target_handler()
						self.target_handler = draw_target(self, context, target)
						self.draw_screen_polyline_rectangle(context, target)
						update_viewport(self, context)				

			elif len(self.snap_points) > 0: # snap to curves and empties
				screen_world_map = get_screen_world_map(self, context, self.snap_points)	
				target = snap_get_target(self, context, cursor, screen_world_map).freeze()

				if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target , self.RADIUS):
					nearest_world = screen_world_map.get(target)
					self.draw_screen_polyline_rectangle(context, target)
					self.diagonal.append((target, nearest_world))
					self.remove_target_handler()
					self.target_handler = draw_target(self, context, target)
					update_viewport(self, context)
		
		elif event.shift:
			self.draw_screen_polyline_rectangle(context, cursor)
			point = project(self, context, cursor, on_mesh=True)
			self.diagonal.append(point)			

		else:
			self.draw_screen_polyline_rectangle(context, cursor)
			point = project(self, context, cursor, on_mesh=False)
			self.diagonal.append(point)
	
	def modal(self, context, event):
		if len(self.points) == 0 and (event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE')):
			return {'PASS_THROUGH'}

		elif event.type == 'MIDDLEMOUSE':
			return {'PASS_THROUGH'}

		elif len(self.points) == 0 and event.type in ('WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
			return {'PASS_THROUGH'}

		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			self.add_point(context, event)      
			if len(self.diagonal) == 2:
				self.remove_target_handler()
				self.remove_curve_2d_handler()
				update_viewport(self, context)              
				new_snap_points = self.build_polyline_rectangle(context)
				self.snap_points = self.snap_points.union(new_snap_points)
				self.points.clear()
				self.diagonal.clear()
				context.workspace.status_text_set(None)
				context.window_manager.bt_modal_on = 'NONE'
				return {'FINISHED'}
			return {'RUNNING_MODAL'}				  

		elif event.type == 'MOUSEMOVE' and len(self.diagonal) != 2:
			cursor = Vector((event.mouse_region_x, event.mouse_region_y))           
			self.remove_target_handler()                        
			self.target_handler = draw_target(self, context, cursor)
			self.remove_curve_2d_handler()
			self.draw_screen_polyline_rectangle(context, cursor)
			update_viewport(self, context)
			return {'RUNNING_MODAL'}

		elif event.type in {'RET'} and event.value == 'PRESS':
			self.add_point(context, event)      
			if len(self.diagonal) == 2:         
				self.build_polyline_rectangle(context)
			self.remove_target_handler()
			self.remove_curve_2d_handler()
			update_viewport(self, context)
			self.points.clear()
			self.diagonal.clear()
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			return {'FINISHED'}       

		elif event.type == 'ESC' and event.value == 'PRESS':
			self.remove_target_handler()
			self.remove_curve_2d_handler()
			update_viewport(self, context)
			self.points.clear()
			self.diagonal.clear()
			context.workspace.status_text_set(None) 
			context.window_manager.bt_modal_on = 'NONE'  
			return {'CANCELLED'}

		return {'RUNNING_MODAL'}

class BT_Snap(Operator):
	bl_idname = "curve.bt_snap"
	bl_label = "Snap"
	bl_description = "Snap selected control points to other Bézier curve or Polyline"
	bl_options = {'REGISTER', 'UNDO'}

	snap_map = {}
	RADIUS = 25
	snap_targets_handler = None
	needs_update = False

	@classmethod
	def poll(cls, context):
		wm = context.window_manager
		return wm.bt_modal_on != 'BT_SNAP' and context.object is not None and context.mode == 'EDIT_CURVE'

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}
		
		curve = context.object
		self.build_snap_map(context)
		
		context.workspace.status_text_set('[LMB]: Snap [LEFT/RIGHT ARROW]: Select next point [ESC]: Quit')
		context.window_manager.bt_modal_on = 'BT_SNAP'

		if is_bezier(curve):
			set_handle_type(self, curve, 'FREE')

		if not len(self.get_points(context.object)):
			self.report({'WARNING'}, self.bl_label + ': Select Polyline or Bézier control point(s). Use Left/Right Arrow to loop throughh points')
		
		context.window_manager.modal_handler_add(self)

		return {'RUNNING_MODAL'}

	def get_points(self, curve):		
		return [point for point in curve.data.splines[0].bezier_points if point.select_control_point] if is_bezier(curve) else [point for point in curve.data.splines[0].points if point.select]	

	def build_snap_map(self, context):
		viewport = get_view_3d(self, context)
		width=viewport.width
		height=viewport.height	
		
		curves = self.get_visible_curves(context)
		curve = context.object
		
		for curve in curves:
			if curve is context.object:
				continue
			
			interpolated_points = mathutils_interpolate_n_bezier_points(curve, curve.data.splines[0].resolution_u+1) if is_bezier(curve) else [curve.matrix_world@Vector(point.co.xyz) for point in curve.data.splines[0].points]

			for point in interpolated_points:
				screen_point = vector_3d_to_screen(self, context, point)
				if screen_point is None or (screen_point.x > width or screen_point.x < 0) or (screen_point.y > height or  screen_point.y < 0):
					continue			
				self.snap_map[screen_point.freeze()] = point
		
		self.snap_targets_handler = draw_snap_targets(self, context, [tuple(point) for point in self.snap_map.values()])

	def calculate_gizmo_center(self, matrix, points): 
		return matrix@(sum((point.co.xyz for point in points), Vector())/len(points))

	def remove_snap_targets_handler(self):
		remove_gpu_draw_handler(self, self.snap_targets_handler)
		self.snap_targets_handler = None

	def get_visible_curves(self, context):
		curves = []
		for obj in context.view_layer.objects:
			if obj.visible_get() and obj.type=='CURVE' and not obj.data.splines[0].type == 'NURBS':
				curves.append(obj)

		return curves

	def snap(self, context, event, points):		
		cursor = get_cursor(self, event)
		curve = context.object
		matrix = curve.matrix_world		

		if len(self.snap_map) > 0:
			target = snap_get_target(self, context, cursor, self.snap_map).copy().freeze()
			if BT_Cursor.is_cusror_in_radius_range_from_nearest_point(self, cursor, target, self.RADIUS):
				nearest_world = self.snap_map.get(target)
				if nearest_world is not None:
					distance = nearest_world - (self.calculate_gizmo_center(matrix, points))
					for point in points:						
						translation = Matrix.Translation(distance.xyz)
						point.co.xyz = translation@point.co.xyz
						if len(point.co) == 3:	# is bezier point
							point.handle_right = translation@point.handle_right
							point.handle_left = translation@point.handle_left

		context.workspace.status_text_set('[LMB]: Snap [LEFT/RIGHT ARROW]: Select next point [ESC]: Quit')
	
	def select_next_point(self, context, direction):
		point = None
		curve = context.object
		points = [point for point in curve.data.splines[0].bezier_points] if is_bezier(curve) else [point for point in curve.data.splines[0].points]	
		if not len(self.get_points(context.object)):
			point = points[0]
		else:
			index = get_bezier_point_index(points, self.get_points(context.object)[0])
			
			if direction == 'left':
				if index-1 >= 0:					
					point = curve.data.splines[0].bezier_points[index-1] if is_bezier(curve) else curve.data.splines[0].points[index-1]
			
			elif direction == 'right':
				if index+1 < len(points):					
					point = curve.data.splines[0].bezier_points[index+1] if is_bezier(curve) else curve.data.splines[0].points[index+1]			
		
		if point is not None:
			bpy.ops.curve.select_all(action='DESELECT')
			if is_bezier(curve):
				point.select_control_point = True
			else:
				point.select = True

	def modal(self, context, event):
		if (event.alt and event.type in ('LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'MOUSEMOVE')):
			self.remove_snap_targets_handler()
			self.needs_update = True
			return {'PASS_THROUGH'}

		elif event.type in ('MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE'):
			self.remove_snap_targets_handler()			
			self.needs_update = True
			return {'PASS_THROUGH'}
	
		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			self.snap(context, event, self.get_points(context.object))
			return {'RUNNING_MODAL'}
		
		elif event.type == 'MOUSEMOVE':
			if self.needs_update:
				self.snap_map.clear()
				self.build_snap_map(context)
				self.needs_update = False

		elif event.type == 'LEFT_ARROW' and event.value == 'PRESS':
			self.select_next_point(context, 'left')

		elif event.type == 'RIGHT_ARROW' and event.value == 'PRESS':
			self.select_next_point(context, 'right')	

		elif event.type in {'ESC', 'RET'}:			
			self.snap_map.clear()
			context.window_manager.bt_modal_on = 'NONE'
			context.workspace.status_text_set(None)
			self.remove_snap_targets_handler()
			update_object_edit(context)
			return {'FINISHED'}

		return{'RUNNING_MODAL'}

def calculate_new_bezier_point_at_t(self, points, t):
# We'll use De Casteljau's algorithm here

#           * p1 --------- ..*.. p1p2 ------------- * p2
#          /          ..                ..             \
#         /    ..  * ......... * ........... *  ..      \
#        / ..     handle left  b(t)   handle right  ..   \
#  p0p1 *                                                 * p2p3
#      /                                                   \
#     /                                                     \
#    * p0                                                    * p3

	p0, p1, p2, p3 = points
	
	p0p1 = p0.lerp(p1, t)
	p1p2 = p1.lerp(p2, t)
	p2p3 = p2.lerp(p3, t)

	# new point	
	handle_left	 = p0p1.lerp(p1p2, t)
	handle_right =  p1p2.lerp(p2p3, t)
	b = handle_left.lerp(handle_right, t)	
	
	# returns coordinates of p0 and p3 with changed handles
	return [
				[
					p0, # handle left
					p0, 
					p0p1 # handle right
				],

				[
					handle_left,
					b,
					handle_right
				],

				[
					p2p3, # handle left
					p3,
					p3  # handle right
				]
			]

class BT_Split(Operator, BT_Cursor):
	bl_idname = "curve.bt_split"
	bl_label = "Split"
	bl_description = "Split and separate a Bézier curve"
	bl_options = {'REGISTER', 'UNDO'}

	RADIUS = 25
	screen_world_map = dict()
	screen_world_map_needs_update = False
	points = None
	target_handler = None

	@classmethod
	def poll(cls, context):
		return context.object is not None and is_bezier(context.object) and context.window_manager.bt_modal_on != 'BT_SPLIT'

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not is_single_view3d(self, context):
			return {'CANCELLED'}

		curve = context.object

		if not is_bezier(curve):
			self.report({'ERROR'}, self.bl_label + ': Can only split Bézier curves!')
			return {'CANCELLED'}

		if len(curve.data.splines) > 1:
			self.report({'ERROR'}, self.bl_label + ': Can only split Bézier curves with a single spline!')
			return {'CANCELLED'}
			
		spline = curve.data.splines[0] #if curve.data.splines.active is None else curve.data.splines.active
		resolution = spline.resolution_u

		if not resolution > 1:
			self.report({'ERROR'}, "Resolution must be higher than 1")
			return {'CANCELLED'}

		self.points = mathutils_interpolate_n_bezier_points(curve, resolution+1)
		context.window.cursor_set('KNIFE')
		
		if len(self.screen_world_map) == 0: 
			self.screen_world_map = get_screen_world_map(self, context, self.points)

		context.workspace.status_text_set('[LMB]: Split and finish  [ESC]: Quit')
		context.window_manager.bt_modal_on = 'BT_SPLIT'
		context.window_manager.modal_handler_add(self)

		return {'RUNNING_MODAL'}

	def remove_target_handler(self):
		remove_gpu_draw_handler(self, self.target_handler)
		self.target_handler = None

	def find_nearest_screen_point(self, cursor, screen_world_map):
		dist = dict()
		for point in screen_world_map.keys():
			dist[get_distance(point, Vector(cursor))] = point

		if not len(dist) > 0:
			return None	

		return dist.get(min(dist)).freeze()

	def get_bezier_split_point(self, context, event):
		points = self.points		
		cursor = get_cursor(self, event)

		nearest_screen_point = self.find_nearest_screen_point(cursor, self.screen_world_map)
		if nearest_screen_point is None:
			return None

		if get_distance(Vector((cursor)), nearest_screen_point) <= self.RADIUS:
			curve = context.object
			
			screen_world_map = get_screen_world_map(self, context, self.points)			
			nearest_world = self.screen_world_map.get(nearest_screen_point).freeze()			
			resolution = curve.data.splines[0].resolution_u

			# control point indices (p0 and p3) are divisible by resolution
			for index, point in enumerate(points):
				if point == nearest_world:
					return (index, nearest_world)

		return None     

	def split(self, context, bezier_split_point):
		source = context.object
		matrix = source.matrix_world		

		for obj in context.selected_objects:
			obj.select_set(False)

		bpy.ops.object.mode_set(mode = 'OBJECT')

		source.select_set(True)

		spline = source.data.splines[0]
		bezier_points = spline.bezier_points	
		resolution = spline.resolution_u

		curve_left = None
		curve_right = None

		index, split_position = bezier_split_point

		split_point_segment_index = index%resolution

		# find p0 and p3 as indices in bezier_points
		p0_full = index - split_point_segment_index
		p3_full = p0_full + resolution

		p0 = int(p0_full/resolution)
		p3 = int(p3_full/resolution)

		t = (index%resolution)/resolution

		split = calculate_new_bezier_point_at_t(self, (
			matrix @ bezier_points[p0].co,
			matrix @ bezier_points[p0].handle_right,
			matrix @ bezier_points[p3].handle_left,
			matrix @ bezier_points[p3].co,
		 ), t)

		points_left_to_split = bezier_points[0:p3]
		points_right_to_split = bezier_points[p3:]		

		curve_data_left = BT_BezierCurve(None)
		# all points on the left
		for point in points_left_to_split:
			curve_data_left.points.append(
								[
									matrix @ point.handle_left,
									matrix @ point.co,
									matrix @ point.handle_right
								]
							)		
		
		point_exists = False
		bezier_full_points_coords = [point.co for point in bezier_points]
		for position in bezier_full_points_coords:			
			if is_equal(matrix@position, split_position, 5):
				point_exists = True
				break

		# add a new point
		if point_exists:
			curve_left = curve_data_left.build(context, resolution, 'BézierCurve', is_set_pivot=False)

			if curve_left is None:
				self.report({'ERROR'}, self.bl_label + 'Could not build the left side curve!')
				return	False		
		else:			
			curve_data_left.points.append(
									[									
										split[1][0],
										split[1][1],
										split[1][1]
									]
								)

			curve_left = curve_data_left.build(context, resolution, 'BézierCurve', is_set_pivot=False)

			if curve_left is None:
				self.report({'ERROR'}, self.bl_label + 'Could not build the left side curve!')
				return False

			curve_left.data.splines[0].bezier_points[-2].handle_right = split[0][2]

			curve_left_last_point = curve_left.data.splines[0].bezier_points[-1]
			curve_left_last_point.handle_right = Matrix.Translation(curve_left_last_point.co) @ curve_left_last_point.co - curve_left_last_point.handle_left

		# curve_2
		curve_data_right = BT_BezierCurve(None)
		curve_data_right.points.append(
								[
									split[1][1],
									split[1][1],
									split[1][2]
								]
							)

		for point in points_right_to_split:
			curve_data_right.points.append(
								[
									matrix @ point.handle_left,
									matrix @ point.co,
									matrix @ point.handle_right
								]
							)

		curve_right = curve_data_right.build(context, resolution, 'BézierCurve', is_set_pivot=False)	
		curve_right.data.splines[0].bezier_points[1].handle_left = split[2][0]

		curve_right_first_point = curve_right.data.splines[0].bezier_points[0]
		curve_right_first_point.handle_left = Matrix.Translation(curve_right_first_point.co) @ curve_right_first_point.co - curve_right_first_point.handle_right
		
		if curve_right is None:
			self.report({'ERROR'}, self.bl_label + 'Could not build the right side curve!')
			return False

		parent = source.parent
		if parent is not None:
			curve_right.parent = parent
			curve_left.parent = parent		
		
		for new_curve in (curve_left, curve_right):
			new_curve.select_set(True)

			collection = source.users_collection[0]
			if collection != context.scene.collection:
				if new_curve.name not in collection.objects:
					collection.objects.link(new_curve)
				if new_curve.name in context.scene.collection.objects:
					context.scene.collection.objects.unlink(new_curve)

		bpy.context.view_layer.objects.active = source
		
		bt_transfer_curve_data(self, source, (curve_left, curve_right))

		for curve in (curve_left, curve_right):
			set_pivot(curve, curve.matrix_world@curve.data.splines[0].bezier_points[0].co)

		source.user_remap(curve_left)
		bpy.data.objects.remove(source)
		context.view_layer.objects.active = curve_left				
		curve_right.select_set(False)

		curve_left.data.splines[0].bezier_points[-1].select_control_point=True
		
		return True

	def modal(self, context, event):
		if (event.alt and event.type in {'LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}) or (event.type in {'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}): 
			self.remove_target_handler()
			update_viewport(self, context)
			self.screen_world_map_needs_update = True			
			return {'PASS_THROUGH'}

		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':
			bezier_split_point = self.get_bezier_split_point(context, event)

			if bezier_split_point is not None:
				if bezier_split_point[0] == 0 or bezier_split_point[0] == len(self.points)-1:					
					return{'RUNNING_MODAL'}

				success = self.split(context, bezier_split_point)
				self.remove_target_handler()
				update_viewport(self, context)

				if success:
					context.window.cursor_set('DEFAULT')
					context.workspace.status_text_set(None)
					context.window_manager.bt_modal_on = 'NONE'					
					bpy.ops.object.mode_set(mode = 'OBJECT')			
					return {'FINISHED'}

			return {'RUNNING_MODAL'}

		elif event.type == 'MOUSEMOVE':
			self.remove_target_handler()		
			cursor = get_cursor(self, event)
			
			if self.screen_world_map_needs_update:
				self.screen_world_map = get_screen_world_map(self, context, self.points)
				self.screen_world_map_needs_update = False

			nearest_screen_point = find_nearest_screen_point(self, get_cursor(self, event), self.screen_world_map)
			
			if get_distance(cursor, nearest_screen_point) <= self.RADIUS:
				self.target_handler = draw_target(self, context, nearest_screen_point)

			update_viewport(self, context)

		elif event.type == 'ESC':
			self.remove_target_handler()
			update_viewport(self, context)		
			context.window.cursor_set('DEFAULT')
			context.window_manager.bt_modal_on = 'NONE'
			update_object_edit(context)
			return {'CANCELLED'}

		return {'RUNNING_MODAL'}

class BT_Join(Operator):
	bl_idname = 'curve.bt_join'
	bl_label = 'Join'
	bl_description = 'Join 2 Bézier curves. The curves are expected to have the same direction. The active curve should be followed by the second curve in the same direction'
	bl_options = {'REGISTER', 'UNDO'}
	remove_src: bpy.props.BoolProperty(name='Remove source', default=True)

	@classmethod
	def poll(cls, context):
		return context.object is not None and is_bezier(context.object)

	def execute(self, context):
		if not len(context.selected_objects) == 2:
			self.report({'ERROR'}, self.bl_label + ': Select two Bézier curves to join them!')
			return {'CANCELLED'}

		if not (is_bezier(context.selected_objects[0]) and is_bezier(context.selected_objects[1])):
			self.report({'ERROR'}, self.bl_label + ': Select only Bézier curves!')
			return {'CANCELLED'}	

		sel = context.selected_objects		

		if sel[0] is not context.active_object:
			sel[0], sel[1] = sel[1], sel[0]
		
		first = BT_BezierCurve(sel[0])
		second = BT_BezierCurve(sel[1])

		active_curve = sel[0]

		first.points[-1][2] = second.points[0][2]

		for point in second.points[1:]:
			first.points.append(point)

		new_curve = first.build(context, 12, 'BézierCurve')		
		
		new_curve.select_set(True)
		context.view_layer.objects.active = active_curve		
		bt_transfer_curve_data(self, active_curve, [new_curve])

		collection = active_curve.users_collection[0]
		if collection != context.scene.collection:
			if new_curve.name not in collection.objects:
				collection.objects.link(new_curve)
			if new_curve.name in context.scene.collection.objects:
				context.scene.collection.objects.unlink(new_curve)
		
		parent = active_curve.parent
		if parent is not None:
			new_curve.parent = parent
		
		active_curve.user_remap(new_curve)
		if self.remove_src:
			for curve in sel:
				if curve.name in bpy.data.objects:
					bpy.data.objects.remove(curve)

		context.view_layer.objects.active = new_curve

		return {'FINISHED'}

class BT_Smooth(Operator):
	bl_idname = 'curve.bt_smooth'
	bl_label = 'Smooth'
	bl_description = 'Makes the curve smoother at the selected control point'
	bl_options = {'REGISTER', 'UNDO'}
	# invert_left: bpy.props.BoolProperty(name='Invert Left', description='Invert handle on the left', options={'SKIP_SAVE'})
	# invert_right: bpy.props.BoolProperty(name='Invert Right', description='Invert handle on the right', options={'SKIP_SAVE'})

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE'
	
	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		curve = context.object

		if not is_bezier(curve):
			self.report({'ERROR'}, self.bl_label + ': Can only smooth out points on Bézier curves!')
			return {'CANCELLED'}

		if len(curve.data.splines) > 1:
			self.report({'ERROR'}, self.bl_label + ': Bézier curve should have a single spline!')
			return {'CANCELLED'}

		if not curve.data.splines[0].resolution_u > 1:
			self.report({'ERROR'}, "Bézier curve spline resolution should be higher than 1")
			return {'CANCELLED'}

		return self.execute(context)

	def execute(self, context):
		curve = context.object
		bezier_points = curve.data.splines[0].bezier_points
		selected_points = [point for point in bezier_points if point.select_control_point]
		
		if len(bezier_points) < 3:
			self.report({'ERROR'}, "The Bézier curve must have at least 3 control points")
			return {'CANCELLED'}
		
		if len(selected_points) != 1:
			self.report({'ERROR'}, "Select only 1 control point on the Bézier curve. It cannot be a handle")
			return {'CANCELLED'}

		point = selected_points[0]

		if point == bezier_points[0] or point == bezier_points[-1]:
			self.report({'ERROR'}, "Select a control point that is not the first or the last on the Bézier curve")
			return {'CANCELLED'}

		index = get_bezier_point_index(bezier_points, point)		
		if index is None:
			self.report({'ERROR'}, "Point is not found")
			return {'CANCELLED'}

		point_left = bezier_points[index-1]
		point_right = bezier_points[index+1]

		# now let's select left and right points		
		point_left.select_control_point = True
		point_right.select_control_point = True
		
		# add support points
		bpy.ops.curve.subdivide()
		
		# update points list
		bezier_points = curve.data.splines[0].bezier_points

		# point should now be at index+1
		point = bezier_points[index+1]
		
		# remove point
		for p in bezier_points:
			if p != point:
				p.select_control_point=False

		bpy.ops.curve.bt_remove_point()#invert_right=self.invert_right, invert_left=self.invert_left

		return {'FINISHED'}

class BT_Merge(Operator):
	bl_idname = 'curve.bt_merge'
	bl_label = 'Merge two points'
	bl_description = 'Merge two neighbouring Bézier control points at the center'
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE'
	
	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		curve = context.object

		if not is_bezier(curve):
			self.report({'ERROR'}, self.bl_label + ': Can only merge points on Bézier curves!')
			return {'CANCELLED'}

		if len(curve.data.splines) > 1:
			self.report({'ERROR'}, self.bl_label + ': Bézier curve should have a single spline!')
			return {'CANCELLED'}

		if not curve.data.splines[0].resolution_u > 1:
			self.report({'ERROR'}, "Bézier curve spline resolution should be higher than 1")
			return {'CANCELLED'}

		return self.execute(context)

	def execute(self, context):
		curve = context.object

		bezier_points = curve.data.splines[0].bezier_points
		if len(bezier_points) < 3:			
			self.report({'ERROR'}, "The Bézier curve must have at least 3 control points")
			return {'CANCELLED'}

		indices = get_selected_bezier_points_indices(bezier_points)
		if not indices:
			self.report({'ERROR'}, "No control points selected")
			return {'CANCELLED'}

		if len(indices) != 2:
			self.report({'ERROR'}, "Select 2 neighbouring control points on the Bézier curve. They cannot be handles")
			return {'CANCELLED'}

		if bezier_points[indices[0]+1] != bezier_points[indices[1]]:
			self.report({'ERROR'}, "Select 2 neighbouring control points")
			return {'CANCELLED'}

		# this will be a merge point
		bpy.ops.curve.subdivide()

		# update points list
		bezier_points = curve.data.splines[0].bezier_points	
			
		for point in bezier_points:
			point.select_control_point = False

		point_left = bezier_points[indices[0]+2]
		point_left.select_control_point=True

		if bpy.ops.curve.bt_remove_point.poll():
			bpy.ops.curve.bt_remove_point()
		else:
			self.report({'ERROR'}, "Select 2 neighbouring control points on the Bézier curve. They cannot be handles")
			return {'CANCELLED'}

		bezier_points = curve.data.splines[0].bezier_points

		point_right = bezier_points[indices[1]-1]
		point_right.select_control_point=True
		bpy.ops.curve.bt_remove_point()

		bezier_points = curve.data.splines[0].bezier_points
		merge_point = bezier_points[indices[0]]
		merge_point.select_control_point=True

		return {'FINISHED'}

class BT_SetBezierHandleType(Operator):
	bl_idname = 'curve.bt_set_handle_type'
	bl_label = 'Set Bézier Points Handle Type'
	bl_description = 'Set Bézier curve points handle type'
	bl_options = {'REGISTER', 'UNDO'}
	handle_type: bpy.props.EnumProperty(items=[
		('FREE','FREE',''),
		('VECTOR','VECTOR',''),
		('ALIGNED','ALIGNED',''),
		('AUTO','AUTO','')],
		default='ALIGNED')

	@classmethod
	def poll(cls, context):
		return context.object is not None and is_bezier(context.object)

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		curve = context.object
		if not is_bezier(curve):
			self.report({'ERROR'}, self.bl_label + ': Selected object should be a Bézier curve')
			return {'CANCELLED'}

		if context.mode == 'OBJECT':
			update_object_edit_object(context)
		else:
			update_edit_object_edit(context)

		return self.execute(context)
		
	def execute(self, context):
		if context.mode == 'EDIT_CURVE':
			for curve in context.selected_objects:			
				bezier_points = curve.data.splines[0].bezier_points
				points = [point for point in bezier_points if point.select_control_point]

				if not len(points):
					points = bezier_points

				for point in points:
					point.handle_left_type = self.handle_type
					point.handle_right_type = self.handle_type	
		else:
			for curve in context.selected_objects:	
				set_handle_type(self, curve, self.handle_type)

		return {'FINISHED'}

def get_bezier_point_index(bezier_points, point):
	for index, bezier_point in enumerate(bezier_points):
		if bezier_point == point:
			return index
	return None

def get_selected_bezier_points_indices(bezier_points):
	indices=[]
	for index, bezier_point in enumerate(bezier_points):
		if bezier_point.select_control_point:
			indices.append(index)
	return indices

def find_nearest_screen_point(self, cursor, screen_world_map):
	dist = dict()
	for point in screen_world_map.keys():
		dist[get_distance(point, Vector(cursor))] = point

	if not len(dist) > 0:
		return None	

	return dist.get(min(dist)).freeze()

class BT_Add(Operator, BT_Cursor):
	bl_idname = 'curve.bt_add_point'
	bl_label = 'Add Point'
	bl_description = 'Add a new control point inside the curve'
	bl_options = {'REGISTER', 'UNDO'}

	RADIUS = 25
	screen_world_map = dict()
	points = None

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE' and context.window_manager.bt_modal_on != 'BT_ADD_POINT'	

	screen_world_map_needs_update = False
	target_handler = None

	def remove_target_handler(self):
		remove_gpu_draw_handler(self, self.target_handler)
		self.target_handler = None

	def get_bezier_split_point(self, context, event):
		points = self.points		
		cursor = get_cursor(self, event)

		dist = dict()
		for point in self.screen_world_map.keys():
			dist[get_distance(point, Vector(cursor))] = point

		if not len(dist) > 0:
			return 0	

		nearest_screen = dist.get(min(dist)).freeze()

		if get_distance(Vector((cursor)), nearest_screen) <= self.RADIUS:
			curve = context.object
			
			screen_world_map = get_screen_world_map(self, context, self.points)			
			nearest_world = self.screen_world_map.get(nearest_screen).freeze()			
			resolution = curve.data.splines[0].resolution_u

			# control point indices (p0 and p3) are divisible by resolution
			for index, point in enumerate(points):
				if point == nearest_world:				
					return index

		return 0

	def invoke(self, context, event):
		if not context.space_data.type == 'VIEW_3D':
			self.report({'ERROR'}, "Current space is not 'VIEW_3D'")
			return {'CANCELLED'}

		if not is_single_view3d(self, context):
			return{'CANCELLED'}

		curve = context.object		

		if not is_bezier(curve):
			self.report({'ERROR'}, "Can only add points on Bézier curves!")
			return{'CANCELLED'}

		resolution = curve.data.splines[0].resolution_u
		if curve.type != 'CURVE':
			self.report({'ERROR'}, "Context Object type is not Curve")
			return {'CANCELLED'}

		self.points = mathutils_interpolate_n_bezier_points(curve, resolution+1)

		if not resolution > 1:
			self.report({'ERROR'}, "Resolution must be higher than 1")
			return {'CANCELLED'}

		context.window.cursor_modal_set('CROSS')
		
		if len(self.screen_world_map) == 0: 
			self.screen_world_map = get_screen_world_map(self, context, self.points)

		context.window_manager.modal_handler_add(self)
		context.workspace.status_text_set('[LMB]: Add a new point  [ESC]: Quit')
		context.window_manager.bt_modal_on = 'BT_ADD_POINT'

		return {'RUNNING_MODAL'}
	
	def add_point(self, context, bezier_split_point):
		source = context.object
		matrix = source.matrix_world
		bezier_points = source.data.splines[0].bezier_points
		resolution = source.data.splines[0].resolution_u

		split_point_segment_index = bezier_split_point%resolution

		# find p0 and p3 as indices in bezier_points
		p0_full = bezier_split_point - split_point_segment_index
		p3_full = p0_full + resolution

		p0 = int(p0_full/resolution)
		p3 = int(p3_full/resolution)

		t = (bezier_split_point%resolution)/resolution

		split = calculate_new_bezier_point_at_t(self, (
			matrix @ bezier_points[p0].co,
			matrix @ bezier_points[p0].handle_right,
			matrix @ bezier_points[p3].handle_left,
			matrix @ bezier_points[p3].co,
		 ), t)

		points_left_to_split = bezier_points[0:p3]
		points_right_to_split = bezier_points[p3:]

		curve_data_left = BT_BezierCurve(None)
		# all points on the left
		for point in points_left_to_split:
			curve_data_left.points.append(
								[
									matrix @ point.handle_left,
									matrix @ point.co,
									matrix @ point.handle_right
								]
							)
		
		point_exists = False
		bezier_full_points_coords = [point.co for point in bezier_points]
		for position in bezier_full_points_coords:			
			if is_equal(matrix@position, split[1][1], 5):
				point_exists = True				
				break
		
		if point_exists:
			return

		# new point
		curve_data_left.points.append(
								[									
									split[1][0],
									split[1][1],
									split[1][1]
								]
							)
			
		# change left to the split point's right handle
		# curve_left.data.splines[0].bezier_points[-2].handle_right = split[0][2]
		curve_data_left.points[-2][2] = split[0][2]

		# curve_2
		curve_data_right = BT_BezierCurve(None)
		curve_data_right.points.append(
								[
									split[1][1],
									split[1][1],
									split[1][2]
								]
							)

		for point in points_right_to_split:
			curve_data_right.points.append(
								[
									matrix @ point.handle_left,
									matrix @ point.co,
									matrix @ point.handle_right
								]
							)

		curve_data_left.points[-1][2] = split[1][2]
		curve_data_right.points[1][0] = split[2][0]

		new_point_index = len(curve_data_left.points) - 1

		for point in curve_data_right.points[1:]:
			curve_data_left.points.append(point)

		new_curve = curve_data_left.build(context, resolution, 'BézierCurve')
		new_curve.select_set(True)

		set_pivot(new_curve, source.location)
		
		bpy.context.view_layer.objects.active = source		
		bt_transfer_curve_data(self, source, (source, new_curve))
		
		name = source.name
		parent = source.parent

		bpy.context.view_layer.objects.active = new_curve
		
		new_curve.name = name
		new_curve.select_set(True)

		collection = source.users_collection[0]
		if collection != context.scene.collection:
			if new_curve.name not in collection.objects:
				collection.objects.link(new_curve)
			if new_curve.name in context.scene.collection.objects:
				context.scene.collection.objects.unlink(new_curve)
		
		if parent is not None:
			new_curve.parent = parent

		bpy.ops.object.mode_set(mode = 'EDIT')		
		point = new_curve.data.splines[0].bezier_points[new_point_index]
		point.select_control_point = True
		point.handle_left_type = 'ALIGNED'
		point.handle_right_type = 'ALIGNED'
		
		source.user_remap(new_curve)				
		bpy.data.objects.remove(source)
		context.view_layer.objects.active = new_curve		
		bpy.ops.object.mode_set(mode = 'EDIT')				

	def modal(self, context, event):		
		if (event.alt and event.type in {'LEFTMOUSE', 'RIGHTMOUSE', 'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}) or (event.type in {'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}): 
			self.remove_target_handler()
			update_viewport(self, context)
			self.screen_world_map_needs_update = True			
			return {'PASS_THROUGH'}

		elif event.type == 'LEFTMOUSE' and event.value == 'PRESS':			
			bezier_split_point = self.get_bezier_split_point(context, event)			

			if bezier_split_point == 0 or bezier_split_point == len(self.points)-1:				
				return{'RUNNING_MODAL'}

			self.add_point(context, bezier_split_point)
			curve = context.object
			self.points = mathutils_interpolate_n_bezier_points(curve, curve.data.splines[0].resolution_u+1)
			self.screen_world_map = get_screen_world_map(self, context, self.points)
			
			self.remove_target_handler()			
			update_viewport(self, context)
			# context.window.cursor_modal_set('DEFAULT')
			# context.workspace.status_text_set(None)
			# context.window_manager.bt_modal_on = 'NONE'			
			update_object_edit(context)
			return {'RUNNING_MODAL'}

		elif event.type == 'MOUSEMOVE':
			self.remove_target_handler()		
			cursor = get_cursor(self, event)
			
			if self.screen_world_map_needs_update:
				self.screen_world_map = get_screen_world_map(self, context, self.points)
				self.screen_world_map_needs_update = False

			nearest_screen_point = find_nearest_screen_point(self, get_cursor(self, event), self.screen_world_map)
			if not nearest_screen_point:
				return {'RUNNING_MODAL'}

			if get_distance(cursor, nearest_screen_point) <= self.RADIUS:
				self.target_handler = draw_target(self, context, nearest_screen_point)
			update_viewport(self, context)
			return {'RUNNING_MODAL'}

		elif event.type in {'ESC', 'RET'}:
			self.remove_target_handler()
			update_viewport(self, context)			
			context.window.cursor_modal_set('DEFAULT')
			context.workspace.status_text_set(None)
			context.window_manager.bt_modal_on = 'NONE'
			update_object_edit(context)
			return {'CANCELLED'}
		
		return{'RUNNING_MODAL'}

class BT_Move(Operator):
	bl_idname = 'curve.bt_move_point'
	bl_label = 'Move Bézier Point'
	bl_description = 'Moves a Bézier point to a position on the spline segment. If the point is not naturally interpolated or moved/changed, the new starting position will be assigned automatically'
	bl_options = {'REGISTER', 'UNDO'}
	t: bpy.props.FloatProperty(min=0.001, soft_min=0.001, soft_max=0.999, max=0.999, name='Value', default=0.5, options={'SKIP_SAVE'})

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE'

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 't', slider=True)

	def find_t(self, point):
		co = point.co
		hl = point.handle_left - co
		hr = co - point.handle_right

		full_length = (point.handle_left - point.handle_right).length
		
		if not full_length > 0:
			return 0
		
		t = hl.length/full_length
		return t

	def execute(self, context):
		curve = context.object
		if not is_bezier(curve):
			self.report({'ERROR'}, "Can only slide Bézier points")
			return{'CANCELLED'}

		update_edit_object_edit(context)
		matrix = curve.matrix_world
		bezier_points = curve.data.splines[0].bezier_points
		selected_points = [point for point in bezier_points if point.select_control_point]

		if len(selected_points) != 1:
			self.report({'ERROR'}, self.bl_label + ': Select a single Bézier control point. It cannot be a handle')
			return {'CANCELLED'}

		point , = selected_points

		# set handles explicitly to FREE type to avoid any possible automatic handle calculations 
		point.handle_left_type = 'FREE'
		point.handle_right_type = 'FREE'

		# get point index		
		index = 0
		for i, p in enumerate(bezier_points):
			if p == point:
				index = i
				break		
	
		if point == bezier_points[0] or point == bezier_points[-1]:
			new_point = None			
			if index == 0:
				p3 = bezier_points[index+1]
				
				# get a new position for the point and handles at t
				new_point = calculate_new_bezier_point_at_t(self, (
					point.co,
					point.handle_right,
					p3.handle_left,
					p3.co,
				 ), self.t)

				point.handle_left = new_point[1][0]			
				point.co = new_point[1][1]
				point.handle_right = new_point[1][2]
				p3.handle_left = new_point[2][0]	

			else:
				p0 = bezier_points[index-1]
				
				new_point = calculate_new_bezier_point_at_t(self, (
					p0.co,
					p0.handle_right,
					point.handle_left,
					point.co,
				 ), self.t)

				point.handle_left = new_point[1][0]			
				point.co = new_point[1][1]
				point.handle_right = new_point[1][2]
				p0.handle_right = new_point[0][2]			

		else:
			T = self.find_t(point)

			if not (T > 0 and T < 1):
				self.report({'WARNING'}, self.bl_label + ': Handles should not be in the control point\'s position!')
				return {'CANCELLED'}

			p0 = bezier_points[index-1]
			p3 = bezier_points[index+1]
					
			# here path is curve segment with the sliding point removed and handles adjusted
			handle_right = p0.co + ((p0.handle_right - p0.co)*(1/T))
			handle_left = p3.co + ((p3.handle_left - p3.co)*(1/(1-T)))
			
			update = calculate_new_bezier_point_at_t(self, (
				p0.co,
				handle_right,
				handle_left,
				p3.co,
			 ), self.t)

			point.handle_left = update[1][0]			
			point.co = update[1][1]
			point.handle_right = update[1][2]					
			
			p0.handle_right = update[0][2]
			p3.handle_left = update[2][0]

		return {'FINISHED'}

class BT_Flatten(Operator):
	bl_idname = 'curve.bt_flatten_points'
	bl_label = 'Flatten'
	bl_description = 'Flatten Bézier points'
	bl_options = {'REGISTER', 'UNDO'}
	axis: bpy.props.BoolVectorProperty(subtype='XYZ', options={'SKIP_SAVE'})
	xyz: bpy.props.BoolProperty(name='XYZ')

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE'

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		row = column.row(align=True)		
		row.prop(self, 'axis', text='Axis', toggle=True)
		row.prop(self, 'xyz', text='XYZ', toggle=True)

	def execute(self, context):
		curve = context.object			
		if not is_bezier(curve):
			self.report({'ERROR'}, "Can only flatten Bézier points!")
			return{'CANCELLED'}

		update_edit_object_edit(context)
		axis = self.axis			
		bezier_points = curve.data.splines[0].bezier_points
		selected_points = [point for point in bezier_points if point.select_control_point]

		if self.xyz:
			axis=(True, True, True)		

		if not len(selected_points) > 1:
			self.report({'ERROR'}, self.bl_label + ': Select at least 2 Bézier control points and their handles to line up')
			return {'CANCELLED'}

		set_handle_type(self, curve, 'FREE')

		# we need to set the curve origin to selected_points[0].co for projection		
		origin = selected_points[0].co.copy()
		offset = Matrix.Translation(origin)
		
		for point in bezier_points:
			point.co = offset.inverted() @ point.co
			point.handle_right = offset.inverted() @ point.handle_right
			point.handle_left = offset.inverted() @ point.handle_left
		
		# first and last selected control points make the line other points will be projected on	
		line = Vector((selected_points[0].co - selected_points[-1].co))
		
		for point in selected_points:
			if point not in (selected_points[0], selected_points[-1]):				
				proj = point.co.project(line)
				px = proj.x
				py = proj.y	
				pz = proj.z

				if axis[0]:
					point.co.x = px
				if axis[1]:
					point.co.y = py
				if axis[2]:
					point.co.z = pz

			if point.select_right_handle:
				proj = point.handle_right.project(line)
				hrx = proj.x
				hry = proj.y
				hrz = proj.z

				if axis[0]:
					point.handle_right.x = hrx
				if axis[1]:
					point.handle_right.y = hry
				if axis[2]:
					point.handle_right.z = hrz

			if point.select_left_handle:
				proj = point.handle_left.project(line)
				hlx = proj.x
				hly = proj.y
				hlz = proj.z

				if axis[0]:
					point.handle_left.x = hlx
				if axis[1]:
					point.handle_left.y = hly
				if axis[2]:
					point.handle_left.z = hlz

		for point in bezier_points:
			point.co = offset@point.co
			point.handle_right = offset@point.handle_right
			point.handle_left = offset@point.handle_left

		return {'FINISHED'}

def find_t(self, point):
	co = point.co
	hl = point.handle_left - co
	hr = co - point.handle_right

	full_length = (point.handle_left - point.handle_right).length

	t = hl.length/full_length
	return t

def bezier_point_prev(points, point):
	for index, bezier_point in enumerate(points):
		if bezier_point == point and bezier_point != points[0]:
			try: 
				return points[index-1]			
			except:
				print('Bezier Points index error')

	return point

def bezier_point_next(points, point):
	for index, bezier_point in enumerate(points):
		if bezier_point == point and bezier_point != points[-1]:
			try: 
				return points[index+1]
			except:
				print('Bezier Points index error')

	return point

#https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/Computation-of-rotation-minimizing-frames.pdf
def calculate_next_rmf(p_data, p_next_data):	
	#t - tangent
	#r - binormal
	#s - normal (s = t x r)

	p, t, r, s = p_data
	p_next, t_next = p_next_data	

	# first reflection
	v1 = p_next - p
	c1 = v1.length_squared
	r_left = r - (2/c1)*(v1.dot(r))*v1
	t_left = t - (2/c1)*(v1.dot(t))*v1

	# second reflection
	v2 = t_next - t_left
	c2 = v2.length_squared
	r_next = r_left - (2/c2)*(v2.dot(r_left))*v2
	s_next = t_next.cross(r_next)

	return (p_next, t_next, r_next, s_next)

def spawn_empty(name, location, *, size=0.05):	
	empty = bpy.data.objects.new(name, None)
	bpy.context.scene.collection.objects.link(empty)		
	empty.empty_display_size = size
	empty.matrix_world.translation = location

class BT_Offset(Operator):
	bl_idname = 'curve.bt_offset'
	bl_label = 'Offset'
	bl_description = 'Offset a Bézier curve'
	bl_options = {'REGISTER', 'UNDO'}
	distance: bpy.props.FloatProperty(name='Distance', description='Distance from the source curve')
	rotation: bpy.props.FloatProperty(name='Rotation°', description='Angle of rotation around the source curve in degrees')
	spawn_offset_points: bpy.props.BoolProperty(name='Spawn Offset Points', description='The target points that a perfect offset curve should pass through')
	precision: bpy.props.IntProperty(name='Precision', description='Changing precision can fix wrong curvature in some cases', soft_min=2, min=2, soft_max=1000, max=1000, default=100)
	duplicate: bpy.props.BoolProperty(name='Duplicate', default=True, description='Keep the original curve unchanged')
	
	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'distance')
		column.prop(self, 'rotation')
		column.prop(self, 'precision')		
		column.prop(self, 'spawn_offset_points')
		column.prop(self, 'duplicate')	
		column.separator()

	@classmethod
	def poll(cls, context):
		return context.object is not None and is_bezier(context.object)

	def rotate(self, vector, axis, angle): #https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula	
		return vector*cos(angle) + (axis.cross(vector))*sin(angle) + axis*(axis.dot(vector))*(1 - cos(angle))

	def calculate_initial_rmf(self, point):
		tangent = (point.handle_right - point.co).normalized()
		binormal = (point.handle_right.cross(point.handle_left).normalized())
		binormal = self.rotate(binormal, (point.handle_left - point.handle_right).normalized(), radians(self.rotation))
		normal = tangent.cross(binormal).normalized()		
		return (point.co, tangent, binormal, normal)

	def find_best_handle_length(self, points, handle_index, target):
		precision = self.precision
		def find_closest_interpolated_point(h):
			interpolated_points = mathutils.geometry.interpolate_bezier(p0, h if handle_index else p1, h if not handle_index else p2, p3, precision)
			kd_tree = build_kd_tree(interpolated_points)			
			return kd_tree.find(target)[0].copy().freeze()

		# Here we will try to approximate the best handle length for offset
		# The best length should give us the closest interpolated point to the target
		# Targets are rmfs interpolated points
		p0, p1, p2, p3 = points

		n = 0.01*(p1-p0) if handle_index else 0.01*(p2-p3)
		h = p1.copy() if handle_index else p2.copy()

		# initial distance to target
		current_distance = get_distance(find_closest_interpolated_point(h), target)		
		best_handle = None

		# try making the handle longer
		for k in range(precision):
			h += n		
			new_distance = get_distance(find_closest_interpolated_point(h), target)
			if new_distance > current_distance:		
				best_handle = h				
				break

			current_distance = new_distance

		# try making the handle shorter		
		for k in range(precision):
			h -= n
			new_distance = get_distance(find_closest_interpolated_point(h), target)
			if new_distance > current_distance:
				best_handle = h				
				break

			current_distance = new_distance				

		return best_handle

	def execute(self, context):	
		context.evaluated_depsgraph_get()	
		bpy.ops.object.mode_set(mode='OBJECT')		
		
		if self.duplicate:
			for obj in context.selected_objects:
				if obj is not context.object:
					obj.select_set(False)

			bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False})
		
		curve = context.object	
		pivot = curve.location.copy()		

		if not is_bezier(curve):
			self.report({'ERROR'}, "Selected object is not a Bézier curve")
			return{'CANCELLED'}
		
		distance = self.distance		
		rotation = self.rotation		

		interpolated_points = mathutils_interpolate_n_bezier_points(curve, 4, world_space=False)

		# we want to remove control points and leave only interpolated intermediate points
		for index, point in enumerate(interpolated_points[:]):
			if index%3 == 0:
				interpolated_points.remove(point)

		# now let's pack each two intermediate points between bezier_points[n] and bezier_points[n+1] segment into tuples
		intermediate_points = list(zip(interpolated_points[::2], interpolated_points[1::2]))			

		bezier_points = curve.data.splines[0].bezier_points

		set_handle_type(self, curve, 'FREE')
		
		initial_rmf = self.calculate_initial_rmf(bezier_points[0])		
		rmfs = [initial_rmf]

		# here we will iterate through bezier segments
		# index = segment index
		# p0 = bezier_points[index]
		# p1 = bezier_points[index+1]
		for index in range(len(bezier_points)-1):
			rmfs.append(calculate_next_rmf(rmfs[-1], (
				intermediate_points[index][0],
				calculate_bezier_tangent((
					bezier_points[index].co,
					bezier_points[index].handle_right,
					bezier_points[index+1].handle_left,
					bezier_points[index+1].co,
					), 1/3).normalized()
				)))
			
			rmfs.append(calculate_next_rmf(rmfs[-1], (
				intermediate_points[index][1],
				calculate_bezier_tangent((
					bezier_points[index].co,
					bezier_points[index].handle_right,
					bezier_points[index+1].handle_left,
					bezier_points[index+1].co,
					), 2/3).normalized()
				)))
			
			rmfs.append(calculate_next_rmf(rmfs[-1], (
				bezier_points[index+1].co,
				(bezier_points[index+1].co - bezier_points[index+1].handle_left).normalized()
				)))

		if self.spawn_offset_points:
			for rmf in rmfs:
				p, s = (rmf[0], rmf[-1])
				
				empty = bpy.data.objects.new('Normal', None)
				bpy.context.scene.collection.objects.link(empty)			
				empty.empty_display_size = 0.05				
			
				offset = Matrix.Translation(p)	
				position = distance*s
				empty.location = curve.matrix_world@(offset@position)

		# Bezier_points_lookup is a list of 4-point segments
		# p0 first point
		# p1 and p2 are rmf interpolated points
		# p3 last point
		bezier_points_lookup = []
		for index in range(len(bezier_points)-1):			
			if (index+3) % len(rmfs) != 0:			
				p0 = Matrix.Translation(rmfs[(index*3)][0])@(distance*rmfs[(index*3)][-1])
				p1 = Matrix.Translation(rmfs[(index*3)+1][0])@(distance*rmfs[(index*3)+1][-1])
				p2 = Matrix.Translation(rmfs[(index*3)+2][0])@(distance*rmfs[(index*3)+2][-1])		
				p3 = Matrix.Translation(rmfs[(index*3)+3][0])@(distance*rmfs[(index*3)+3][-1])	

				bezier_points_lookup.append((
					p0,
					p1,
					p2,
					p3
				   ))

		for index in range(len(bezier_points)-1):			
			p0, p1, p2, p3 = bezier_points_lookup[index]

			point = bezier_points[index]
			normal = rmfs[index*3][-1]
			offset = Matrix.Translation(point.co)
			position = distance*normal
			point.co = p0
			offset = Matrix.Translation(point.handle_right)
			point.handle_right = offset@position
			point.handle_right = (point.handle_right - point.co) + p0

			next_point = bezier_points[index+1]
			normal = rmfs[(index*3)+3][-1]
			offset = Matrix.Translation(next_point.co)
			position = distance*normal
			next_point.co = p3
			offset = Matrix.Translation(next_point.handle_left)
			next_point.handle_left = offset@position
			next_point.handle_left = (next_point.handle_left - next_point.co) + p3
			
			# Approximate right handle 
			handle_right = self.find_best_handle_length((
			 p0, point.handle_right, next_point.handle_left, p3),
			 True,
			 Matrix.Translation(rmfs[(index*3)+1][0])@(distance*rmfs[(index*3)+1][-1]),
			 )
			
			if handle_right:
				point.handle_right = handle_right

			# Approximate left handle
			handle_left = self.find_best_handle_length((
			 p0, point.handle_right, next_point.handle_left, p3),
			 False,
			 Matrix.Translation(rmfs[(index*3)+2][0])@(distance*rmfs[(index*3)+2][-1]),
			 )

			if handle_left:
				next_point.handle_left = handle_left
			
		# Fix the first and last point's idle handles
		point_first, point_last = (bezier_points[0], bezier_points[-1])
		
		normal = rmfs[0][-1]
		position = distance*normal
		offset = Matrix.Translation(point_first.handle_left)
		point_first.handle_left = offset@position
		point_first.handle_left = ((point_first.handle_left - point_first.co ) + bezier_points_lookup[0][0])

		normal = rmfs[-1][-1]
		position = distance*normal		
		offset = Matrix.Translation(point_last.handle_right)
		point_last.handle_right = offset@position
		point_last.handle_right = ((point_last.handle_right - point_last.co) + bezier_points_lookup[-1][-1])

		set_pivot(curve, pivot)
		# bpy.ops.object.mode_set(mode='EDIT')
			
		return {'FINISHED'}

class BT_Remove(Operator, BT_Cursor):
	bl_idname = 'curve.bt_remove_point'
	bl_label = 'Remove Point'
	bl_description = 'Remove a Bézier point'
	bl_options = {'REGISTER', 'UNDO'}
	# invert_left: bpy.props.BoolProperty(name='Invert Left', description='Invert handle on the left', options={'SKIP_SAVE'})
	# invert_right: bpy.props.BoolProperty(name='Invert Right', description='Invert handle on the right', options={'SKIP_SAVE'})

	@classmethod
	def poll(cls, context):
		return context.object is not None and context.mode == 'EDIT_CURVE'

	def execute(self, context):
		curve = context.object
		set_handle_type(self, curve, 'FREE')

		if not is_bezier(curve):
			self.report({'ERROR'}, "Can only remove Bézier points")
			return{'CANCELLED'}

		if not len(curve.data.splines) == 1:
			self.report({'ERROR'}, self.bl_label + ': Can only remove points on Bézier curves with a single segment')	
			return{'CANCELLED'}			

		bezier_points = curve.data.splines[0].bezier_points
		points = [point for point in bezier_points if point.select_control_point]

		if not len(points) == 1:
			self.report({'ERROR'}, self.bl_label + ': Select a single control point. It cannot be a handle')
			return {'CANCELLED'}

		point , = points

		# Get point's index	
		index = 0
		for i, p in enumerate(bezier_points):
			if p == point:
				index = i
				break

		if index == 0 or index == len(bezier_points)-1:
			bpy.ops.curve.delete(type='VERT')
			return {'FINISHED'}

		# get t
		co = point.co
		hl = point.handle_left - co
		hr = co - point.handle_right

		full_length = (point.handle_left - point.handle_right).length

		if not full_length > 0:
			self.report({'ERROR'}, self.bl_label + ': Zero divizion!')
			return {'CANCELLED'}

		t = hl.length/full_length

		if t == 0 or t == 1:
			self.report({'ERROR'}, self.bl_label + ': Handles should not be in the control point\'s position!')
			return {'CANCELLED'}

		# remove point
		bpy.ops.curve.delete(type='VERT')

		bezier_points = curve.data.splines[0].bezier_points
		p0 = bezier_points[index-1]
		p1 = bezier_points[index]
		p0.handle_right = p0.co + ((p0.handle_right - p0.co)*(1/t))
		p1.handle_left = p1.co + ((p1.handle_left - p1.co)*(1/(1-t)))

		# if self.invert_right:
		# 	p1.handle_left = Matrix.Translation(p1.co) @ (p1.co - p1.handle_left)

		# if self.invert_left:
		# 	p0.handle_right = Matrix.Translation(p0.co) @ (p0.co - p0.handle_right)

		return {'FINISHED'}

class BT_Blend(Operator):
	bl_idname = 'curve.bt_blend_bezier'
	bl_label = "Blend 2x0"
	bl_description = 'Build an array of interpolated Bézier curves between two Rails. Takes 2 Bézier curves opposite to each other'
	bl_options = {'REGISTER', 'UNDO'}
	count: bpy.props.IntProperty(default=3, min=1, name = 'Density')

	@classmethod
	def poll(cls, context):
		sel = context.selected_objects
		return  context.active_object is not None and len(sel) == 2 and is_bezier(sel[0]) and is_bezier(sel[1])

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'count')

	def execute(self, context):
		curve_1 = context.active_object
		curve_2 = [curve for curve in context.selected_objects if curve is not curve_1][0]

		if not is_equal_n(curve_1, curve_2):
			self.report({'ERROR'}, "Source curves must have equal number of points!")
			return{'CANCELLED'}
		
		bpy.ops.object.mode_set(mode='OBJECT')
		bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)		

		dot = (to_world(self, curve_1.matrix_world, curve_1.data.splines[0].bezier_points[0].co) - to_world(self, curve_1.matrix_world, curve_1.data.splines[0].bezier_points[1].co)).dot(to_world(self, curve_2.matrix_world, curve_2.data.splines[0].bezier_points[0].co) - to_world(self, curve_2.matrix_world, curve_2.data.splines[0].bezier_points[1].co))
		if dot < 0:
			reverse_curve(self, curve_2)		

		blends = blend_bezier(self, context, self.count, BT_BezierCurve(curve_1), BT_BezierCurve(curve_2))
		
		for blend in blends:
			blend.build(context, curve_1.data.splines[0].resolution_u, 'BézierCurve')       

		if bpy.ops.object.select_all.poll():
			bpy.ops.object.select_all(action='DESELECT')        
		for curve in (curve_1, curve_2):
			curve.select_set(True)
		context.view_layer.objects.active = curve_1

		return {'FINISHED'}

class BT_Reverse(Operator):
	bl_idname = "curve.bt_reverse_curve"
	bl_label = "Reverse"
	bl_description = "Reverse Curve"
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):
		return bpy.context.object and bpy.context.object.type =='CURVE'

	def execute(self, context):
		for curve in context.selected_objects:
			if curve.type == 'CURVE':
				reverse_curve(self, curve)

		return {'FINISHED'}

class BT_CalcCurveLength(Operator):
	bl_idname = 'curve.bt_calculate_curve_length'
	bl_label = "Get Length"
	bl_description = 'Calculate curve length'
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):
		obj = context.object
		return obj is not None and obj.type == 'CURVE'  

	def execute(self, context):
		curve = context.object       
		self.report({'INFO'}, str(round(calculate_curve_length(self, curve), 5)))
		return {'FINISHED'}

class BT_SetCurveLength(Operator):
	bl_idname = 'curve.bt_set_curve_length'
	bl_label = "Set Length"
	bl_description = 'Set curve length'
	bl_options = {'REGISTER', 'UNDO'}
	input_length: bpy.props.FloatProperty(default=1.0, min=0, name='Set Length: ')
	
	@classmethod
	def poll(cls, context):
		obj = context.object
		return obj is not None and obj.type == 'CURVE'
		
	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'input_length')

	def execute(self, context):
		for curve in context.selected_objects:
			# pivot = curve.location.copy()           
			length = calculate_curve_length(self, curve)
			if not length > 0:
				self.report({'WARNING'}, "Zero-length curves can't be scaled!")
				return {'CANCELLED'}

			scale = self.input_length/length

			if curve.type == 'CURVE':
				for spline in curve.data.splines:
					if spline.type == 'POLY' or spline.type == 'NURBS':             
						points = [Vector(point.co[:3]) for point in spline.points]              
						for point in spline.points:                     
							point.co *= scale
					
					elif spline.type == 'BEZIER':
						set_handle_type(self, curve, 'FREE')
						points = [point for point in spline.bezier_points]                                          
						for point in points:                        
							point.co *= scale
							point.handle_left *= scale
							point.handle_right *= scale

				# set_pivot(curve, pivot)
				# curve.location = pivot

		return {'FINISHED'}

class BT_Blend1Profile2Rails(Operator):
	bl_idname = 'curve.bt_blend_1_profile_2_rails'
	bl_label = 'Blend 2x1'
	bl_description = 'Build an array of interpolated Bézier curves. Takes 3 Bézier curves:: 2 Rails and 1 Profile (active object)'
	bl_options = {'REGISTER', 'UNDO'}
	count: bpy.props.IntProperty(default=12, min=1, name='Count')
	precision: bpy.props.IntProperty(name='Precision', min=1, max=100, default=10, description='Resolution of constraint curve. Low - may lead to missing points, high - to slow calculation')
	search_limit: bpy.props.IntProperty(default=5, min=1, max=100, name = 'Search Limit', description='Limit of searching curves common points. Increase or decrease in case of search warnings')
		
	@classmethod
	def poll(cls, context):
		return context.active_object is not None and len(context.selected_objects) == 3
	
	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'count')
		column.prop(self, 'precision')	

	def execute(self, context):		
		if not context.active_object or not is_bezier(context.active_object):
			self.report({'ERROR'}, "Active object must be the Profile curve for the Rails!")
			return{'CANCELLED'}

		sel = context.selected_objects   

		for curve in sel:
			if not is_bezier(curve):        
				self.report({'ERROR'}, "Expected selection of 3 Bézier curves!")
				return{'CANCELLED'}

		# bpy.ops.object.mode_set(mode='OBJECT')
		# bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)		

		# points_map
		points_map = {}
		for curve in sel:
			points_map[curve] = []
			for point in curve.data.splines[0].bezier_points:
				points_map[curve].append(to_world(self, curve.matrix_world, point.co.copy().freeze()))

		path = context.active_object
		rail_1 = None
		rail_2 = None

		need_reverse = []

		if path not in points_map.keys():
			self.report({'ERROR'}, "Construction data is not valid! Possible reason: context object is not in expected Path position between 2 Rails")
			return {'CANCELLED'}

		# find the closest curve to path start point    
		for curve, coords in points_map.items():
			if curve is not path:           
				if is_equal(points_map[path][0], coords[0], self.search_limit):
					rail_1 = curve
				elif is_equal(points_map[path][0], coords[-1], self.search_limit):
					need_reverse.append(curve)
					rail_1 = curve

				if is_equal(points_map[path][-1], coords[-1], self.search_limit):
					need_reverse.append(curve)
					rail_2 = curve
				elif is_equal(points_map[path][-1], coords[0], self.search_limit):
					rail_2 = curve

		for curve in need_reverse:
			reverse_curve(self, curve)

		for curve in (path, rail_1, rail_2):
			if curve is None:
				self.report({'ERROR'}, "Construction data is not valid. Path curve must be the scene active object. Check if the patch perimeter is enclosed and there are no gaps between curves")
				return {'CANCELLED'}
		
		if len(rail_1.data.splines[0].bezier_points) != len(rail_2.data.splines[0].bezier_points):
			self.report({'ERROR'}, "Rails must have equal number of points")
			return {'CANCELLED'}            
		
		# snap points		
		interpolated_points = space_interpolate_bezier(path, self.precision, self.count+1)[1:-1]

		blends = blend_bezier(self, context, len(interpolated_points), BT_BezierCurve(rail_1), BT_BezierCurve(rail_2))
		curves = []

		resolution = rail_1.data.splines[0].resolution_u

		for index, blend in enumerate(blends):
			curve = blend.build(context, resolution, 'BézierCurve', is_set_pivot=False)
			p0 = curve.data.splines[0].bezier_points[0]
			target = interpolated_points[index]
			translation = Matrix.Translation(target-p0.co)
			p0.co = translation @ p0.co
			p0.handle_right = translation @ p0.handle_right
			p0.handle_left = translation @ p0.handle_left       

		for obj in context.selected_objects:
			obj.select_set(False)

		for curve in (path, rail_1, rail_2):
			curve.select_set(True)

		context.view_layer.objects.active = path

		return {'FINISHED'}

def blend_2_profiles_2_rails(self, context, count, *, curves):
	sel = curves

	# here, paths are rails used as side limits for profiles
	# path1 is the first element in the list
	path1 = sel[0]
	path2 = sel[1]

	# source profiles to generate blends between them
	profile1 = sel[2]
	profile2 = sel[3]

	resolution = path1.data.splines[0].resolution_u

	path1_interp_points = space_interpolate_bezier(path1, self.precision, count)[1:-1]
	path2_interp_points = space_interpolate_bezier(path2, self.precision, count)[1:-1]

	# for index, point in enumerate(path1_interp_points):
	# 	path1_interp_points[index] = path1.matrix_world.inverted()@point

	# for index, point in enumerate(path2_interp_points):
	# 	path2_interp_points[index] = path2.matrix_world.inverted()@point

	# blends
	blends = blend_bezier(self, context, len(path1_interp_points), BT_BezierCurve(profile1), BT_BezierCurve(profile2))

	curves = []

	# fit blends 
	for index, blend in enumerate(blends):
		curve = blend.build(context, resolution, 'BézierCurve', is_set_pivot=False)
		p_first = curve.data.splines[0].bezier_points[0]
		offset = Matrix.Translation(curve.matrix_world.inverted()@-p_first.co)

		if index < len(path1_interp_points):
			curve.data.transform(Matrix.Translation(offset@path1_interp_points[index]))

		if index < len(path2_interp_points):
			p_last = curve.data.splines[0].bezier_points[-1]
			target = path2_interp_points[index]
			offset = Matrix.Translation(curve.matrix_world.inverted()@(target - p_last.co))
			p_last.co = target

			p_last.handle_right = offset @ p_last.handle_right
			p_last.handle_left = offset @ p_last.handle_left

		curves.append(curve)
	
	return curves

class BT_Blend2Profiles2Rails(Operator):
	bl_options = {'REGISTER', 'UNDO'}
	bl_idname = 'curve.bt_blend_2_profiles_2_rails'
	bl_label = 'Blend 2x2'
	bl_description = 'Build an array of interpolated Bézier curves. Takes 4 Bézier curves: 2 Rails and 2 Profiles'
	count: bpy.props.IntProperty(default=1, min=1, soft_min=1, name = 'Count')
	precision: bpy.props.IntProperty(name='Precision', min=1, max=100, default=10, description='Resolution of constraint curve. Low - may lead to missing points, high - to slow calculation')
	search_limit : bpy.props.IntProperty(default=5, min=1, max=5, name = 'Search Limit', description='Limit of searching curves common points. Increase or decrease in case of search warnings')
	
	@classmethod
	def poll(cls, context):
		return context.active_object is not None and len(context.selected_objects) == 4

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'count')
		column.prop(self, 'precision')	
		column.prop(self, 'search_limit')

	def execute(self, context):
		sel = context.selected_objects      
		
		if not context.active_object or not is_bezier(context.active_object):
			self.report({'ERROR'}, "Active object must be one of the selected loop curves!")
			return{'CANCELLED'}     

		for obj in sel:
			if not is_bezier(obj):
				self.report({'ERROR'}, "Required a loop of 4 separate Bézier curves!")
				return{'CANCELLED'}

		# bpy.ops.object.mode_set(mode='OBJECT')
		# bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)		

		points_map = {}
		for curve in sel:
			points_map[curve] = []
			for point in curve.data.splines[0].bezier_points:
				points_map[curve].append(to_world(self, curve.matrix_world, point.co.copy().freeze()))

		horizon_1 = context.active_object
		horizon_2 = None
		vertical_1 = None
		vertical_2 = None

		need_reverse = []

		if horizon_1 not in points_map.keys():
			self.report({'ERROR'}, "Construction data is not valid! Possible reason: context object is not one of 4 loop curves")
			return {'CANCELLED'}

		# find the closest curve to horizon_1 start point   
		for curve, coords in points_map.items():
			if curve is not horizon_1:                                  
				if is_equal(points_map[horizon_1][0], coords[0], self.search_limit):
					vertical_1 = curve                          
				elif is_equal(points_map[horizon_1][0], coords[-1], self.search_limit):
					need_reverse.append(curve)              
					vertical_1 = curve                      
									
				if is_equal(points_map[horizon_1][-1], coords[-1], self.search_limit):
					need_reverse.append(curve)
					vertical_2 = curve                                      
				elif is_equal(points_map[horizon_1][-1], coords[0], self.search_limit):
					vertical_2 = curve

				if  (not is_equal(points_map[horizon_1][0], coords[0], self.search_limit) and not is_equal(points_map[horizon_1][0], coords[-1], self.search_limit)) and (not is_equal(points_map[horizon_1][-1], coords[0], self.search_limit) and not is_equal(points_map[horizon_1][-1], coords[-1], self.search_limit)):
					horizon_2 = curve

		for curve in need_reverse:
			reverse_curve(self, curve)

		for curve in (horizon_1, vertical_1, vertical_2, horizon_2):
			if curve is None:
				self.report({'ERROR'}, "Construction data is not valid. Check if the curve loop is enclosed and there are no gaps between curves")
				return {'CANCELLED'}
		
		if len(horizon_1.data.splines[0].bezier_points) != len(horizon_2.data.splines[0].bezier_points) or len(vertical_1.data.splines[0].bezier_points) != len(vertical_2.data.splines[0].bezier_points):
			self.report({'ERROR'}, "Parallel curves must have equal number of points")
			return {'CANCELLED'}
		
		points_map.clear()
		for curve in (horizon_1, vertical_1, vertical_2, horizon_2):
			points_map[curve] = []
			for point in curve.data.splines[0].bezier_points:
				points_map[curve].append(to_world(self, curve.matrix_world, point.co.copy().freeze()))

		if not is_equal(points_map[vertical_2][-1], points_map[horizon_2][-1], self.search_limit):
			reverse_curve(self, horizon_2)

		blend_2_profiles_2_rails(self, context, self.count+1, curves=(horizon_1, horizon_2, vertical_1, vertical_2))
		
		if bpy.ops.object.select_all.poll():
			bpy.ops.object.select_all(action='DESELECT')        
		
		for curve in (horizon_1, horizon_2, vertical_1, vertical_2):
			curve.select_set(True)
		context.view_layer.objects.active = horizon_1
		
		return{'FINISHED'}

def add_polyline(self, context, coords, name, *, is_closed=False):
	sel = context.selected_objects
	
	if bpy.ops.object.select_all.poll():
		bpy.ops.object.select_all(action='DESELECT')

	curve = bpy.data.curves.new(name="Polyline", type='CURVE')
	curve.dimensions = '3D'
	polyline = bpy.data.objects.new(name, curve)
	
	if len(coords) > 0:
		spline = curve.splines.new('POLY')
		
		spline.points.add(len(coords)-1)		
		
		for index, coord in enumerate(coords):
			if spline.points[index] is not None:
				spline.points[index].co[:3] = coord
	
	context.scene.collection.objects.link(polyline)
	polyline.select_set(True)
	context.view_layer.objects.active = polyline
	bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')

	polyline.data.bevel_depth = context.scene.bt_pipe_radius
	polyline.data.bevel_resolution = context.scene.bt_pipe_resolution
	polyline.data.extrude = context.scene.bt_band_width

	if not is_closed:
		set_pivot(polyline, polyline.matrix_world@polyline.data.splines[0].points[0].co.xyz)

	for obj in sel:
		obj.select_set(True)	

	return polyline

def add_polyline_spline(self, context, curve, data):
	# add a new spline to the existing polyline
	# data is a list of points
	spline = curve.data.splines.new('POLY')
	spline.points.add(len(data)-1)

	for index, point in enumerate(data):
		spline.points[index].co = point

	return spline

def add_bezier(self, context, resolution, name):
	sel = context.selected_objects
	# bpy.ops.object.select_all(action='DESELECT')

	curve = bpy.data.curves.new(name="Curve", type='CURVE')
	curve.dimensions = '3D'
	bezier = bpy.data.objects.new(name, curve)
	spline = curve.splines.new('BEZIER')
	spline.resolution_u = resolution    
	context.scene.collection.objects.link(bezier)
	# bezier.select_set(True)
	context.view_layer.objects.active = bezier
	
	for obj in sel:
		obj.select_set(True)

	return bezier

def create_bezier(self, context, data, resolution, name):
	bezier = add_bezier(self, context, resolution, name)
	spline = bezier.data.splines[0]
	points = bezier.data.splines[0].bezier_points

	# if n-degree Bézier has more than 2 points
	if isinstance(data, list):
		# <data> is a list of chunks of vectors
		# each chunk is a portion of n-degree spline
		spline.bezier_points.add(len(data))
	
		for index, chunk in enumerate(data):
			points[index].co = chunk.p0_co          
			points[index].handle_right = chunk.p0_handle_right          
			points[index+1].co = chunk.p1_co
			points[index+1].handle_left = chunk.p1_handle_left  
	
	else: # standard 2-point cubic bezier
		spline.bezier_points.add(1)
		points[0].co = data.p0_co
		points[0].handle_left = data.p0_handle_left
		points[0].handle_right = data.p0_handle_right
		points[1].co = data.p1_co
		points[1].handle_left = data.p1_handle_left
		points[1].handle_right = data.p1_handle_right

	return bezier

def add_bezier_spline(self, context, curve, data, resolution):
	# add a new spline to the existing bezier curve
	spline = curve.data.splines.new('BEZIER')
	points = spline.bezier_points  
	if isinstance(data, list):
		points.add(len(data))           
		for index, chunk in enumerate(data):
			points[index].co = chunk.p0_co          
			points[index].handle_right = chunk.p0_handle_right      
			points[index+1].co = chunk.p1_co
			points[index+1].handle_left = chunk.p1_handle_left
	else:
		spline.bezier_points.add(1)
		points[0].co = data.p0_co
		points[0].handle_left = data.p0_handle_left
		points[0].handle_right = data.p0_handle_right
		points[1].co = data.p1_co
		points[1].handle_left = data.p1_handle_left
		points[1].handle_right = data.p1_handle_right

	# fix idle handles
	points[0].handle_left = Matrix.Translation(points[0].co) @ points[0].co - points[0].handle_right
	points[-1].handle_right = Matrix.Translation(points[-1].co) @ points[-1].co - points[-1].handle_left

	spline.resolution_u = resolution
	return spline

def blend_bezier(self, context, count, curve1, curve2):
	if count == 0:
		return []
	
	blends = []

	count += 1
	for index in range(count):
		if index == 0:
			continue

		t = index/count

		interpolated_points = []
		for i, point in enumerate(curve1.points):           
			interpolated_points.append([
				interpolate_2_vectors(self, point[0], curve2.points[i][0], t),
				interpolate_2_vectors(self, point[1], curve2.points[i][1], t),              
				interpolate_2_vectors(self, point[2], curve2.points[i][2], t),
				])
		
		blends.append(BT_BezierCurve(interpolated_points))

	return blends

def rebuild_bezier(self, context, curve):   
	name = copy(curve.name)
	resolution = copy(curve.data.splines[0].resolution_u)   
	data = BT_BezierCurve(curve)    
	new_curve = data.create_curve(context, resolution)
	bpy.data.objects.remove(curve)
	new_curve.name = name
	new_curve.data.splines[0].resolution_u = resolution
	return new_curve

def sort_by_distance(self, obj, curves):
	# obj is the starting point for measuring distances and should be the first and active curve in the selection set
	get_origin = lambda p0, p1 : (p0 + p1)/2
	obj_points = obj.data.splines[0].bezier_points if obj.data.splines[0].type == 'BEZIER' else (obj.data.splines[0].points[0], obj.data.splines[0].points[-1])
	origin = get_origin(obj.matrix_world @ obj_points[0].co, obj.matrix_world @ obj_points[1].co) if obj.data.splines[0].type == 'BEZIER' else get_origin(obj.matrix_world @ Vector((obj_points[0].co[:3])), obj.matrix_world @ Vector((obj_points[-1].co[:3])))

	distance_map = dict()

	for curve in curves:
		points = (curve.data.splines[0].bezier_points[0].co, curve.data.splines[0].bezier_points[1].co) if curve.data.splines[0].type == 'BEZIER' else (Vector((curve.data.splines[0].points[0].co[:3])), Vector((curve.data.splines[0].points[-1].co[:3])))
		pivot = get_origin(curve.matrix_world @ points[0], curve.matrix_world @ points[1])      
		distance_map[curve] = get_distance(origin, pivot)
	
	comp = lambda c1: distance_map.get(c1)
	curves.sort(key=comp)

def bt_transfer_curve_data(self, source, targets):
	for curve in targets:
		if curve.type == 'CURVE':
			curve.data.bevel_depth = source.data.bevel_depth 
			curve.data.bevel_factor_end = source.data.bevel_factor_end 
			curve.data.bevel_factor_mapping_end = source.data.bevel_factor_mapping_end 
			curve.data.bevel_factor_mapping_start = source.data.bevel_factor_mapping_start 
			curve.data.bevel_factor_start = source.data.bevel_factor_start 
			curve.data.bevel_mode = source.data.bevel_mode 
			curve.data.bevel_object = source.data.bevel_object
			curve.data.bevel_resolution = source.data.bevel_resolution 
			curve.data.dimensions = source.data.dimensions 
			curve.data.extrude = source.data.extrude 
			curve.data.fill_mode = source.data.fill_mode     
			curve.data.offset = source.data.offset 
			curve.data.render_resolution_u = source.data.render_resolution_u 
			curve.data.render_resolution_v = source.data.render_resolution_v 
			curve.data.splines[0].resolution_u = source.data.splines[0].resolution_u
			curve.data.splines[0].resolution_v = source.data.splines[0].resolution_v     
			curve.data.use_fill_caps = source.data.use_fill_caps 
			curve.data.use_map_taper = source.data.use_map_taper
			curve.data.use_path = source.data.use_path 
			curve.data.use_path_clamp = source.data.use_path_clamp 
			curve.data.use_path_follow = source.data.use_path_follow 
			curve.data.use_radius = source.data.use_radius
			curve.data.use_stretch = source.data.use_stretch
			curve.color = source.color

	bpy.ops.object.make_links_data(type='MODIFIERS')	
	bpy.ops.object.make_links_data(type='MATERIAL')

class BT_TransferCurveData(Operator):
	bl_idname = "curve.bt_transfer_curve_data"
	bl_label = "Transfer Curve Data"
	bl_description = "Transfer curve data from active curve"
	bl_options = {'REGISTER', 'UNDO'}

	@classmethod
	def poll(cls, context):
		return context.active_object is not None and context.active_object.type =='CURVE' and len(context.selected_objects) > 1

	def execute(self, context):
		source = bpy.context.active_object
		targets = [obj for obj in context.selected_objects if obj.type == 'CURVE']
		bt_transfer_curve_data(self, source, targets)
		return {'FINISHED'}

def interpolate_bezier_classic_formula(curve, t):
	spline = curve.data.splines[0]
	points = spline.bezier_points
	p0=points[0], handle_right=points[0].handle_right, handle_left=points[1].handle_left, p3=points[1]
	return pow((1 - t), 3)*p0.co + 3*(pow((1 - t), 2))*t*handle_right + 3*(1 - t)*pow(t, 2)*handle_left + pow(t, 3)*p3.co

class BT_BezierInterpolate(Operator):
	bl_idname = 'curve.bt_bezier_interpolate'
	bl_label = 'Bezier Interpolate'
	bl_description = 'Spawn empties at interpolated points'
	bl_options = {'REGISTER', 'UNDO'}
	segments_count: bpy.props.IntProperty(name='Segments Count', min=2, max=64, default=10, description='Number of interpolated segments')
	precision: bpy.props.IntProperty(name='Precision', min=1, max=100, default=10, description='Resolution of constraint curve. Low - may lead to missing points, high - to slow calculation')
	empty_radius:bpy.props.FloatProperty(name='Radius', min=0.0, default=0.01, description='Radius of empties')
	exact: bpy.props.BoolProperty(name='Exact', description='Spawn empties without calculated spacing at their real positions based on spline resolution')

	@classmethod
	def poll(cls, context):
		return context.object is not None and is_bezier(context.object)

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		if not self.exact:
			column.prop(self, 'segments_count')
			column.prop(self, 'precision')		
		column.prop(self, 'empty_radius')
		column.prop(self, 'exact', toggle=True)	

	def execute(self, context):
		context.evaluated_depsgraph_get()		
		curve = context.object
		if not is_bezier(curve):
			return{'CANCELLED'}

		interpolated_points = mathutils_interpolate_n_bezier_points(curve, curve.data.splines[0].resolution_u+1) if self.exact else space_interpolate_bezier(curve, self.precision, self.segments_count, debug=False)

		for point in interpolated_points:
			spawn_empty('Point', point, size=self.empty_radius)

		return {'FINISHED'}

class BT_Convert(Operator):
	bl_idname = 'object.bt_convert'
	bl_label = 'Convert'
	bl_description = 'Convert objects to another type. Types can be Bézier, Polyline or Mesh'
	bl_options = {'REGISTER', 'UNDO'}
	type: bpy.props.EnumProperty(items=[('Polyline', 'Polyline', ''), ('Bezier', 'Bézier', '')], name='Type')
	remove_src: bpy.props.BoolProperty(name='Remove source', default=True)
	resolution: bpy.props.IntProperty(name='Resolution', default = 12, min=2, soft_min=2)
	keep_all_points: bpy.props.BoolProperty(name='Keep All Points', default=True, description='Set all points to Bézier explicitly with specified handles')
	handle_type: bpy.props.EnumProperty(items=[('AUTO', 'AUTO', ''), ('FREE', 'FREE', ''), ('VECTOR', 'VECTOR', ''), ('ALIGNED', 'ALIGNED', '')], name='Handle: ')
	to_wireframe: bpy.props.BoolProperty(name='Wireframe', description='Converts the result to a mesh wireframe object')
	to_face: bpy.props.BoolProperty(name='Face', description='Converts the result to a mesh single-face object')
	exact: bpy.props.BoolProperty(name='Exact', description='No spacing. Keep existing Bézier interpolation')

	do_not_remove = []

	@classmethod
	def poll(cls, context):
		return context.object is not None
	
	def draw(self, context):
		layout = self.layout
		column = layout.column(align=True)
		column.prop(self, 'type')
		column.separator(factor=1.0)
		if self.type == 'Bezier':
			column.prop(self, 'keep_all_points')
			if self.keep_all_points:
				column.separator(factor=1.0)
				column.prop(self, 'handle_type')
				
			column.separator(factor=1.0)

		row = column.row(align=True)
		row.label(text='Resolution:')
		row.prop(self, 'resolution', text='')
		column.separator(factor=1.0)

		column = layout.column(align=True)
		row = column.row(align=True)
		row.label(text='To Mesh: ')

		if not self.to_face:
			row.prop(self, 'to_wireframe', text='Wireframe', toggle=True)
			column.separator(factor=1.0)
		if not self.to_wireframe:
			row.prop(self, 'to_face', text='Face', toggle=True)
			column.separator(factor=1.0)

		column.prop(self, 'exact')
		column.prop(self, 'remove_src')	

	# find the handle for a square bezier 
	def get_square_bezier_handle(self, point, t, p0, p2):
		if (2*(1-t)*t) == 0:
			print ('Zero division in get_square_bezier_handle!')
			return Vector()
		return ((point - pow((1-t), 2)*p0 - pow(t, 2)*p2)) / (2*(1-t)*t)

	def poly_to_bezier(self, context, curve, spline):
		points = [Vector((point.co[:3])) for point in spline.points]
	
		# if we have only 3 points, we will make a simple quadratic to cubic bezier conversion
		# we will calculate a quadratic bezier from given points, then
		# use that data to calculate a cubic bezier
		if len(points) == 3:
			p0 = points[0]
			point = points[1]           
			p2 = points[2]
			
			len1 = get_distance(p0, point)
			len2 = get_distance(point, p2)
			length = len1 + len2
			t = len1/length

			handle = self.get_square_bezier_handle(point, t, p0, p2)

			data =  BT_BezierCurve((
					p0,
					p0,
					((handle - p0)*2/3)+p0,
					p2,
					((handle - p2)*2/3)+p2,
					p2  
					))
			
			spline = add_bezier_spline(self, context, curve, data, self.resolution)
			
			if spline is None:
				return None

		else:
			chunk_points = []
			for index, point in enumerate(points):          
				if index%3 == 0:
					chunk_points.append((points[index:index+4]))

			data = []
			leftover = []
			
			for points in chunk_points:         
				# leftover chunk
				if len(points) < 4:
					for point in points[1:]:
						leftover.append(point)                  
					break                           

				# t1, t2 = calculate_t_from_bezier_interpolated_points_cubic(self, points)

				handles = calculate_bezier_handles(					
					points,
					1/3, 2/3
					# t1, t2
				)

				if len(handles) == 0:
					return None

				data.append (BT_BezierCurve((
					points[0],
					points[0],
					handles[0],
					points[3],
					handles[1],
					points[3]
					)))

			# now we can add a new spline
			spline = add_bezier_spline(self, context, curve, data, self.resolution)
			
			if spline is None:
				return None

			# cleaning up
			points = spline.bezier_points
			points[0].handle_left = points[0].co
			points[-1].handle_right = points[-1].co

			# leftover points 
			if len(leftover) > 0:               
				# if only 1 point, it is converted explicitly
				if len(leftover) == 1:
					# points[-1].handle_left_type = 'AUTO'
					# points[-1].handle_right_type = 'AUTO'             
					points.add(1)
					points[-1].co = point
					points[-1].handle_left_type = 'AUTO'
					points[-1].handle_right_type = 'AUTO'
					points[-1].handle_left_type = 'ALIGNED'
					points[-1].handle_right_type = 'ALIGNED'			
					points[-2].handle_right = Matrix.Translation(points[-2].co) @ points[-2].co - points[-2].handle_left                              

				elif len(leftover) == 2:
					# we will calculate a quadratic bezier from 3 leftover points, then
					# use that data to calculate a cubic bezier
					point = leftover[0]
					p0 = points[-1].co
					p2 = leftover[1]
					
					len1 = get_distance(p0, point)
					len2 = get_distance(point, p2)
					length = len1 + len2
					t = len1/length

					handle = self.get_square_bezier_handle(point, t, p0, p2)                

					# 2/3 handle shortening rule for conversion from quadratic to cubic bezier
					# https://upload.wikimedia.org/wikipedia/commons/thumb/d/d5/Quadratic_to_cubic_Bezier_curve.svg/330px-Quadratic_to_cubic_Bezier_curve.svg.png
					points[-1].handle_right = ((handle - p0)*2/3)+p0    
					points.add(1)
					points[-1].co = p2
					points[-1].handle_left = ((handle - p2)*2/3)+p2			

			# fix idle handles
			points[0].handle_left = Matrix.Translation(points[0].co) @ points[0].co - points[0].handle_right
			points[-1].handle_right = Matrix.Translation(points[-1].co) @ points[-1].co - points[-1].handle_left			 

			# # convert all handles to ALIGNED type
			# if len(spline.bezier_points) > 2:     
			#   for index, point in enumerate(spline.bezier_points):
			#       if index>0 and ((index + 1) % len(points) != 0):
			#           point.handle_left_type = 'ALIGNED'
			#           point.handle_right_type = 'ALIGNED'

		return spline
	
	def bezier_to_poly(self, context, curve, spline):		
		bezier_points = spline.bezier_points
		interpolated_points = []		
		matrix=curve.matrix_world
		poly_points = [matrix.inverted()@point.to_4d() for point in (mathutils_interpolate_n_bezier_points(curve, self.resolution) if self.exact else space_interpolate_bezier(curve, 1000, self.resolution))]
		spline = add_polyline_spline(self, context, curve, poly_points)
		return spline

	def explicit_to_bezier(self, spline, handle_type):
		spline.type = 'BEZIER'
		points = spline.bezier_points
		for point in points:
			point.handle_left_type = handle_type
			point.handle_right_type = handle_type
		spline.resolution_u = self.resolution

	def add_curve_copy(self, curve):
		new_curve = curve.copy()
		bpy.context.scene.collection.objects.link(new_curve)
		bpy.context.view_layer.objects.active = new_curve		
		new_curve.color = bpy.context.scene.bt_color
		new_curve.select_set(True)		
		return new_curve

	def mesh_to_curve(self, obj):
		curve_data = bpy.data.curves.new('Curve', 'CURVE')
		curve_data.dimensions = '3D'		
		curve = bpy.data.objects.new('ConvertedCurve', curve_data)
		spline = curve.data.splines.new('POLY')
		bpy.context.scene.collection.objects.link(curve)

		curve.matrix_world = obj.matrix_world		
		mesh = obj.data		

		bm = bmesh.new()
		verts = bm.verts
		edges = bm.edges
		bm.from_mesh(mesh)

		verts.index_update()
		verts.ensure_lookup_table()		
		edges.index_update()
		edges.ensure_lookup_table()	
		
		# find start_vert as an entry point	for search	
		start_vert = verts[0] # if closed loop
		is_closed_loop = True # if not closed loop		
		for vert in verts:
			if len(vert.link_edges) == 1:
				start_vert = vert
				is_closed_loop = False
				break	

		# find start edge
		start_edge = [edge for edge in edges if (start_vert in edge.verts)][0]

		ordered_edges = []
		ordered_verts = [start_vert.index]

		def find_next_edge_recursive(edge, vert):
			if edge is None:
				return None
			
			if not edge.index in ordered_edges:
				ordered_edges.append(edge.index)

			if not vert.index in ordered_verts:
				ordered_verts.append(vert.index)

			link_edges = list(vert.link_edges)
			if len(link_edges) > 1:
				link_edges.remove(edge)
				next_edge = link_edges[0]

				# if loop
				if next_edge.index in ordered_edges:
					return None

				return find_next_edge_recursive(next_edge, next_edge.other_vert(vert))

			return None		
		
		find_next_edge_recursive(start_edge, start_edge.other_vert(start_vert))
				
		if len(ordered_verts) == len(verts):
			spline.points.add(len(verts)-1)
			for index, index_ordered in enumerate(ordered_verts):
				spline.points[index].co.xyz = verts[index_ordered].co.copy()

			if is_closed_loop:
				spline.points.add(1)
				spline.points[-1].co.xyz = spline.points[0].co.xyz

			if self.type == 'Bezier':
				if self.keep_all_points:
					self.explicit_to_bezier(spline, self.handle_type)
				else:
					new_spline = self.poly_to_bezier(bpy.context, curve, spline)
					curve.data.splines.remove(spline)

			bpy.context.view_layer.objects.active = curve
			curve.select_set(True)

		else:
			# self.report({'ERROR'}, self.bl_idname + ': Count of converted and original points is not the same!')			
			self.do_not_remove.append(obj)
		
		bm.free()

	def any_to_mesh(self, context, curve):
		data = bpy.data
		scene = context.scene
		mesh = data.meshes.new(curve.name + ' mesh')
		obj = bpy.data.objects.new(curve.name, mesh)
		scene.collection.objects.link(obj)
		matrix = curve.matrix_world

		bm = bmesh.new()
		bm.from_mesh(mesh)		
		
		edge_buffer = []
		points = (mathutils_interpolate_n_bezier_points(curve, self.resolution) if self.exact else space_interpolate_bezier(curve, 1000, self.resolution)) if is_bezier(curve) else [matrix@point.co.xyz for point in curve.data.splines[0].points]
			
		for index, point in enumerate(points):
			points[index] = matrix.inverted()@point

		for point in points:
			bm.verts.new(point)

		bm.verts.index_update()
		bm.verts.ensure_lookup_table()

		for index, vert in enumerate(bm.verts):
			if index + 1 != len(bm.verts):
				edge_buffer.append([vert, bm.verts[index+1]])

		for i in range(len(points)-1):
			bm.edges.new(edge_buffer[i])

		bm.edges.index_update()
		bm.edges.ensure_lookup_table()

		weld_map = bmesh.ops.find_doubles(bm, verts=bm.verts, dist=1e-5)
		bmesh.ops.weld_verts(bm, targetmap=weld_map['targetmap'])

		if self.to_face:
			face = bm.faces.new(
				[vert for vert in bm.verts]
				)
			bm.faces.index_update()
			bm.faces.ensure_lookup_table()
			face.normal_update()

			# new face should be turned towards view direction, hopefully
			if face.normal.dot(get_view_direction(self, context)) < 0:
				face.normal_flip()
				face.normal_update()

		bm.to_mesh(mesh)
		bm.free()
		obj.matrix_world = curve.matrix_world
		obj.select_set(True)
		context.view_layer.objects.active = obj

	def execute(self, context):		
		# if depsgraph is not updated, history sets object matrices to identity
		context.evaluated_depsgraph_get()
		
		sel = [obj for obj in context.selected_objects if obj.type in {'CURVE', 'MESH'}]
		if not len(sel) > 0:
			self.report({'ERROR'}, self.bl_idname + ': Nothing selected!')
			return {'CANCELLED'}	

		for obj in sel:
			if obj.type == 'MESH':
				self.mesh_to_curve(obj)						
				continue
			
			elif obj.type == 'CURVE':
				curve = obj
				
				if self.to_wireframe or self.to_face:										
					self.any_to_mesh(context, curve)					
					continue

				new_curve = self.add_curve_copy(curve)	

				for spline in new_curve.data.splines:
					if not spline.type in {'BEZIER', 'POLY'}:
						self.report({'WARNING'}, self.bl_idname + ": " + str(spline) + " unsupported type " + spline.type)
						continue

					if spline.type == 'POLY' and self.type == 'Bezier':
						# explicit conversion: standard poly to bezier control point conversion where each polyline point becomes a cubic bezier control point with auto handles
						if self.keep_all_points:
							self.explicit_to_bezier(spline, self.handle_type)
							continue

						# optimal conversion: interpolated bezier points will pass source polyline points where every 4 polypoints make 1 cubic bezier segment with 2 control points
						self.poly_to_bezier(context, new_curve, spline)									
						new_curve.data.splines.remove(spline)                                           

					elif spline.type == 'BEZIER' and self.type == 'Polyline':
						self.bezier_to_poly(context, new_curve, spline)
						new_curve.data.splines.remove(spline)

		if self.remove_src:			
			for obj in sel[:]:
				if obj not in self.do_not_remove:
					bpy.data.objects.remove(obj, do_unlink=True)

		self.do_not_remove.clear()

		return {'FINISHED'}

class BT_ChangeColor(Operator):
	bl_idname = 'object.bt_change_color'
	bl_label = 'Change Color'
	bl_description = 'Change Object Color of selected meshes and curves. Wireframe Color should be set to Object to see colored curves and object wireframe'
	bl_options = {'REGISTER', 'UNDO'}

	needs_update: bpy.props.BoolProperty(options={'SKIP_SAVE', 'HIDDEN'})

	@classmethod
	def poll(cls, context):
		return context.object is not None
	
	def draw(self, context):
		layout = self.layout
		column = layout.column()
		row = column.row(align=True)
		row.label(text='Color:')
		row.prop(self, 'color', text = "")	


	def change_color(self, context):
		self.needs_update=True
		return None

	color: bpy.props.FloatVectorProperty(name='Color', subtype='COLOR', size=4, min=0, max=1.0, default=(0,0,0,1), update=change_color)	
	
	def execute(self, context):
		self.sel = [obj for obj in context.selected_objects if obj.type in {'MESH', 'CURVE'}]
		if not len(self.sel):		
			self.report({'ERROR'}, self.bl_label + ': Nothing selected!')
			return {'CANCELLED'}	
		
		if self.needs_update:
			for obj in context.selected_objects:
				if obj.type in {'MESH', 'CURVE'}:					
					obj.color = self.color
			self.needs_update = False

		return {'FINISHED'}

# MESH OPS ######################################################################
def recalculate_face_normal(self, context, face):
	if face.normal.dot(get_view_direction(self, context).normalized()) < 0:
		face.flip()

def get_quad_loop_verts_data(self, curve_1_interp_points_buffer, curve_2_interp_points_buffer):
	curve_2_interp_points_buffer.reverse()
	vertex_buffer = []

	for point in curve_1_interp_points_buffer:
		vertex_buffer.append(point)

	for point in curve_2_interp_points_buffer:
		vertex_buffer.append(point)

	return vertex_buffer

def build_quad_loop(self, bm, loop):
	verts_buffer_size = len(loop)

	for index, vert in enumerate(loop):
		if index < (verts_buffer_size/2)-1:
			try:
				new_face = bm.faces.new(
					[
						loop[index],
						loop[index+1],
						loop[(verts_buffer_size-1)-(index+1)],
						loop[(verts_buffer_size-1)-index]
					]
				)
				recalculate_face_normal(self, bpy.context, new_face)
			except:				
				pass

		bm.faces.index_update()
		bm.faces.ensure_lookup_table()

		for face in bm.faces:
			face.smooth = True

def loft_bezier(self, context, curves, count, flip_normals, merge_distance, *, precision=10, name):
	data = bpy.data
	scene = context.scene
	BT_Loft_data = data.meshes.new('BT_Loft_data')

	bm = bmesh.new()
	bm.from_mesh(BT_Loft_data)

	# each quad loop vertex buffer will be stored in vertex_buffer<collection of collections>
	vertex_buffer = []
	
	for index, curve in enumerate(curves):
		if (index+1) >= len(curves):
			break
		
		curve_next = curves[index+1]
		vertex_buffer.append(get_quad_loop_verts_data(self,
			[point for point in space_interpolate_bezier(curve, precision, count)],
			[point for point in space_interpolate_bezier(curve_next, precision, count)]
			)
		)

	# now we fill bm.verts and build faces
	for loop in vertex_buffer:
		quads = []
		for vert in loop:
			quads.append(bm.verts.new(vert))

		# build quad loop
		build_quad_loop(self, bm, quads)

		bm.verts.index_update()
		bm.verts.ensure_lookup_table()

	for vert in bm.verts:
		vert.normal_update()

	weld_map = bmesh.ops.find_doubles(bm, verts=bm.verts, dist=merge_distance) 
	bmesh.ops.weld_verts(bm, targetmap=weld_map['targetmap'])

	bm.faces.ensure_lookup_table()
	
	for vert in bm.faces:
		vert.normal_update()    

	if are_normals_flipped(self, context, bm):
		for face in bm.faces:           
			face.normal_flip()
			face.normal_update()

	if flip_normals:
		for face in bm.faces:           
			face.normal_flip()
			face.normal_update()

	# finalizing bmesh
	bm.to_mesh(BT_Loft_data)    

	# finalizing BT_Loft
	BT_Loft_object = bpy.data.objects.new(name, BT_Loft_data)
	BT_Loft_data.calc_loop_triangles()
	scene.collection.objects.link(BT_Loft_object)
	bm.free()

	return BT_Loft_object	

def loft_polyline(self, context, curves, flip_normals):
	# create bmesh
	data = bpy.data
	scene = context.scene
	BT_Loft_data = data.meshes.new('BT_Loft_data')

	bm = bmesh.new()
	bm.from_mesh(BT_Loft_data)

	# each quad loop vertex buffer will be stored in vertex_buffer<collection of collections>
	vertex_buffer = []

	for index, curve in enumerate(curves):
		if (index+1) >= len(curves):
			break        

		vertex_buffer.append(get_quad_loop_verts_data(self,
			[to_world(self, curve.matrix_world, point) for point in [Vector((point.co.xyz)) for point in curve.data.splines[0].points]],
			[to_world(self, curves[index+1].matrix_world, point) for point in [Vector((point.co.xyz)) for point in curves[index+1].data.splines[0].points]]
			)
		)

	# now we fill bm.verts and build faces
	for loop in vertex_buffer:
		quads = []
		for vert in loop:
			quads.append(bm.verts.new(vert))

		# build quad loop
		build_quad_loop(self, bm, quads)

		bm.verts.index_update()
		bm.verts.ensure_lookup_table()

	for vert in bm.verts:
		vert.normal_update()

	weld_map = bmesh.ops.find_doubles(bm, verts=bm.verts, dist=0.0001)
	bmesh.ops.weld_verts(bm, targetmap=weld_map['targetmap'])

	if flip_normals:
		for face in bm.faces:
			face.normal_flip()
			face.normal_update()

	# finalizing bmesh
	bm.to_mesh(BT_Loft_data)    

	# finalizing BT_Loft
	BT_Loft_object = bpy.data.objects.new('BT_LoftMesh', BT_Loft_data)
	BT_Loft_data.calc_loop_triangles()
	scene.collection.objects.link(BT_Loft_object)
	bm.free()

	return BT_Loft_object

class BT_Loft(Operator):
	bl_idname = 'object.bt_bezier_mesh_loft'
	bl_label = 'Loft'
	bl_description = 'Build a Loft Mesh. Takes at least 2 parallel Bézier or Polyline curves'
	bl_options = {'REGISTER', 'UNDO'}
	resolution: bpy.props.IntProperty(name='Resolution', default=8, min=1)
	precision: bpy.props.IntProperty(name='Precision', min=1, max=100, default=10, description='Resolution of constraint curve. Low - may lead to missing points, high - to slow calculation')
	merge_distance: bpy.props.FloatProperty(name='Merge Distance', default=0.001, min=0, soft_min=0, description='Distance at which neighbouring vertices will merge')
	flip_normals: bpy.props.BoolProperty(name='Flip Normals', default=False, description='Flip face normals', options={'SKIP_SAVE'})
	remove_source: bpy.props.BoolProperty(name='Remove Source', default=False, description='Remove source curves and finish', options={'SKIP_SAVE'})

	@classmethod
	def poll(self, context):
		return context.object is not None

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'resolution')
		column.prop(self, 'precision')		
		column.prop(self, 'merge_distance')
		column.prop(self, 'flip_normals', toggle=1)
		column.prop(self, 'remove_source')

	def execute(self, context):
		context.evaluated_depsgraph_get()
		curves = [obj for obj in context.selected_objects if obj.type == 'CURVE']
		first_curve = context.object
		resolution = self.resolution

		if is_valid_bezier_selection(self, curves):
			for curve in curves:
				if curve is not first_curve:
					dot = (to_world(self, curve.matrix_world, curve.data.splines[0].bezier_points[0].co) - to_world(self, curve.matrix_world, curve.data.splines[0].bezier_points[1].co)).dot(to_world(self, first_curve.matrix_world, first_curve.data.splines[0].bezier_points[0].co) - to_world(self, first_curve.matrix_world, first_curve.data.splines[0].bezier_points[1].co))
					if dot < 0:
						reverse_curve(self, curve)          

			# sort curves in the selection list by distance starting from the active one        
			sort_by_distance(self, context.object, curves)          
			loft_mesh = loft_bezier(self, context, curves, resolution+1, self.flip_normals, self.merge_distance, precision=self.precision, name='LoftMesh') 

			if loft_mesh is not None:
				loft_mesh.select_set(True)
				context.view_layer.objects.active = loft_mesh  

		elif is_valid_polyline_selection(self, curves):
			for curve in curves:
				if curve is not first_curve:
					dot = (to_world(self, curve.matrix_world, Vector((curve.data.splines[0].points[0].co[:3]))) - to_world(self, curve.matrix_world, Vector((curve.data.splines[0].points[-1].co[:3])))).dot(to_world(self, first_curve.matrix_world, Vector((first_curve.data.splines[0].points[0].co[:3]))) - to_world(self, first_curve.matrix_world, Vector((first_curve.data.splines[0].points[-1].co[:3]))))
					if dot < 0:
						reverse_curve(self, curve)

			sort_by_distance(self, context.object, curves)

			# bpy.ops.object.mode_set(mode='OBJECT')
			# bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)		

			loft_mesh = loft_polyline(self, context, curves, self.flip_normals)
			
			if loft_mesh is not None:
				loft_mesh.select_set(True)
				context.view_layer.objects.active = loft_mesh		
		
		if self.remove_source:
			for curve in curves[:]:
				if curve is not None and curve.name in bpy.data.objects:
					bpy.data.objects.remove(curve, do_unlink=True)

		bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='MEDIAN')	  
			
		return {'FINISHED'}

class BT_Patch(Operator):
	bl_idname = "object.bt_build_bezier_mesh_patch"
	bl_label = "Patch"
	bl_description = 'Build a Patch mesh. Takes 4 Bézier curves: 2 Rails and 2 Profiles'
	bl_options = {'REGISTER', 'UNDO'}	
	resolution_u: bpy.props.IntProperty(default=8, min=1, max=100, name='Resolution U')      
	resolution_v: bpy.props.IntProperty(default=8, min=1, max=100, name = 'Resolution V')
	precision: bpy.props.IntProperty(name='Precision', min=1, max=100, default=10, description='Resolution of constraint curve. Low - may lead to missing points, high - to slow calculation')
	search_limit : bpy.props.IntProperty(default=5, min=1, max=5, name = 'Search Limit', description='Limit value for searching common points for loop curves. Increase or decrease in case of search warnings')
	merge_distance: bpy.props.FloatProperty(name='Merge Distance', default=0.001, min=0.0, description='Distance at which neighbouring vertices will merge')
	flip_normals: bpy.props.BoolProperty(name='Flip Normals', default=False, description='Flip face normals', options={'SKIP_SAVE'})
	remove_source: bpy.props.BoolProperty(name='Remove Source', default=False, description='Remove source curves and finish', options={'SKIP_SAVE'})
	
	@classmethod
	def poll(cls, context):
		return context.object is not None

	def draw(self, context):
		layout = self.layout
		column = layout.column()
		column.prop(self, 'resolution_u')
		column.prop(self, 'resolution_v')
		column.prop(self, 'precision')		
		column.prop(self, 'merge_distance')
		column.prop(self, 'search_limit')		
		column.prop(self, 'flip_normals', toggle=True)
		column.prop(self, 'remove_source')
	
	def execute(self, context): 
		context.evaluated_depsgraph_get()  
		sel = [obj for obj in context.selected_objects if obj.type=='CURVE']
		for curve in sel:
			if  len(curve.data.splines) == 1 and is_bezier(curve):
				continue
			else:
				self.report({'ERROR'}, "Patch requires selection of a loop made by 4 separate bezier curves")
				return{'CANCELLED'}

		# points_map
		points_map = {}
		for curve in sel:
			points_map[curve] = []
			for point in curve.data.splines[0].bezier_points:
				points_map[curve].append(to_world(self, curve.matrix_world, point.co.copy().freeze()))

		horizon_1 = context.object
		horizon_2 = None
		vertical_1 = None
		vertical_2 = None

		needs_reverse = []

		if horizon_1 not in context.selected_objects:
			self.report({'ERROR'}, "Context object must be one of the loop curves!")
			return {'CANCELLED'}

		# find the closest curve to horizon_1 start point   
		for curve, coords in points_map.items():
			if curve is not horizon_1:                          
				if is_equal(points_map[horizon_1][0], coords[0], self.search_limit):
					vertical_1 = curve                          
				elif is_equal(points_map[horizon_1][0], coords[-1], self.search_limit):
					needs_reverse.append(curve)             
					vertical_1 = curve                      
									
				if is_equal(points_map[horizon_1][-1], coords[-1], self.search_limit):
					needs_reverse.append(curve)
					vertical_2 = curve                                      
				elif is_equal(points_map[horizon_1][-1], coords[0], self.search_limit):
					vertical_2 = curve

				if  (not is_equal(points_map[horizon_1][0], coords[0], self.search_limit) and not is_equal(points_map[horizon_1][0], coords[-1], self.search_limit)) and (not is_equal(points_map[horizon_1][-1], coords[0], self.search_limit) and not is_equal(points_map[horizon_1][-1], coords[-1], self.search_limit)):
					horizon_2 = curve

		for curve in needs_reverse:
			reverse_curve(self, curve)      

		for curve in (horizon_1, vertical_1, vertical_2, horizon_2):
			if curve is None:
				self.report({'ERROR'}, "Patch construction data is not valid. Check if the curve perimeter is enclosed and there are no gaps between curves")
				return {'FINISHED'}
		
		if len(horizon_1.data.splines[0].bezier_points) != len(horizon_2.data.splines[0].bezier_points) or len(vertical_1.data.splines[0].bezier_points) != len(vertical_2.data.splines[0].bezier_points):
			self.report({'ERROR'}, "Parallel curves must have equal number of points")
			return {'CANCELLED'}

		points_map.clear()
		for curve in (horizon_1, vertical_1, vertical_2, horizon_2):
			points_map[curve] = []
			for point in curve.data.splines[0].bezier_points:
				points_map[curve].append(to_world(self, curve.matrix_world, point.co.copy().freeze()))

		if not is_equal(points_map[vertical_2][-1], points_map[horizon_2][-1], self.search_limit):
			reverse_curve(self, horizon_2)

		curves = [vertical_1]
		blends = blend_2_profiles_2_rails(self, context, self.resolution_v, curves = (horizon_1, horizon_2, vertical_1, vertical_2))
		curves.extend(blends)
		curves.append(vertical_2)
		patch_mesh = loft_bezier(self, context, curves, self.resolution_u, self.flip_normals, self.merge_distance, precision=self.precision, name='PatchMesh')

		if patch_mesh is not None:
			patch_mesh.select_set(True)
			context.view_layer.objects.active = patch_mesh			
		
		# horizon_1.select_set(True)
		# context.view_layer.objects.active = horizon_1
		
		if self.remove_source:
			for curve in (horizon_1, vertical_1, vertical_2, horizon_2):
				if curve is not None and curve.name in bpy.data.objects:
					bpy.data.objects.remove(curve, do_unlink=True)   

		for blend in blends:
			bpy.data.objects.remove(blend)
		
		bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='MEDIAN')

		return{'FINISHED'}

# UTILS #########################################################################

EPSILON = 1.e-5

# https://en.wikipedia.org/wiki/ANSI_escape_code#:~:text=ANSI%20escape%20sequences%20are%20a,character%2C%20are%20embedded%20into%20text
# print(BT_LogColor.GREEN + 'Target engaged' + BT_LogColor.NORMAL)
class BT_LogColor:
	RED    = ERROR   =  '\033[91m'
	GREEN  = SUCCESS =  '\033[92m'
	BLUE   =                '\033[94m'
	CYAN   = INFO    =  '\033[96m'
	YELLOW = WARNING =  '\033[93m'
	NORMAL =                '\033[0m'

# Raycast
class BT_HitResult():
	def __init__(self, args):
		self.result = False
		self.location = Vector()
		self.normal = Vector()
		self.index = -1
		self.obj = None
		self.matrix = Matrix()

		if isinstance(args, tuple) and len(args) == 6:
			self.result   = args[0]
			self.location = args[1].copy()
			self.normal   = args[2].copy()
			self.index    = args[3]
			self.obj      = args[4]
			self.matrix   = args[5].copy()
		else:
			pass

	def is_valid(self):
		return self.result == True

def scene_ray_cast(self, context, cursor):
	depsgraph = context.evaluated_depsgraph_get()
	region = context.region
	rv3d = context.region_data

	view_vector =  bpy_extras.view3d_utils.region_2d_to_vector_3d(region, rv3d, cursor)
	ray_origin  =  bpy_extras.view3d_utils.region_2d_to_origin_3d(region, rv3d, cursor)
	ray_target  =  ray_origin + view_vector
	direction = ray_target - ray_origin
	return context.scene.ray_cast(depsgraph, ray_origin, direction.normalized())

# # Info
# messages = {    
# 'BlendOneRail': ((
# 	'Select Path1',
# 	'Select Path2',
# 	'Select Profile'
# 	)),
# 'BlendTwoRails': ((
# 	'Select Path1',
# 	'Select Path2',
# 	'Select Profile1',
# 	'Select Profile2'
# 	))
# }

# Curves --------------------------------------------
#https://en.wikipedia.org/wiki/B%C3%A9zier_curve

def is_bezier(curve):
	return curve.type == 'CURVE' and curve.data.splines[0].type == 'BEZIER'

def is_equal_n(curve1, curve2):
	if not is_bezier(curve1) or not is_bezier(curve2):
		return False
	curve1_points = curve1.data.splines[0].bezier_points
	curve2_points = curve2.data.splines[0].bezier_points    
	return len(curve1_points) == len(curve2_points)

def is_valid_bezier_selection(self, sel):
	for obj in sel:
		if obj.type != 'CURVE':
			return False
		for spline in obj.data.splines:
			if spline.type != 'BEZIER':
				return False
	return True

def is_valid_polyline_selection(self, sel):
	for obj in sel:
		if obj.type != 'CURVE':
			return False
		for spline in obj.data.splines:
			if spline.type != 'POLY':
				return False            
	return True 

def set_handle_type(self, curve, handle_type):
	for spline in curve.data.splines:
		if spline.type != 'BEZIER':
			spline.type = 'BEZIER'
		points = spline.bezier_points
		for point in points:
			if point.handle_left_type != handle_type:
				point.handle_left_type = handle_type            
			if point.handle_right_type != handle_type:  
				point.handle_right_type = handle_type    


# point = pow((1 - t), 3)*p0.co + 3*(pow((1 - t), 2))*t*handle_right + 3*(1 - t)*pow(t, 2)*handle_left + pow(t, 3)*p3.co

def interpolate_cubic_bezier_matrix(self, t, p0, p1, p2, p3):
	#https://pomax.github.io/bezierinfo/#matrix
	
	variables = Matrix([
	(1, t, pow(t, 2), pow(t, 3)),
	(0, 0, 0, 0)
	])
	
	coefs = Matrix([
	(1, 0, 0, 0),
	(-3, 3, 0, 0),
	(3, -6, 3, 0),
	(-1, 3, -3, 1)
	])
	
	control_points = Matrix([
	(p0),
	(p1),
	(p2),
	(p3)
	])
	
	return variables @ coefs @ control_points

def calculate_bezier_handles(points, t1, t2):
	# Find p1 and p2 control points(handles) if p0, t1, t2 and p3 are given   
	# https://stackoverflow.com/questions/54198446/fitting-a-single-bezier-curve-to-4-points-in-3d

	handles = [None]*2
	
	det = a11 = a12 = a21 = a22 = b1 = b2 = None

	tt1 = 1 - t1
	tt2 = 1 - t2

	a11 = 3 * pow(tt1, 2) * t1
	a12 = 3 * tt1 * pow(t1, 2)
	a21 = 3 * pow(tt2, 2) * t2
	a22 = 3 * tt2 * pow(t2, 2)
	det = a11 * a22 - a12 * a21

	b1 = points[1] - points[0] * pow(tt1, 3) - points[3] * pow(t1, 3)
	b2 = points[2] - points[0] * pow(tt2, 3) - points[3] * pow(t2, 3)

	if det == 0:
		print(BT_LogColor.ERROR + 'calculate_bezier_handles: zero division' + BT_LogColor.NORMAL)
		return []
	
	handles[0] = (b1 * a22 - b2 * a12) / det
	handles[1] = (-b1 * a21 + b2 * a11) / det

	return handles

def calculate_t_from_bezier_interpolated_points_cubic(points):
	if len(points) != 4:
		print('4 points were expected, but got ' + str(len(points)))
		return(1/3, 2/3)

	length = 0.0
	for index, point in enumerate(points):
		if (index + 1) % len(points) == 0:
			break
		length += get_distance(point, points[index + 1])

	t1 = get_distance(points[0], points[1])/length
	t2 = get_distance(points[0], points[2])/length

	return (t1, t2)

def get_interpolated_bezier(self, data, count):
	# returns interpolated points without control points p0, p1
	if count == 0:
		return []
	
	interpolated_points = []

	count += 1
	for index in range(count):
		if index == 0:
			continue
		
		interpolated_points.append(
			interpolate_cubic_bezier_matrix(
				self,
				index/count,
				data.p0_co,
				data.p0_handle_right,
				data.p1_handle_left,
				data.p1_co
				)[0].xyz
		)

	return interpolated_points

def mathutils_interpolate_bezier_data(self, data, count):
	# returns all interpolated points
	return mathutils.geometry.interpolate_bezier(
		data.p0_co,
		data.p0_handle_right,
		data.p1_handle_left,
		data.p1_co,
		count
		)

def mathutils_interpolate_bezier_points(self, points, count):
	# returns all interpolated points
	
	# linear
	if len(points) == 2:
		return mathutils.geometry.interpolate_bezier(
			points[0],
			points[0],
			points[1],
			points[1],
			count
			)
	# quadratic
	elif len(points) == 3:
		return mathutils.geometry.interpolate_bezier(
			points[0],
			points[1],
			points[1],
			points[2],
			count
			)
	# cubic
	elif len(points) > 3:
		return mathutils.geometry.interpolate_bezier(
			points[0],
			points[1],
			points[2],
			points[3],
			count
			)   
		
	return None

def mathutils_interpolate_n_bezier_points(curve, count, *, world_space=True, proportional=False, debug=False):	
	spline = curve.data.splines[0]
	points = spline.bezier_points
	matrix = curve.matrix_world if world_space else Matrix()
	interpolated_points = [matrix@points[0].co]

	disribution=[]
	full_length = spline.calc_length()

	if proportional:
		# adaptive distribution of interpolated points depending on lengths of spline's segments between control points
		# in this case final interpolated points count may not match the given count		
		for index, point in enumerate(points):
			if index+1 != len(points):
				length=0
				ips = mathutils.geometry.interpolate_bezier(point.co, point.handle_right, points[index+1].handle_left, points[index+1].co, 12)			
				for index_ip, ip in enumerate(ips):
					if index_ip+1 != len(ips):
						ip_next = ips[index_ip+1]
						length += (ip - ip_next).length
				disribution.append(int(count*(length/full_length)))		

		# if we wanted to be a bit closer to the target count
		# for i in range(abs(sum(disribution) - count)):
		# 	if sum(disribution) < count:					
		# 		_min = min(enumerate(disribution))[0]
		# 		disribution[_min] += 1		

		# for i in range(abs(sum(disribution) - count)):
		# 	if sum(disribution) > count:
		# 		_max = max(enumerate(disribution))[0]
		# 		if _max > 2:							
		# 			disribution[_max] -= 1
			
		# bezier interpolation requires count of at least 2
		for index, c in enumerate(disribution):
			if c < 2:
				disribution[index] = 2

	# now we can do final interpolation
	for index, point in enumerate(points):
		if index+1 != len(points):
			for ip in mathutils.geometry.interpolate_bezier(
				point.co,
				point.handle_right,
				points[index+1].handle_left,
				points[index+1].co,
				(disribution[index] if len(disribution) else count))[1:]:
				interpolated_points.append(matrix@ip)	

	if debug:
		for ip in interpolated_points:
			bpy.ops.object.empty_add(radius=0.01, location=ip)

	return interpolated_points

def space_interpolate_bezier(curve, precision, count, *, debug=False):
	spline = curve.data.splines[0]
	bezier_points = spline.bezier_points
	matrix = curve.matrix_world
	interpolated_points = mathutils_interpolate_n_bezier_points(curve, precision*100, world_space=False, proportional=True)
	space_points = [matrix@interpolated_points[0]]
	target_segment_length = round(spline.calc_length()/count, 9)
	
	length=0
	for index, point in enumerate(interpolated_points):	
		if index+1 != len(interpolated_points):
			point_next = interpolated_points[index+1]		
			length += (point - point_next).length
			
			if length > target_segment_length:										
				space_points.append(matrix@interpolated_points[index-1])
				length = 0

	if len(space_points)-1 < count:
		space_points.append(matrix@interpolated_points[-1])

	if not isclose(space_points[-1].length - interpolated_points[-1].length, 0.0, abs_tol=1e-5):		
		space_points[-1] = matrix@interpolated_points[-1]
	
	if debug:
		for point in space_points:
			bpy.ops.object.empty_add(radius=0.01, location=point)

	return space_points

def calculate_bezier_tangent(points, t):
	p0, p1, p2, p3 = points
	return 3*((1 - t)*(1 - t))*(p1 - p0) + 6*(1 - t)*t*(p2 - p1) + 3*(t*t)*(p3 - p2)

def calculate_curve_length(self, curve): 
	length = 0
	for spline in curve.data.splines:
		length += spline.calc_length(resolution=spline.resolution_u)
	return length

def get_bezier_point_t_map(self, curve):	
	resolution = curve.data.splines[0].resolution_u

	# {point index : t}
	point_t_map = dict()
	points = mathutils_interpolate_n_bezier_points(curve, resolution+1)

	for index in range(len(points)):
		# # we don't need coordinates of t = 0.0 and t = 1.0
		# if index == 0 or index == len(points):
		# 	continue
		point_t_map[points[index].copy().freeze()] = index/len(points)

	return point_t_map

def snap_get_points(self, context):
	viewport = get_view_3d(self, context)
	width=viewport.width
	height=viewport.height

	coords = set()
	curves = [obj.evaluated_get(context.evaluated_depsgraph_get()) for obj in bpy.data.objects if ((obj.type == 'CURVE') and (obj.name in context.view_layer.objects) and (obj.visible_get() == True ))]
	empties = [obj for obj in bpy.data.objects if ((obj.type == 'EMPTY') and (obj.name in context.view_layer.objects) and (obj.visible_get() == True ))]
	
	for curve in curves:		
		interpolated_points = mathutils_interpolate_n_bezier_points(curve, curve.data.splines[0].resolution_u+1) if is_bezier(curve) else [curve.matrix_world@Vector(point.co[0:3]) for point in curve.data.splines[0].points]
	
		for point in interpolated_points:
			screen_point = vector_3d_to_screen(self, context, point)
			if screen_point is None or (screen_point.x > width or screen_point.x < 0) or (screen_point.y > height or  screen_point.y < 0):
				continue		
			coords.add(point.freeze())
	
	for empty in empties:
		coords.add(empty.location.copy().freeze())

	return coords

def snap_get_target(self, context, cursor, screen_world_map):	
	return BT_Cursor.get_nearest_target_point_world(self, cursor, screen_world_map)

def reverse_curve(self, curve): 
	def get_bezier_props(point):
		return {        
			'co': point.co.copy(),
			'handle_left' : point.handle_left.copy(),
			'handle_left_type' : point.handle_left_type,
			'handle_right' : point.handle_right.copy(),     
			'handle_right_type' : point.handle_right_type,
			'hide' : point.hide,
			'radius' : point.radius,
			'select_control_point' : point.select_control_point,
			'select_left_handle' : point.select_left_handle,
			'select_right_handle' : point.select_right_handle,
			'tilt' : point.tilt,
			'weight_softbody' : point.weight_softbody
			}

	def get_poly_props(point):
		return {
			'co' : point.co.copy(),
			'hide' : point.hide,
			'radius' : point.radius,
			'select' : point.select,
			'tilt' : point.tilt,
			'weight' : point.weight,
			'weight_softbody' : point.weight_softbody
		}

	spline = curve.data.splines[0]
	if spline.type == 'BEZIER':
		points = spline.bezier_points
		for index, point in enumerate(points):
			if index < len(points)/2:               
				p0 = point
				p1 = points[-(index+1)]

				temp_p0 = get_bezier_props(p0)
				temp_p1 = get_bezier_props(p1)      

				# handle type must go first because it will break the 
				# curve if done after points swap
				p0.handle_right_type =      temp_p1['handle_left_type']
				p0.handle_left_type =       temp_p1['handle_right_type']
				# control point goes second
				p0.co =                     temp_p1['co']           
				# handle stuff cross swapped
				p0.handle_right =           temp_p1['handle_left']
				p0.handle_left =            temp_p1['handle_right']             
				p0.select_right_handle =    temp_p1['select_left_handle']
				p0.select_left_handle =     temp_p1['select_right_handle'] 
				# other stuff               
				p0.hide =                   temp_p1['hide']
				p0.radius =                 temp_p1['radius']
				p0.select_control_point =   temp_p1['select_control_point']             
				p0.tilt =                   temp_p1['tilt']
				p0.weight_softbody =        temp_p1['weight_softbody']

				p1.handle_right_type =      temp_p0['handle_left_type']
				p1.handle_left_type =       temp_p0['handle_right_type']                            
				p1.co =                     temp_p0['co']               
				p1.handle_right =           temp_p0['handle_left']
				p1.handle_left =            temp_p0['handle_right']             
				p1.select_right_handle =    temp_p0['select_left_handle']
				p1.select_left_handle =     temp_p0['select_right_handle']
				p1.hide =                   temp_p0['hide']
				p1.radius =                 temp_p0['radius']
				p1.select_control_point =   temp_p0['select_control_point']
				p1.tilt =                   temp_p0['tilt']
				p1.weight_softbody =        temp_p0['weight_softbody']

	else:
		points = spline.points
		for index, point in enumerate(points):
			if index < len(points)/2:
				p0 = point
				p1 = points[-(index+1)]

				temp_p0 = get_poly_props(p0)
				temp_p1 = get_poly_props(p1)
											
				p0.co =                     temp_p1['co']               
				p0.hide =                   temp_p1['hide']
				p0.radius =                 temp_p1['radius']
				p0.select =                 temp_p1['select']
				p0.tilt =                   temp_p1['tilt']
				p0.weight =                 temp_p1['weight']
				p0.weight_softbody =        temp_p1['weight_softbody']

				p1.co =                     temp_p0['co']           
				p1.hide =                   temp_p0['hide']
				p1.radius =                 temp_p0['radius']
				p1.select =                 temp_p0['select']
				p1.tilt =                   temp_p0['tilt']
				p1.weight =                 temp_p0['weight']
				p1.weight_softbody =        temp_p0['weight_softbody']

def is_single_view3d(self, context):
	view3d_areas = [area for area in context.window.screen.areas if area.type == 'VIEW_3D']	
	if len(view3d_areas) != 1:
		self.report({'ERROR'}, "Split VIEW_3D areas are not supported!")
		return False
	
	spaces = [space for space in view3d_areas[0].spaces]
	if len(spaces) != 1:
		self.report({'ERROR'}, "Can only work inside a single VIEW_3D space!")
		return False
	
	if len(spaces[0].region_quadviews) > 0:
		self.report({'ERROR'}, "Quad Views are not supported!")
		return False

	return True

def get_view_3d(self, context):	
	for area in bpy.context.window.screen.areas:		
		if area.type == 'VIEW_3D':
			return area

	return None

def viewport_to_screen_coordinates_set(self, context, points):
	region = context.region
	rv3d = context.region_data
	screen_coords = set()
	view_3d = get_view_3d(self, context)    
	
	if view_3d is None:
		return set()

	for point in points:
		coord = bpy_extras.view3d_utils.location_3d_to_region_2d(region, rv3d, point)           
		
		if not is_vector2D_on_screen(self, coord, view_3d):
			continue

		screen_coords.add(coord.freeze())

	return frozenset(screen_coords)

def viewport_to_screen_coordinates_list(self, context, points):
	region = context.region
	rv3d = context.region_data
	screen_coords = list()
	view_3d = get_view_3d(self, context)    
	
	if view_3d is None:
		return list()
	
	for point in points:
		coord = bpy_extras.view3d_utils.location_3d_to_region_2d(region, rv3d, point)       

		# if not is_vector2D_on_screen(self, coord, view_3d):
		#   continue
		if coord is not None:
			screen_coords.append(coord.freeze())
		else:
			screen_coords.append(Vector((0, 0)).freeze())

	return screen_coords

def get_cursor(self, event, *, as_tuple=False):
	return (event.mouse_region_x, event.mouse_region_y) if as_tuple else Vector((event.mouse_region_x, event.mouse_region_y))

def get_region_view_3d(self, context):
	view3d = bpy.context.space_data
	return view3d.region_3d

def get_projection_location(self, context, origin, obj, direction, view_vector):
	hit_result = None

	if obj is not None:
		hit_result = get_closest_hit_result(self,
			obj,
			to_local(self, obj.matrix_world, origin),
			direction
			)
	
	if hit_result is None:
		if get_region_view_3d(self, context).is_perspective:
			# to viewport
			return view_vector + origin
		else:
			# to origin XYZ planes                      
			return origin*invert_basis_vector(get_view_direction(self, context))

	projection = to_world(self, obj.matrix_world, hit_result[1])
	return projection

def get_view_direction(self, context):
	area  = get_view_3d(self, context)	
	if area is None:
		return Vector()

	view_rotation = get_region_view_3d(self, context).view_rotation

	x = view_rotation.x
	y = view_rotation.y
	z = view_rotation.z 
	w = view_rotation.w

	direction = Vector()
	
	direction[0] = 2 * (x * z + w * y)
	direction[1] = 2 * (y * z - w * x)
	direction[2] = 1 - 2 * (x * x + y * y)

	return direction.normalized()

def get_view_vector_and_ray_origin(self, context, cursor):
	region = context.region
	rv3d = context.region_data

	view_vector = bpy_extras.view3d_utils.region_2d_to_vector_3d(region, rv3d, cursor)
	ray_origin = bpy_extras.view3d_utils.region_2d_to_origin_3d(region, rv3d, cursor)
	return (view_vector, ray_origin)

def get_closest_hit_result(self, obj, origin, direction):
	if obj is None:
		return None

	def do_ray_cast(direction):     
		return obj.ray_cast(origin, direction)

	hit_result_1 = do_ray_cast(direction)
	
	# inverse     
	hit_result_2 = do_ray_cast(-1*direction)        
	
	if hit_result_1[0] and not hit_result_2[0]:
		return hit_result_1
	
	elif not hit_result_1[0] and hit_result_2[0]:
		return hit_result_2 
	
	elif not hit_result_1[0] and not hit_result_2[0]:
		return None

	# closest to the origin
	elif hit_result_1[0] and hit_result_2[0]:
		return {True:hit_result_1, False:hit_result_2}[(hit_result_1[1]-origin).length < (hit_result_2[1]-origin).length]

def point_3d_to_2d(self, context, point):
	return bpy_extras.view3d_utils.location_3d_to_region_2d(context.region, context.region_data, point)

def get_screen_world_map(self, context, points):
	screen_world_map = dict()
	
	view_3d_area = get_view_3d(self, context)	
	
	if view_3d_area is None:
		return dict()
	
	for point in points:
		screen_coord = point_3d_to_2d(self, context, point)
		if screen_coord is None:
			continue
			
		if not is_vector2D_on_screen(self, screen_coord, view_3d_area):
			continue

		world_coord = point.copy().freeze()

		screen_world_map[screen_coord.freeze()] = world_coord
	
	return screen_world_map

def is_close(point_1, point_2):
	return isclose(point_1.length, point_2.length)

def is_equal(point_1, point_2, precision):
	return round(point_1.length, precision) == round(point_2.length, precision)

def calculate_average_normal(self, verts):
	if not len(verts) > 0:
		return Vector()

	normal_sum = Vector()
	for vert in verts:
		normal_sum = normal_sum + vert.normal

	return normal_sum * (1/len(verts))

def are_normals_flipped(self, context, bm):
	# checks if face normals are flipped relative to the view
	median_normal = calculate_average_normal(self, bm.verts).normalized()
	view_direction = get_view_direction(self, context).normalized()
	# print(median_normal.dot(view_direction))
	return median_normal.dot(view_direction) <= 0

def project(self, context, cursor, *, on_mesh=False):
	tool_settings = bpy.context.scene.tool_settings         

	view_vector, ray_origin = get_view_vector_and_ray_origin(self, context, cursor)
	
	direction = view_vector.normalized()
	hit_result = scene_ray_cast(self, context, cursor)
	
	if on_mesh and hit_result[0]:		
		target_object = hit_result[4]
		target_obj_eval = target_object.evaluated_get(context.evaluated_depsgraph_get())
		return (cursor, get_projection_location(self, context, ray_origin, target_obj_eval, direction, view_vector))

	return (cursor, get_projection_location(self, context, ray_origin, None, direction, view_vector))	
	   
def get_distance(vector1, vector2):
	return (vector1 - vector2).length

def vectors_are_equal(self, vector1, vector2, epsilon):
	dist = get_distance(vector1, vector2)
	return dist <= epsilon

def to_world(self, matrix, vector):
	return matrix @ vector

def to_local(self, matrix, vector):
	return matrix.inverted() @ vector

def interpolate_2_vectors(self, vector1, vector2, t):
	return vector1 + t*(vector2 - vector1)

def is_vector2D_on_screen(self, vector, area):
	if vector is None:
		return False    
	return (vector.x < area.width and vector.x > 0) and (vector.y < area.height and vector.y > 0)

def invert_basis_vector(vector): 
	# returns swapped 0 and 1 vector components, for example: (1, 0, 0) -> (0, 1, 1), (-1, -0.001, 0.005) -> (0, 1, 1)
	inv = Vector()
	for index, co in enumerate(vector):
		if abs(co) > 0.5:
			inv[index] = 0
		else:
			inv[index] = 1
	return inv
	 
def vector_3d_to_screen(self, context, vector):
	region = context.region
	rv3d = context.region_data
	view_3d = get_view_3d(self, context)
	return bpy_extras.view3d_utils.location_3d_to_region_2d(region, rv3d, vector)

def vector_2d_to_world(self, context, vector):
	region = context.region
	rv3d = context.region_data
	return bpy_extras.view3d_utils.region_2d_to_location_3d(region, rv3d, vector, Vector())

def set_pivot(obj, new_pos):
	pos = obj.matrix_world.translation
	obj.data.transform(Matrix.Translation(pos-new_pos))
	obj.matrix_world.translation = new_pos

def build_kd_tree(points):
	kd_tree = mathutils.kdtree.KDTree(len(points))
	for index, point in enumerate(points):
		kd_tree.insert(point.copy().freeze(), index)
	kd_tree.balance()
	return kd_tree

# Viewport
def update_viewport(self, context):
	for area in bpy.context.window.screen.areas:
		if area.type == 'VIEW_3D':
			area.tag_redraw()

def remove_gpu_draw_handler(self, handler):
	if handler is not None:
		bpy.types.SpaceView3D.draw_handler_remove(handler, 'WINDOW')

def draw_target(self, context, pos):
	def draw():
		shader.uniform_float("color", context.scene.bt_color)#(0.0, 1.0, 0.5, 1.0)
		batch.draw(shader)

	coords =    [
				(pos.x - 5, pos.y - 5), (pos.x + 5, pos.y - 5),
				(pos.x - 5, pos.y + 5), (pos.x + 5, pos.y + 5),
				(pos.x - 5, pos.y + 5), (pos.x - 5, pos.y - 5),
				(pos.x + 5, pos.y + 5), (pos.x + 5, pos.y - 5)
				]

	shader = gpu.shader.from_builtin('UNIFORM_COLOR')
	batch = batch_for_shader(shader, 'LINES', {"pos": coords})

	handler = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_PIXEL')  
	update_viewport(self, context)

	return handler

def draw_snap_targets(self, context, points):
	def draw():
		shader.uniform_float("color", (0.0, 0.7, 0.7, 1.0))
		batch.draw(shader)

	shader = gpu.shader.from_builtin('POINT_UNIFORM_COLOR')
	batch = batch_for_shader(shader, 'POINTS', {"pos": points})
	gpu.state.point_size_set(10)

	handler = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_VIEW')
	update_viewport(self, context)

	return handler

def draw_2d_polyline(self, context, points, index_buffer):
	def draw():
		shader.uniform_float("color", context.scene.bt_color)
		batch.draw(shader)

	shader = gpu.shader.from_builtin('UNIFORM_COLOR')
	batch = batch_for_shader(shader, 'LINES', {"pos": points}, indices=index_buffer)

	handler = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_PIXEL')
	update_viewport(self, context)

	return handler

def update_object_edit(context):
	if context.object is not None:
		bpy.ops.object.mode_set(mode = 'OBJECT')
		bpy.ops.object.mode_set(mode = 'EDIT')

def update_edit_object(context):
	if context.object is not None:
		bpy.ops.object.mode_set(mode = 'EDIT')
		bpy.ops.object.mode_set(mode = 'OBJECT')

def update_edit_object_edit(context):
	if context.object is not None:
		bpy.ops.object.mode_set(mode = 'EDIT')
		bpy.ops.object.mode_set(mode = 'OBJECT')
		bpy.ops.object.mode_set(mode = 'EDIT')	

def update_object_edit_object(context):
	if context.object is not None:
		bpy.ops.object.mode_set(mode = 'OBJECT')
		bpy.ops.object.mode_set(mode = 'EDIT')
		bpy.ops.object.mode_set(mode = 'OBJECT')

# UI ############################################################################################
pcoll = None

class BT_Settings(Panel):
	bl_label = 'Settings'  
	bl_idname = "SCENE_PT_bt_settings"
	bl_space_type = 'PROPERTIES'
	bl_region_type = 'WINDOW'
	bl_context = "scene"
	bl_description = 'Properties assigned to a new curve'
	
	def draw(self, context):
		layout = self.layout		
		column = layout.column()	
		row=column.row(align=True)
		row.scale_x = 0.7
		row.label(text='Resolution:')
		row.prop(context.scene, 'bt_resolution', text='')
		column.separator()
	
		row=column.row(align=True)
		row.scale_x = 0.7
		row.label(text='Pipe:')
		row.prop(context.scene, 'bt_pipe_radius', text='')
		row.prop(context.scene, 'bt_pipe_resolution', text='')
		column.separator()

		row=column.row(align=True)
		row.scale_x = 0.7
		row.label(text='Band:')
		row.prop(context.scene, 'bt_band_width', text='')
		column.separator()

		row=column.row(align=True)
		row.scale_x = 0.7
		row.label(text='Color:')
		row.prop(context.scene, 'bt_color', text='')
		column.separator()

class BT_BuildCurvePanel(Panel):
	bl_label = "Build Curve"
	bl_idname = "OBJECT_PT_BT_DRAW_PANEL"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_category = 'Camso Curve Toolkit'
	bl_order = 0
	bl_options =  {'DEFAULT_CLOSED'}

	def draw(self, context):
		layout = self.layout
		wm = context.window_manager		
		column = layout.column(align=True)		

		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25			
		row.operator(BT_DrawBezierLine.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_LINE' else False), icon_value=pcoll['bezier_line_icon'].icon_id)
		row.operator(BT_DrawPolyBezier.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_POLYCURVE_PARABOLA' else False), icon_value=pcoll['polybezier_parabola_icon'].icon_id).is_parabola=True
		row.operator(BT_DrawPolyBezier.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_POLYCURVE_MIN' else False), icon_value=pcoll['polybezier_min_icon'].icon_id).to_bezier=1								 		    
		row.operator(BT_DrawPolyBezier.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_POLYCURVE_MAX' else False), icon_value=pcoll['polybezier_max_icon'].icon_id).to_bezier=2		

		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_DrawBezierCurve.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_CURVE' else False), icon_value=pcoll['bezier_polyline_icon'].icon_id).spline_type='BEZIER'
		row.operator(BT_DrawBezierCurve.bl_idname, text="", depress=(True if wm.bt_modal_on=='BT_POLYLINE' else False), icon_value=pcoll['polyline_icon'].icon_id).spline_type='POLY'	
		row.operator(BT_DrawPolylineCircle.bl_idname, text = "", depress=(True if wm.bt_modal_on=='BT_POLYCIRCLE' else False), icon_value=pcoll['polyline_circle_icon'].icon_id)
		row.operator(BT_DrawPolylineRectangle.bl_idname, text = "", depress=(True if wm.bt_modal_on=='BT_POLYRECTANGLE' else False), icon_value=pcoll['polyline_rectangle_icon'].icon_id)
		
		column = layout.column(align=True)
		row = column.split(align=True)
		row.scale_y = 1.25		
		settings = row.operator('wm.call_panel', text='', icon_value=pcoll['settings_icon'].icon_id)
		settings.name = BT_Settings.bl_idname		

class BT_EditBezierPanel(Panel):
	bl_label = "Edit Bézier"
	bl_idname = "OBJECT_PT_BT_EDIT_BEZIER_PANEL"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_category = 'Camso Curve Toolkit'
	bl_options =  {'DEFAULT_CLOSED'}
	bl_order = 1

	def draw(self, context):
		layout = self.layout
		wm = context.window_manager

		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_Add.bl_idname, text = "", depress=(True if wm.bt_modal_on=='BT_ADD_POINT' else False), icon_value=pcoll['add_icon'].icon_id)
		row.operator(BT_Remove.bl_idname, text = "", icon_value=pcoll['remove_icon'].icon_id)
		row.operator(BT_Move.bl_idname, text = "", icon_value=pcoll['move_icon'].icon_id)
		row.operator(BT_Merge.bl_idname, text = "", icon_value=pcoll['merge_icon'].icon_id)

		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25	
		row.operator(BT_Smooth.bl_idname, text = "", icon_value=pcoll['smooth_icon'].icon_id)
		row.operator(BT_Flatten.bl_idname, text = "", icon_value=pcoll['flatten_icon'].icon_id)
		row.operator(BT_Split.bl_idname, text = "", depress=(True if wm.bt_modal_on=='BT_SPLIT' else False), icon_value=pcoll['split_icon'].icon_id)		
			
		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_Join.bl_idname, text = "", icon_value=pcoll['join_icon'].icon_id)		
		row.operator(BT_Offset.bl_idname, text = "", icon_value=pcoll['offset_icon'].icon_id)
		row.operator(BT_Snap.bl_idname, text = "", depress=(True if wm.bt_modal_on=='BT_SNAP' else False), icon_value=pcoll['snap_icon'].icon_id)
		
		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25	
		row.operator(BT_SetBezierHandleType.bl_idname, text = "", icon_value=pcoll['set_handle_type_icon'].icon_id)	
		row.operator(BT_Reverse.bl_idname, text = "", icon_value=pcoll['reverse_icon'].icon_id)
		row.operator(BT_Convert.bl_idname, text = "", icon_value=pcoll['convert_icon'].icon_id)

		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_TransferCurveData.bl_idname, text = "", icon_value=pcoll['transfer_icon'].icon_id)
		row.operator(BT_BezierInterpolate.bl_idname, text = "", icon_value=pcoll['interpolate_icon'].icon_id)
		row.operator(BT_ChangeColor.bl_idname, text = "", icon_value=pcoll['change_color_icon'].icon_id)		

		column = layout.column(align=True)
		row = column.split(align=True)
		row.scale_y = 1.25				
		row.operator(BT_CalcCurveLength.bl_idname, text = "", icon_value=pcoll['get_length_icon'].icon_id)
		row.operator(BT_SetCurveLength.bl_idname, text = "", icon_value=pcoll['set_length_icon'].icon_id)		

class BT_BlendPanel(Panel):
	bl_label = "Blend Bézier"
	bl_idname = "OBJECT_PT_BT_BLEND_PANEL"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_category = 'Camso Curve Toolkit'
	bl_order = 2
	bl_options =  {'DEFAULT_CLOSED'}

	def draw(self, context):
		layout = self.layout
		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_Blend.bl_idname, text = "", icon_value=pcoll['blend2x0_icon'].icon_id)
		row.operator(BT_Blend1Profile2Rails.bl_idname, text = "", icon_value=pcoll['blend2x1_icon'].icon_id)
		row.operator(BT_Blend2Profiles2Rails.bl_idname, text = "", icon_value=pcoll['blend2x2_icon'].icon_id)

class BT_BuildMeshPanel(Panel):
	bl_label = "Build Mesh"
	bl_idname = "OBJECT_PT_BT_MESH_PANEL"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_category = 'Camso Curve Toolkit'
	bl_order = 3
	bl_options =  {'DEFAULT_CLOSED'}

	def draw(self, context):
		layout = self.layout
		column = layout.column(align=True)
		row = column.split(align=True)		
		row.scale_y = 1.25
		row.operator(BT_Loft.bl_idname, text = "", icon_value=pcoll['loft_icon'].icon_id)		
		row.operator(BT_Patch.bl_idname, text = "", icon_value=pcoll['patch_icon'].icon_id)

class BT_ActiveObjectPanel(Panel):
	bl_label = "Active Object Statistics"
	bl_idname = "OBJECT_PT_BT_ACTIVE_OBJECT_PANEL"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_category = 'Camso Curve Toolkit'
	bl_options =  {'DEFAULT_CLOSED'}
	bl_order = 4

	@classmethod
	def poll(cls, context):
		return context.object is not None

	def draw(self, context):
		layout = self.layout		
		column = layout.column(align=True)		
		obj = context.object
		
		if obj is not None:
			column.label(text='Type: ' + (obj.data.splines[0].type if obj.type == 'CURVE' else obj.type))			
			column.separator()			
			column.prop(obj, 'name', text='Name')
			column.separator()

			if obj.type == 'CURVE':
				curve = obj
				spline = obj.data.splines[0]
				if spline.type in {'BEZIER', 'NURBS'}:
					row = column.row(align=True)
					row.label(text='Resolution:')
					row.prop(spline, 'resolution_u', text='')
					column.separator()

				column.label(text='Points count: ' + str(len(spline.bezier_points) if is_bezier(obj) else len(spline.points)) )			
				column.separator()		

				row = column.row(align=True)			
				row.label(text='Pipe:')
				row.prop(curve.data, 'bevel_depth', text='')
				row.prop(curve.data, 'bevel_resolution', text='')	
				column.separator()
				row = column.row(align=True)
				row.label(text='Band:')
				row.prop(curve.data, 'extrude', text='')
				column.separator()				

				row = column.row(align=True)
				row.label(text='Cyclic:')
				row.prop(spline, 'use_cyclic_u', text='')
				column.separator()

			elif obj.type == 'MESH':
				column.label(text='Vertex count: ' + str(len(obj.data.vertices)))	
				column.separator()
			
			row = column.row(align=True)
			row.label(text='Color:')
			row.prop(obj, 'color', text='')
			column.separator()

##################################################################################################

classes = (
	BT_BuildCurvePanel,
	BT_ActiveObjectPanel,
	BT_EditBezierPanel,
	BT_BlendPanel,
	BT_BuildMeshPanel,
	BT_Settings,
	BT_Blend,
	BT_Blend1Profile2Rails,
	BT_Blend2Profiles2Rails,
	BT_DrawBezierCurve,
	BT_DrawBezierLine,
	BT_DrawPolyBezier,
	BT_DrawPolylineCircle,
	BT_DrawPolylineRectangle,
	BT_Add,
	BT_Move,
	BT_Flatten,
	BT_Offset,
	BT_Remove,
	BT_Split,
	BT_Join,
	BT_Reverse,
	BT_TransferCurveData,
	BT_CalcCurveLength,
	BT_SetCurveLength,
	BT_Convert,
	BT_Patch,
	BT_Loft,
	BT_Snap,
	BT_Smooth,
	BT_Merge,
	BT_SetBezierHandleType,
	BT_BezierInterpolate,
	BT_ChangeColor
)

def register():
	global pcoll
	pcoll = bpy.utils.previews.new()
	dir = os.path.join(os.path.dirname(__file__), "icons")
	pcoll.load('bezier_line_icon', dir + "/bezier_line.png", "IMAGE")
	pcoll.load('bezier_polyline_icon', dir + "/bezier_polyline.png", "IMAGE")
	pcoll.load('polybezier_parabola_icon', dir + "/polybezier_parabola.png", "IMAGE")
	pcoll.load('polybezier_min_icon', dir + "/polybezier_min.png", "IMAGE")
	pcoll.load('polybezier_max_icon', dir + "/polybezier_max.png", "IMAGE")
	pcoll.load('polyline_icon', dir + "/polyline.png", "IMAGE")
	pcoll.load('polyline_circle_icon', dir + "/polyline_circle.png", "IMAGE")
	pcoll.load('polyline_rectangle_icon', dir + "/polyline_rectangle.png", "IMAGE")
	pcoll.load('settings_icon', dir + "/settings.png", "IMAGE")
	pcoll.load('add_icon', dir + "/add.png", "IMAGE")
	pcoll.load('remove_icon', dir + "/remove.png", "IMAGE")
	pcoll.load('move_icon', dir + "/move.png", "IMAGE")
	pcoll.load('smooth_icon', dir + "/smooth.png", "IMAGE")
	pcoll.load('merge_icon', dir + "/merge.png", "IMAGE")
	pcoll.load('flatten_icon', dir + "/flatten.png", "IMAGE")
	pcoll.load('split_icon', dir + "/split.png", "IMAGE")
	pcoll.load('join_icon', dir + "/join.png", "IMAGE")
	pcoll.load('offset_icon', dir + "/offset.png", "IMAGE")
	pcoll.load('snap_icon', dir + "/snap.png", "IMAGE")
	pcoll.load('set_handle_type_icon', dir + "/set_handle_type.png", "IMAGE")
	pcoll.load('reverse_icon', dir + "/reverse.png", "IMAGE")
	pcoll.load('convert_icon', dir + "/convert.png", "IMAGE")
	pcoll.load('transfer_icon', dir + "/transfer.png", "IMAGE")
	pcoll.load('interpolate_icon', dir + "/interpolate.png", "IMAGE")
	pcoll.load('change_color_icon', dir + "/change_color.png", "IMAGE")		
	pcoll.load('get_length_icon', dir + "/get_length.png", "IMAGE")
	pcoll.load('set_length_icon', dir + "/set_length.png", "IMAGE")
	pcoll.load('blend2x0_icon', dir + "/blend2x0.png", "IMAGE")
	pcoll.load('blend2x1_icon', dir + "/blend2x1.png", "IMAGE")
	pcoll.load('blend2x2_icon', dir + "/blend2x2.png", "IMAGE")
	pcoll.load('loft_icon', dir + "/loft.png", "IMAGE")
	pcoll.load('patch_icon', dir + "/patch.png", "IMAGE")

	for cls in classes:
		bpy.utils.register_class(cls)
	
	bpy.types.Scene.bt_resolution = bpy.props.IntProperty(default=12, min=3, name='', description='Curve\'s smoothness. It is a value stored in the scene and assigned to every new curve. For changing resolution of an active curve go to [Active Spline] settings')
	bpy.types.Scene.bt_color = bpy.props.FloatVectorProperty(name='Color', subtype='COLOR', 
		size=4, min=0, max=1.0, default=(0.0, 1.0, 0.5, 1.0),
		description='Color assigned to every new curve. Only takes effect when [Viewport Shading-> Wireframe Color-> Object]')

	bpy.types.Scene.bt_pipe_radius = bpy.props.FloatProperty(name='Pipe Radius', description='Pipe Radius', min=0, step=1)
	bpy.types.Scene.bt_pipe_resolution = bpy.props.IntProperty(name='Pipe Resolution', description='Pipe Section Resolution', min=0)
	bpy.types.Scene.bt_band_width = bpy.props.FloatProperty(name='Band Width', description='Band Width', min=0, step=1)

	bpy.types.WindowManager.bt_modal_on = bpy.props.EnumProperty(items=[
		('NONE','',''),
		('BT_LINE','',''),
		('BT_CURVE','',''),
		('BT_POLYCURVE_PARABOLA','',''),			
		('BT_POLYCURVE_MIN','',''),	
		('BT_POLYCURVE_MAX','',''),	
		('BT_POLYLINE','',''),
		('BT_POLYCIRCLE','',''),
		('BT_POLYRECTANGLE','',''),
		('BT_SPLIT','',''),
		('BT_ADD_POINT','',''),
		('BT_SNAP','',''),		
		])

def unregister():
	for cls in reversed(classes):
		bpy.utils.unregister_class(cls)	

	del bpy.types.Scene.bt_resolution

	bpy.utils.previews.remove(pcoll)
