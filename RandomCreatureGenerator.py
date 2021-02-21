#------------------------------------------------------------------
#       RANDOM CREATURE GENERATOR
#       Create by Jennifer Lindner 
#       as part of a masters thesis project
#       Released under the MIT License.
#------------------------------------------------------------------

import sys
import os
import os.path
import bpy
import bmesh
import datetime
import mathutils
import math
import numpy
import time
from math import sqrt, radians, pi, cos, sin
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
from random import random, seed, uniform, randint, randrange, choices, sample
from enum import IntEnum
from colorsys import hls_to_rgb
from pprint import pprint


#------------------------------------------------------------------
#           Global Parameters
#------------------------------------------------------------------
# DOES NOT INCLUDE PARAMETERS STORING RANDOM NUMBERS (e.g. uniform(0,1))
# as these need to be calculated with set seed

#-------------------------
#       General
#-------------------------

# If seed is empty random will use current time as seed instead 
# Same seed will get the same result
SEED = ""    
MERGE_OBJECTS = False
SMOOTH_OBJECTS = True

# SOLVES: Recalculations (e.g. of positions) running for too long, ends loop after certain amount of time
# Timeout in seconds
TIMEOUT = 5


#-------------------------
#       Torso
#-------------------------

# SOLVES: Generating to many bodyparts that could intersect and cross eachother,
# by only generating some parts or at chance nothing
# Remove body parts from this list to generate more specific creatures
BODY_PARTS_LIST = ["Legs", "Wings", "Fins", "Tail", "Nothing","Nothing"]
BODY_NR_BODY_PARTS = 2 

BODY_FACE_SCALE_CHANCE = 0.7

BODY_NR_TORSO_PARTS = range(1, 11)
# Needs same length as BODY_NR_TORSO_PARTS
BODY_NR_TORSO_PARTS_CHANCE = [0.3, 0.2, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.025, 0.025]

# Recalculate the center of the torso after extrusion and deformation
BODY_RECALCULATE_CENTER = False
# Recalculate center via specific parameter
# Options: 
# ORIGIN_CENTER_OF_MASS
# ORIGIN_GEOMETRY
# ORIGIN_CENTER_OF_VOLUME
BODY_RECALCULATE_CENTER_TYPE = "ORIGIN_CENTER_OF_VOLUME"


#-------------------------
#       Head and Neck
#-------------------------

# Either use torso center or (0,0,0) 
NECK_USE_TORSO_CENTER = False

NECK_CHANCE = 0.7

NECK_LENGTH_EXTENDED_CHANCE = 0.1


#-------------------------
#       Eyes
#-------------------------

EYES_NR_OF_EYES = range(1, 4)
# Needs same length as EYES_NR_OF_EYES
EYES_NR_OF_EYES_CHANCE = [0.7, 0.2, 0.1]

EYE_OBJ_SEGMENTS_U = 10
EYE_OBJ_SEGMENTS_V = 10


#-------------------------
#       Fins
#-------------------------

FINS_SIDE_FIN_CHANCE = 0.7
FINS_TOP_FIN_CHANCE = 0.7

# There are two different methods of generating the top fin, 
# one using a xyz-function the other using a z-function
# set chance of how often either or will be used (never both)
FINS_TOP_FIN_FUNCTION_CHANCE = 0.5

FINS_TOP_FIN_MESH_RESOLUTION_X = 100
FINS_TOP_FIN_MESH_RESOLUTION_Y = 100

FINS_REDO = True


#-------------------------
#       Wings
#-------------------------

# There are two different methods of generating wings, 
# one using a xyz-function the other using extruded cubes
# set chance of how often either or will be used (never both)
WINGS_FUNCTION_OBJECT_CHANCE = 0.5

WINGS_REDO = True


#-------------------------
#       Legs
#-------------------------

# Used in randInt so will exclude max number
LEGS_MAX_NR_LEGS = 4

    
#------------------------------------------------------------------
#           Body Part Functions
#------------------------------------------------------------------

        
def generate_creature(random_seed=''):
    # Set seed for random libraray function, if given
    if random_seed:
        seed(random_seed)

    # Create a Creature and save seed
    creature = Creature(random_seed)
    
    # Create Torso
    mesh = bmesh.new()
    bmesh.ops.create_cube(mesh, size=1)
    
    # Scale randomly to create more interesting shapes
    BODY_SCALE_X = uniform(2,5)
    BODY_SCALE_Y = uniform(1,5)
    BODY_SCALE_Z = uniform(1,5)
    bmesh.ops.scale(mesh, vec=(BODY_SCALE_X, BODY_SCALE_Y, BODY_SCALE_Z), verts=mesh.verts)
    
    # At chance scale some faces randomly, additionally save the face face at the back of the body
    BACK_FACE= ""
    for face in mesh.faces:
        if face.index == 0:
            BACK_FACE = face  
        if face.normal.y == 0:
            if random() > BODY_FACE_SCALE_CHANCE:
                BODY_FACE_SCALE_X = uniform(0.7,1.5)
                BODY_FACE_SCALE_Y = uniform(0.7,1.5)
                scale_face(mesh, face, BODY_FACE_SCALE_X ,BODY_FACE_SCALE_Y, 1)
    
    # Weighted chance of how long the torso will be / how many parts will be generated   
    NR_TORSO_PARTS = choices(BODY_NR_TORSO_PARTS, weights = BODY_NR_TORSO_PARTS_CHANCE, k = 1)[0]   
    
    # Extrude and scale faces       
    for x in range(0, NR_TORSO_PARTS):
        BODY_FACE_EXTRUDE_LENGTH = uniform(0.5, 2)
        BODY_FACE_EXTRUDE_NORMAL_MODIFIER = uniform(-0.2, 0.2)  
        BACK_FACE = extrude_face(mesh, BACK_FACE, BODY_FACE_EXTRUDE_LENGTH, BODY_FACE_EXTRUDE_NORMAL_MODIFIER)
        
        BODY_FACE_UNIFORM_SCALE = uniform(0.5, 1.2)
        scale_face(mesh, BACK_FACE, BODY_FACE_UNIFORM_SCALE, BODY_FACE_UNIFORM_SCALE, 1)
      

    obj_torso = addMeshToScene(mesh, (f'Creature_Torso'))
    
    # Recalculate objects center after modifications
    if BODY_RECALCULATE_CENTER:
        obj_torso.select_set(True)
        bpy.ops.object.origin_set(type=BODY_RECALCULATE_CENTER_TYPE, center='MEDIAN')
    
    
    # Save as body part and create torso
    bp_torso = BodyPart(obj_torso, obj_torso.location)
    creature.torso = bp_torso  
            
    # Start creating various body parts
    creature.head = createHead(bp_torso)
    
    # Choose a number of body parts from available list
    BODY_PARTS = sample(BODY_PARTS_LIST, k=BODY_NR_BODY_PARTS)
    
    # DEBUG - See which body parts are chosen
    print(BODY_PARTS)
    
    # Create selected body parts
    for part in BODY_PARTS:
        if part == "Legs":
            creature.legs = createLegs(bp_torso)
        if part == "Wings":
            creature.wings = createWings(bp_torso)
        if part == "Fins":
            creature.fins = createFins(bp_torso, creature.head.bp_head.center)
        if part == "Tail":
            creature.tail = createTail(bp_torso, BACK_FACE)
        if part == "Nothing":
            print("Do Nothing")
            
            
            
    if MERGE_OBJECTS:        
        # Merge all generated objects
        merge()
        
        if SMOOTH_OBJECTS:
            # Smooth object, only if they where merged 
            smooth()
            
    # Deselect everything, just for better visual 
    bpy.ops.object.select_all(action='DESELECT')

def createHead(bp_torso):
    
    # Get random frontfacing location on main body
    HEAD_TORSO_RAYCAST_VEC_X = uniform(0,5)
    # Y is 0 to keep head center
    HEAD_TORSO_RAYCAST_VEC_Y = 0
    HEAD_TORSO_RAYCAST_VEC_Z = uniform(0,5)
    
    # Use actual center of torso object or origin vector 
    center_vector = Vector((0,0,0))
    new = center_vector - bp_torso.center
    origin_point = center_vector if NECK_USE_TORSO_CENTER else new
    #local_coord = bp_torso.obj.matrix_world.inverted() * origin_point
    
    (world_location, hit, hit_vector, normal_vector, faceID) = getLocationRayCast(
        bp_torso.obj,
        origin_point,
        (HEAD_TORSO_RAYCAST_VEC_X, HEAD_TORSO_RAYCAST_VEC_Y, HEAD_TORSO_RAYCAST_VEC_Z))
    
    # Saves ancor point for head, either on torso (when no neck is generated) or neck
    NECK_END = world_location
    
    bp_neck = ""
    
    # Chance to generate neck
    if random() < NECK_CHANCE:
        # Random vector for neck
        NECK_VECTOR_X = uniform(0,10)
        # Y is 0 to keep head center
        NECK_VECTOR_Y = 0
        NECK_VECTOR_Z = uniform(0,10)
        neck_vector = Vector((NECK_VECTOR_X, NECK_VECTOR_Y, NECK_VECTOR_Z))
        
        # Random neck with chance to generate very long neck
        NECK_LENGTH = uniform(0.7,2)
        NECK_LENGTH_EXTENDED = uniform(5,7)
        if random() < NECK_LENGTH_EXTENDED_CHANCE:
            NECK_LENGTH = NECK_LENGTH_EXTENDED
        
        # Get center point towards neck_vector
        center = Vector(getPointOnVector(world_location, neck_vector, (NECK_LENGTH/2), -0.5))
        
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1)
        
        # Scale randomly, scale neck to match NECK_LENGTH 
        NECK_SCALE_X = uniform(0.5,1)
        NECK_SCALE_Y = uniform(0.5,1)
        bmesh.ops.scale(bm, 
            vec=(NECK_SCALE_X, NECK_SCALE_Y, NECK_LENGTH), 
            verts=bm.verts)

        obj_neck = addMeshToScene(bm, 'Creature_Neck')
        obj_neck.location = center   
        
        # Calculate and set rotation of obj to align with neck_vector
        (phi, theta) = getRotationRad(world_location, center)
        obj_neck.rotation_euler[1] = theta 
        obj_neck.rotation_euler[2] = phi
        
        bp_neck = BodyPart(obj_neck, center)
        
        NECK_END = Vector(getPointOnVector(center, neck_vector, (NECK_LENGTH/2)))
    
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1)    
    
    # Scale head randomly 
    HEAD_SCALE_X = uniform(0.5,2)
    HEAD_SCALE_Y = uniform(0.5,2)
    HEAD_SCALE_Z = uniform(0.5,2)
    bmesh.ops.scale(bm, vec=(HEAD_SCALE_X, HEAD_SCALE_Y, HEAD_SCALE_Z), verts=bm.verts)
    
    # This needs to be done to ensure all faces are indexed correctly  
    bm.faces.ensure_lookup_table()
    
    # Get front looking face and extrude and scale for a more interesting head shape
    HEAD_FACE_EXTRUDE_LENGTH = uniform(0.5, 2)
    HEAD_FACE_EXTRUDE_SCALE_X = uniform(0.5, 1)
    HEAD_FACE_EXTRUDE_SCALE_Y = uniform(0.5, 1)
    front_face = bm.faces[2]
    extruded_face = extrude_face(bm, front_face, HEAD_FACE_EXTRUDE_LENGTH)
    scale_face(bm, extruded_face, HEAD_FACE_EXTRUDE_SCALE_X, HEAD_FACE_EXTRUDE_SCALE_Y, 1)
    
    obj = addMeshToScene(bm, 'Creature_Head')
    obj.location = NECK_END
    
    bp_head = BodyPart(obj, NECK_END)
    
    head = Head(bp_head, bp_neck)
    
    createEyes(head)
    
    return head


def createEyes(head):  
    
    NR_OF_EYES = choices(EYES_NR_OF_EYES, weights = EYES_NR_OF_EYES_CHANCE, k = 1)[0]
    
    for x in range(0, NR_OF_EYES):
        
        isIntersecting = True
        
        # NOTE: While bmesh.ops.create_uvsphere requires a "diameter" it is in fact used as a radius 
        # Going forward it will be named as such
        # It can be tested by setting the "diameter" to 1 and comparing the size with the grid on a normal scale 
        # Supposedly fixed: 
        # https://developer.blender.org/T52923
        EYES_RADIUS = uniform(0.1,0.3)
        
        rayCast = ""        

        timeout_start = time.time()
        
        # SOLVES: Eyes intersecting with eachother, 
        # generate a new position if the eye would intersect with other already generate eyes
        while isIntersecting and (time.time() < timeout_start + TIMEOUT):
            # Random position on head
            EYES_HEAD_RAYCAST_VEC_X = uniform(0,1)
            EYES_HEAD_RAYCAST_VEC_Y = uniform(0.5,1)
            EYES_HEAD_RAYCAST_VEC_Z = uniform(0,1)
            rayCast = getLocationRayCast(head.bp_head.obj,
                (0,0,0), 
                (EYES_HEAD_RAYCAST_VEC_X, EYES_HEAD_RAYCAST_VEC_Y, EYES_HEAD_RAYCAST_VEC_Z))
            
            # Check if current positon might cause intersection with existing eyes    
            for eye in head.GetEyes():
                if checkDistance(rayCast[0], EYES_RADIUS, eye.center, eye.radius):
                    isIntersecting = True
                    break
                else:
                    isIntersecting = False
            else:
                isIntersecting = False
                
        # Position found, proceed to create eye obj                       
        if not isIntersecting:                         
            bm = bmesh.new()
            bmesh.ops.create_uvsphere(bm, u_segments=EYE_OBJ_SEGMENTS_U, v_segments=EYE_OBJ_SEGMENTS_V, diameter=EYES_RADIUS)
            obj_eye = addMeshToScene(bm, (f'Creature_Eye_{x}'))
            
            # Get data from successful raycast
            (world_location, hit, hit_vector, normal_vector, faceID) = rayCast
            obj_eye.location = world_location
                
            mirrorObj(obj_eye)
            
            # Create Eye object and save in head            
            bp_eye = Eye(obj_eye, world_location, EYES_RADIUS)
            head.AddEye(bp_eye)
 

def createFins(bp_main, head_center):
    fins = []
    
    # Chance that side fins are generated 
    if random() < FINS_SIDE_FIN_CHANCE:
        
        found = False
        
        world_location = None
        
        timeout_start = time.time()
        
        # SOLVES: Fins being generated on the front/top/bottom/back face causing weird sturctures because of wrong rotation, etc.. 
        # Generate a new position if the fin is not generated on the side faces
        while not found and (time.time() < timeout_start + TIMEOUT):
            FINS_SIDE_FIN_RAYCAST_VEC_X = uniform(-1, 0)
            FINS_SIDE_FIN_RAYCAST_VEC_Y = uniform(0, 1)
            FINS_SIDE_FIN_RAYCAST_VEC_Z = uniform(0,0.5)
            side_fin_vector = Vector((FINS_SIDE_FIN_RAYCAST_VEC_X, FINS_SIDE_FIN_RAYCAST_VEC_Y, FINS_SIDE_FIN_RAYCAST_VEC_Z))
            (world_location, hit, hit_vector, normal_vector, faceID) = getLocationRayCast(bp_main.obj, (0,0,0), side_fin_vector) 
            
            # Get hit face and see if its correct face        
            face = getFace(bp_main.obj, faceID)
            if face.normal.x > -0.5 and face.normal.x < 0.5 and face.normal.z > -0.5 and face.normal.z < 0.5:
                found = True

        context = bpy.context 
        
        # Create a semi random function to create a fin like shape
        x_divide= uniform(1,4)
        y_divide = uniform(1,4)
        x_function = choices(["sin(v)", "cos(v)"], k=1)[0]
        
        # Combine random values with template functions
        x_equation = f"({x_function} + u * cos(v) * sin(v)) / {x_divide}"
        y_equation = f"(u * sin(v)) / {y_divide} "
        z_equation = f"cos(u)"

        # Generate mesh with functions
        bpy.ops.mesh.primitive_xyz_function_surface(
            x_eq=x_equation,
            y_eq=y_equation,
            z_eq=z_equation)
            
        fin_obj = context.object
        fin_obj.name = "Creature_Side_Fin"  
        
        me = fin_obj.data
        bm = bmesh.from_edit_mesh(me)
        
        # Remove excess verticies that wont be part of the fin mesh
        verts = [f for f in bm.verts if f.co.z < 0.9 or f.co.y < 0]
        bmesh.ops.delete(bm, geom=verts)
        
        # Clean up mesh
        
        # Delete loose verticies, edges, faces
        bpy.ops.mesh.delete_loose()
        
        # Create coherent and connected mesh, filling in holes, etc..
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.convex_hull()
        
        bmesh.update_edit_mesh(me)
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Reset pivot point to where the fin would connect to the torso
        fin_obj.location = (0,0,0)
        # Curser location is dummy for TODO
        saved_location = context.scene.cursor.location
        context.scene.cursor.location = Vector((0,0.2,0.9))
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        context.scene.cursor.location = saved_location
        fin_obj.location=(world_location)
        
        bp_fin = Fin(fin_obj, world_location, [x_equation, y_equation, z_equation])
        
        fins.append(bp_fin)
        
        mirrorObj(fin_obj)
        
    # Chance that top fins are generated   
    if random() < FINS_TOP_FIN_CHANCE:
        
        context = bpy.context  
    
        found = False
        
        world_location = None
        
        # SOLVES: Fin being generated infront of the head
        # making sure it starts behind it or inside it 
        while not found:
            FINS_TOP_FIN_RAYCAST_VEC_X = uniform(-1, 0.5)
            FINS_TOP_FIN_RAYCAST_VEC_Y = 0
            FINS_TOP_FIN_RAYCAST_VEC_Z = uniform(0.1,1)
            top_fin_vector = Vector((FINS_TOP_FIN_RAYCAST_VEC_X, FINS_TOP_FIN_RAYCAST_VEC_Y, FINS_TOP_FIN_RAYCAST_VEC_Z))
            (world_location, hit, hit_vector, normal_vector, faceID) = getLocationRayCast(bp_main.obj, (0,0,0), top_fin_vector)         
            
            if head_center.x > world_location.x:
                found = True
         
        # Use either xyz-function or z-function, these generate different shapes         
        if random() < FINS_TOP_FIN_FUNCTION_CHANCE:
            # Create a semi random function to create a fin like shape      
            x_divide = uniform(1,4)
            z_divide = uniform(1,4)   
            z_divide_2 = uniform(2,4)
                    
            x_equation = f"(u*sin(v/2)*-1)/{x_divide}"
            y_equation = f"0"
            z_equation = f"(sin(v)+u*cos(v/{z_divide_2})*sin(v))/{z_divide}"  

            bpy.ops.mesh.primitive_xyz_function_surface(
                x_eq=x_equation,
                y_eq=y_equation,
                z_eq=z_equation)
                    
            fin_obj = context.object
            fin_obj.name = "Creature_Top_Fin"  
            
            me = fin_obj.data
            bm = bmesh.from_edit_mesh(me)
            
            # Delete unused verticies/ edges/faces, deleting the front half of the function mesh
            verts = [f for f in bm.verts if f.co.x > 0]
            bmesh.ops.delete(bm, geom=verts)
            bpy.ops.mesh.delete_loose()
            bmesh.update_edit_mesh(me)
            
            bpy.ops.object.mode_set(mode='OBJECT')
            
            adjusted_location = (world_location.x, world_location.y, world_location.z-0.2)
            fin_obj.location =  adjusted_location      
            bp_fin = Fin(fin_obj, adjusted_location, [x_equation, y_equation, z_equation])
            fins.append(bp_fin)
            
        else:
            # Create a semi random function to create a fin like shape
            x_scale = uniform(1,4)
            sin_divide = uniform(1,2)
            
            equation = f"sin(x*{x_scale})/{sin_divide}+1"
            
            bpy.ops.mesh.primitive_z_function_surface(
                equation = equation,
                div_x = FINS_TOP_FIN_MESH_RESOLUTION_X, 
                div_y = FINS_TOP_FIN_MESH_RESOLUTION_Y)
            
            fin_obj = context.object
                
            fin_obj.name = "Creature_Top_Fin"  
            
            FINS_TOP_FIN_SCALE_X = uniform(0.5,2)    
            FINS_TOP_FIN_SCALE_Y = 0.01
            FINS_TOP_FIN_SCALE_Z = 1
            
            # Scale curve mesh some more
            bpy.ops.transform.resize(value=(FINS_TOP_FIN_SCALE_X, FINS_TOP_FIN_SCALE_Y, FINS_TOP_FIN_SCALE_Z))
            
            # Extrude in z-direction to give curve thickness
            bpy.ops.object.editmode_toggle()
            bpy.ops.mesh.extrude_region_move( TRANSFORM_OT_translate={"value":(0, 0, -2)})
            bpy.ops.transform.resize(value=(0.5, 1, 1), proportional_size=1)
            bpy.ops.object.editmode_toggle()
            
            adjusted_location = (world_location.x, world_location.y, world_location.z-0.2)
            fin_obj.location =  adjusted_location      
            bp_fin = Fin(fin_obj, adjusted_location, [equation])
            fins.append(bp_fin)
            
    # Remove Fins that may intersect with other body parts 
    removed_fins = False
    for fin in fins:
        removed_fins = removeIntersecting([fin], ["Creature_Leg", "Creature_Side_Fin", "Creature_Top_Fin", "Creature_Wing"])    
    
    if FINS_REDO and removed_fins:
        fins = createFins(bp_main, head_center)        

    return fins   


def createTail(bp_main, back_face):
    # get the center point of the furthest face in the back
    world_location = back_face.calc_center_bounds()
    
    # Create bezier curve to start tail with some random x rotation
    TAIL_ROTATION = random()
    bpy.ops.curve.primitive_bezier_curve_add(
        enter_editmode=True, 
        align='WORLD', 
        location=world_location,
        rotation=(TAIL_ROTATION,0,0))
    
    # Select the end of the tail to extend it
    bpy.ops.curve.de_select_last()
    
    # Extrude and move around
    TAIL_EXTEND_X = uniform(-1,0)
    TAIL_EXTEND_Y = uniform(-1,1)
    TAIL_EXTEND_Z = uniform(-1,1)
    bpy.ops.curve.extrude_move(
        CURVE_OT_extrude={"mode":'TRANSLATION'},
        TRANSFORM_OT_translate={"value":(TAIL_EXTEND_X, TAIL_EXTEND_Y, TAIL_EXTEND_Z)})

    bpy.ops.object.editmode_toggle()
    
    # Bevel (round) mesh 
    TAIL_BEVEL_DEPTH = uniform(0.01, 0.2)
    bpy.context.object.data.bevel_depth = TAIL_BEVEL_DEPTH
    bpy.ops.object.convert(target='MESH')
    
    tail_obj = bpy.context.active_object
    tail_obj.name = "Creature_Tail"
    
    # Fill mesh to avoid holes and problems later when smoothing
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.fill()
    
    bpy.ops.object.editmode_toggle()
    
    bp_tail = BodyPart(tail_obj, world_location)
    
    return bp_tail    


def createWings(bp_main): 
    context = bpy.context  
    
    bp_wing = ""
    
    WING_RAYCAST_VEC_X = uniform(-0.2, 0.2)
    WING_RAYCAST_VEC_Y = 1
    WING_RAYCAST_VEC_Z = uniform(0,0.5)
    wing_vector = Vector((WING_RAYCAST_VEC_X, WING_RAYCAST_VEC_Y, WING_RAYCAST_VEC_Z))
    (world_location, hit, hit_vector, normal_vector, faceID) = getLocationRayCast(bp_main.obj, (0,0,0), wing_vector)   
    
    # Either use xyz-function or mesh-extrusion method
    if random() < WINGS_FUNCTION_OBJECT_CHANCE:
                        
        x_divide = uniform(1,4)
        y_divide = uniform(2,4)
             
        x_equation = f"((sin(v) + u * sin(v) * sin(v)) / {x_divide}) * -1"
        y_equation = f"(u * sin(v)) / {y_divide} "
        z_equation = f"cos(u)"
        
        bpy.ops.mesh.primitive_xyz_function_surface(
            x_eq=x_equation,
            y_eq=y_equation,
            z_eq=z_equation)
       
        wing_obj = context.object
        wing_obj.name = "Creature_Wing"  
        
        me = wing_obj.data
        bm = bmesh.from_edit_mesh(me)

        # Delete extra verticies/ edges/ faces, specifically half of the generated curve, and underpart
        verts = [f for f in bm.verts if f.co.z < 0.9 or f.co.y < 0]
        bmesh.ops.delete(bm, geom=verts)
        
        # Fix mesh 
        bpy.ops.mesh.delete_loose()
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.convex_hull()
        bmesh.update_edit_mesh(me)
        
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Reset obj location to where wing would meet the torso
        wing_obj.location = (0,0,0)
        saved_location = context.scene.cursor.location
        context.scene.cursor.location = Vector((0,0.2,0.9))
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
        context.scene.cursor.location = saved_location
        
        # Recess into torso slightly
        adjusted_location = (world_location.x, world_location.y - 0.2,world_location.z)
        wing_obj.location = adjusted_location
        
        # Rotate to look a bit nicer
        bpy.ops.transform.rotate(value=-1, orient_axis='X', orient_type='LOCAL')
        bpy.ops.transform.rotate(value=0.2, orient_axis='Y', orient_type='LOCAL')
        
        mirrorObj(wing_obj)
        
        bp_wing = Wing(wing_obj, adjusted_location, [x_equation, y_equation, z_equation])
        
    else:
        bm = bmesh.new()
        
        bmesh.ops.create_cube(bm, size=1) 
        
        # Scale wing to be flat 
        WING_SCALE_X = uniform(1, 2)
        WING_SCALE_Y = uniform(0.5, 1) 
        WING_SCALE_Z = uniform(0.1, 0.3)
        bmesh.ops.scale(bm, vec=(WING_SCALE_X, WING_SCALE_Y, WING_SCALE_Z), verts=bm.verts)

        # This needs to be done to ensure all faces are indexed correctly 
        bm.faces.ensure_lookup_table()
        
        # Select tip of the wing and scale 
        face = bm.faces[1]
        WING_FACE_SCALE_X = uniform(1, 1.5)
        WING_FACE_SCALE_Y = uniform(1, 1.5)
        scale_face(bm, face,  WING_FACE_SCALE_X,  WING_FACE_SCALE_Y, 1)
        
        # Move face slighly to generate curves 
        WING_FACE_MOVE_X = uniform(-0.5,0)
        WING_FACE_MOVE_Y = 0
        WING_FACE_MOVE_Z = 0
        move_face(bm, face, (WING_FACE_MOVE_X, WING_FACE_MOVE_Y, WING_FACE_MOVE_Z))
        
        # Extrude, scale and move wing face to create tip
        x = 0
        WING_NR_OF_PARTS = randint(2,5)
        while x <= WING_NR_OF_PARTS:
            WING_SUB_FACE_EXTRUDE_LENGTH = uniform(0.2, 1)
            WING_SUB_FACE_EXTRUDE_NORMAL_MODIFIER = uniform(-1,-0.5)
            face = extrude_face(bm, face, WING_SUB_FACE_EXTRUDE_LENGTH, WING_SUB_FACE_EXTRUDE_NORMAL_MODIFIER) 
            
            # If last part scale to create pointy end
            if x == WING_NR_OF_PARTS:
                scale_face(bm, face, 0.01, 0.01,1)
            # Scale to get smaller    
            else:
                WING_SUB_FACE_SCALE_X = uniform(0.5, 0.9)
                WING_SUB_FACE_SCALE_Y = uniform(0.8, 0.9)
                scale_face(bm, face, WING_SUB_FACE_SCALE_X, WING_SUB_FACE_SCALE_Y, 1)
            
            # Move face to give nice curve
            WING_SUB_FACE_MOVE_X = uniform(-0.5,0)
            move_face(bm, face, (WING_SUB_FACE_MOVE_X, 0, 0)) 
               
            x = x + 1   
            
        wing_obj = addMeshToScene(bm, (f'Creature_Wing'))
        adjusted_location = (world_location.x, world_location.y - 0.2,world_location.z)
        wing_obj.location = adjusted_location
        
        mirrorObj(wing_obj)
        
        bp_wing = Wing(wing_obj, adjusted_location) 
    
    # Remove Legs that may intersect with other body parts    
    removed_wing = removeIntersecting([bp_wing], ["Creature_Leg", "Creature_Side_Fin", "Creature_Top_Fin", "Creature_Wing"])    
    
    if WINGS_REDO and removed_wing:
        bp_wing = createWings(bp_main)
    
    return bp_wing  

       
def createLegs(bp_main):
    # Number of legs
    
    LEGS_NR_LEGS = randint(2,LEGS_MAX_NR_LEGS)
    
    # Array holding leg objects
    legs = []
    
    # Get random starting location for each leg left side
    for x in range(LEGS_NR_LEGS):
        
        # Exception raised to escape for loops, continue with new while loop 
        continue_i = ContinueI()
    
        # End loop when taking to long
        timeout_start = time.time()
        
        # SOLVES: Legs being too close together, generates new point if too close to other legs
        found_point = False
        leg = ""
        while not found_point and (time.time() < timeout_start + TIMEOUT):
            try: 
                # Generate random point on torso obj from the center, where leg will be placed
                LEGS_RAYCAST_VEC_X = uniform(-1,1)
                LEGS_RAYCAST_VEC_Y = uniform(0.5,1)
                LEGS_RAYCAST_VEC_Z = uniform(-0.5,0)
                uniform_vector = Vector((LEGS_RAYCAST_VEC_X, LEGS_RAYCAST_VEC_Y, LEGS_RAYCAST_VEC_Z))
                
                (world_location, hit, hit_vector, normal_vector, faceID) = getLocationRayCast(bp_main.obj, (0,0,0), uniform_vector)
                leg = Leg(RayCastHitObj(world_location, hit, hit_vector, normal_vector, faceID))
                
                LEGS_CHECK_DISTANCE = 1
                
                # Check if generated location is to close to other legs
                for item in legs:
                    if bool(item):
                        if getDistBetweenPoints(item.main_body_joint.world_vector, world_location) < LEGS_CHECK_DISTANCE:
                            raise continue_i  
                found_point = True               
            except ContinueI:
                continue
               
        if found_point:
            legs.append(leg)
            
    # Sort leg array by x position (back to front)
    legs = sorted(legs, key=lambda legs: (legs.main_body_joint.world_vector.x))
    
    # Create each individual leg
    for index, x in enumerate(legs):
        createLeg(x,  index, LEGS_NR_LEGS, bp_main)
        # Remove Legs that may intersect with other body parts    
        removeIntersecting(x.GetLegParts(), ["Creature_Head", "Creature_Wing", "Creature_Side_Fin", "Creature_Top_Fin"])    
        
        
    return legs
        
     
def createLeg(leg, index, n_of_legs, bp_main):
    
    LEGS_NR_SECTIONS = uniform(1,3)
    
    start = leg.main_body_joint.world_vector
    
    LEGS_PART_LENGTH_Y = uniform(0.5,1)
    LEGS_PART_LENGTH_Z = uniform(0.5,1)

    i = 0
    while i < LEGS_NR_SECTIONS:
    
        # Prevent overlapping in rotation
        # SOVLES: Legs would criss cross
        # Rotation angle start and end point determind my index of leg 
        x_start = -1 + ((2/LEGS_NR_SECTIONS) * index)
        x_end = numpy.clip((x_start + (2/LEGS_NR_SECTIONS)),-1,1)
    
        # Random vectors for rotation
        LEGS_ROTATION_VEC_X = uniform(x_start,x_end)
        LEGS_ROTATION_VEC_Y = uniform(0,2)
        LEGS_ROTATION_VEC_Z = uniform(-2,2)
        random_x_vector = start.x + LEGS_ROTATION_VEC_X
        random_y_vector = start.y + LEGS_ROTATION_VEC_Y
        random_z_vector = start.z + LEGS_ROTATION_VEC_Z
    
        # Random vectors for transformation
        LEGS_PART_LENGTH_X = uniform(1,3)
        
        # Calculate the center of the leg and the point where it ends
        center = Vector(getPointOnVector(start, Vector((random_x_vector,(random_y_vector),random_z_vector)), (LEGS_PART_LENGTH_X/2), -0.4))
        end = Vector(getPointOnVector(center, Vector((random_x_vector,(random_y_vector),random_z_vector)), (LEGS_PART_LENGTH_X/2)))
   
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1)
        
        # Scale to length and get smaller the further down the leg 
        bmesh.ops.scale(bm, vec=(LEGS_PART_LENGTH_Z, LEGS_PART_LENGTH_Y, LEGS_PART_LENGTH_X), verts=bm.verts)
        
        leg_obj = addMeshToScene(bm, (f'Creature_Leg_{index}.{i}'))
        leg_obj.location = center
        
        # Calculate rotation and rotate obj to align with vector
        (phi, theta) = getRotationRad(center, start)
        leg_obj.rotation_euler[1] = theta 
        leg_obj.rotation_euler[2] = phi
        
        mirrorObj(leg_obj)
        leg_part = BodyPart(leg_obj, center)
        leg.AddLegPart(leg_part)
        
        LEGS_PART_LENGTH_Y = numpy.clip((LEGS_PART_LENGTH_Y - uniform(0.1,0.3)), 0.2, 1)
        LEGS_PART_LENGTH_Z = numpy.clip((LEGS_PART_LENGTH_Z - uniform(0.1,0.3)), 0.2, 1)
        start = end
        i += 1    
         
  
#------------------------------------------------------------------
#           Classes
#------------------------------------------------------------------
  

class Creature:
    head = []
    legs = []
    tail = []
    fins = []
    wings = []
    torso = []
    
    def __init__(self, seed = ""):
        self.seed = seed
        

class BodyPart:
    def __init__(self, obj, center):
        self.obj = obj
        self.center = center
        
        
class Head():
    bp_eyes = []
    
    def __init__(self, bp_head, bp_neck = ""):
        self.bp_head = bp_head
        self.bp_neck = bp_neck
        
    def AddEye(self, eye):
        self.bp_eyes.append(eye)
        
    def GetEyes(self):
        return self.bp_eyes   
            
        
class Eye(BodyPart):
    def __init__(self, obj, center, radius):
        self.obj = obj
        self.center = center      
        self.radius = radius  
        
        
class Fin(BodyPart):
    def __init__(self, obj, center, curve_function = ""):
        self.obj = obj
        self.center = center      
        self.curve_function = curve_function      
                      
            
class Wing(BodyPart):
    def __init__(self, obj, center, curve_function = ""):
        self.obj = obj
        self.center = center      
        self.curve_function = curve_function         
   
       
class Leg(BodyPart):
    leg_parts = []
    
    def __init__(self, main_body_joint, center = ""):
        self.main_body_joint = main_body_joint
        self.center = center
     
    def AddLegPart(self, part):
        self.leg_parts.append(part)
        
    def GetLegParts(self):
        return self.leg_parts       
     
     
# Object to hold raycast data for later use         
class RayCastHitObj:
    def __init__(self, world_vector, hit, hit_vector, normal_vector, faceID):
        self.world_vector = world_vector
        self.hit = hit
        self.hit_vector = hit_vector
        self.normal_vector = normal_vector
        self.faceID = faceID           
    

#------------------------------------------------------------------
#           Helper Functions 
#           by https://github.com/a1studmuffin/SpaceshipGenerator
#           released under MIT License
#------------------------------------------------------------------


def extrude_face(bm, face, translate_forwards=0.0, face_normal_modifier=0, extruded_face_list=None):
    new_faces = bmesh.ops.extrude_discrete_faces(bm, faces=[face])['faces']
    
    if extruded_face_list != None:
        extruded_face_list += new_faces[:]
        
    new_face = new_faces[0]
    new_face.normal.z = new_face.normal.z + face_normal_modifier
    bmesh.ops.translate(bm, vec=new_face.normal * translate_forwards, verts=new_face.verts)
    
    return new_face


def scale_face(bm, face, scale_x, scale_y, scale_z):
    face_space = get_face_matrix(face)
    face_space.invert()
    
    bmesh.ops.scale(bm,
        vec=Vector((scale_x, scale_y, scale_z)),
        space=face_space,
        verts=face.verts)
        
            
def get_face_matrix(face, pos=None):
    x_axis = (face.verts[1].co - face.verts[0].co).normalized()
    z_axis = -face.normal
    y_axis = z_axis.cross(x_axis)
    
    if not pos:
        pos = face.calc_center_bounds()

    mat = Matrix()
    mat[0][0] = x_axis.x
    mat[1][0] = x_axis.y
    mat[2][0] = x_axis.z
    mat[3][0] = 0
    mat[0][1] = y_axis.x
    mat[1][1] = y_axis.y
    mat[2][1] = y_axis.z
    mat[3][1] = 0
    mat[0][2] = z_axis.x
    mat[1][2] = z_axis.y
    mat[2][2] = z_axis.z
    mat[3][2] = 0
    mat[0][3] = pos.x
    mat[1][3] = pos.y
    mat[2][3] = pos.z
    mat[3][3] = 1
    return mat            

#------------------------------------------------------------------
#           Helper Functions 
#------------------------------------------------------------------


# Deletes all existing mesh from the scene
def reset_scene():
    for o in bpy.context.scene.objects:
        if o.type == 'MESH' or o.type == 'CURVE':
            o.select_set(True)
        else:
            o.select_set(False)

    bpy.ops.object.delete()


# Merge all final body parts into one object
def merge():
    context = bpy.context
    
    obj = bpy.data.objects['Creature_Torso']
    obj.select_set(state=True)
    
    context.view_layer.objects.active = obj
    
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.join()
    
    obj = bpy.context.active_object
    obj.name = "Creature"
    
    
# Subdivision modifier to smooth out mesh    
def smooth():
    # First Simple Subdivision to fix potential prob with the mesh
    bpy.ops.object.modifier_add(type='SUBSURF')
    bpy.context.object.modifiers["Subdivision"].subdivision_type = 'SIMPLE'
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Subdivision")
    
    # Second Catmull-Clark Subdivision to "round" mesh 
    bpy.ops.object.modifier_add(type='SUBSURF')
    bpy.context.object.modifiers["Subdivision"].levels = 3
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Subdivision")
    
    
def addMeshToScene(bm, name):
    me = bpy.data.meshes.new('Mesh')
    bm.to_mesh(me)
    
    scene = bpy.context.scene
    obj = bpy.data.objects.new(name, me)
    scene.collection.objects.link(obj)
    
    return obj


# Mirror an obj over x-axis / yz-plane
def mirrorObj(obj):
    bpy.ops.object.select_all(action='DESELECT')
    
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    
    me = obj.data
    bm = bmesh.from_edit_mesh(me)
    
    geom = bmesh.ops.mirror(bm, geom=bm.faces[:] + bm.verts[:] + bm.edges[:], axis="Y",
        matrix=obj.matrix_world, 
        merge_dist=-1)["geom"]
        
    bmesh.update_edit_mesh(me)
    
    bpy.ops.object.mode_set(mode='OBJECT')    
     
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.active_object.select_set(False)
    bpy.context.view_layer.objects.active = None    
        
        
def move_face(bm, face, vec):
    bmesh.ops.translate(bm, verts=face.verts, vec=vec) 
    
    
# Returns coordinates of a point along a vector a certain distance from the start    
def getPointOnVector(start, vector, length, offset = 0):
    vector.normalize()
    length = length + (length * offset)
    
    resultx = start.x + vector.x*length
    resulty = start.y + vector.y*length
    resultz = start.z + vector.z*length
    
    return (resultx,resulty,resultz)


# Returns rotation values to align vec_one towards vec_two
def getRotationRad(vec_one, vec_two):
    dx = vec_one.x - vec_two.x
    dy = vec_one.y - vec_two.y
    dz = vec_one.z - vec_two.z
            
    dist = math.sqrt(dx**2 + dy**2 + dz**2)

    phi = math.atan2(dy, dx) 
    theta = math.acos(dz/dist)
    
    return phi, theta


def getFace(obj, faceID):
    f = obj.data.polygons[faceID]
    
    return f

     
def getDistBetweenPoints(pt_one, pt_two):
    dist = math.sqrt(
        ((pt_two.x - pt_one.x)**2) + ((pt_two.y - pt_one.y)**2) + ((pt_two.z - pt_one.z)**2))
    
    return dist


def checkDistance(pt_one, radius_one, pt_two, radius_two):
    dist = getDistBetweenPoints(pt_one, pt_two) 
    
    return True if dist < (radius_one + radius_two) else False 


# Execute raycast from a starting point in a specific direction and return result 
# including the hit-location in object and world coordinates     
def getLocationRayCast(start_obj, start, direction):
    (hit, hit_vector, normal_vector, faceID) = start_obj.ray_cast(start, direction)
    
    mat = start_obj.matrix_world
    
    world_location = mat @ hit_vector
    
    return (world_location, hit, hit_vector, normal_vector, faceID)


# An exception which can be raised to break out of certain loops        
class ContinueI(Exception):
     pass


# Remove newly generated bodyparts if they are intersecting with existing parts 
def removeIntersecting(obj, check_against):
    # Get all meshes 
    objs = [f.obj for f in obj]
    
    # Get all meshes to check against
    other_objs = []
    for other_name in check_against:
        list = [f for f in bpy.data.objects if f.type == 'MESH' and (other_name in f.name)]
        other_objs = other_objs + list
    
    if other_objs != []:
        intersecting = intersectionCheck(objs, other_objs)
        if intersecting:
            for o in objs:
                o.select_set(True) 
            bpy.ops.object.delete() 
            print(f"Removed objects of type {str(obj)} due to intersection")
            return True
    return False    
      

# Test target array if they intersect with any other geometry
def intersectionCheck(obj_array, other_obj_array):
    
    for obj in obj_array:
        
        for other_obj in other_obj_array:
            
            if obj == other_obj:
                continue
            # Create bmesh and fill with data from target objects
            bm1 = bmesh.new()
            bm2 = bmesh.new()
            bm1.from_mesh(obj.data)
            bm2.from_mesh(other_obj.data)            

            # Make sure they are in the same world space
            bm1.transform(obj.matrix_world)
            bm2.transform(other_obj.matrix_world) 

            obj_BVHtree = BVHTree.FromBMesh(bm1)
            other_obj_BVHtree = BVHTree.FromBMesh(bm2)           

            # Find any overlap between the objects
            overlap = obj_BVHtree.overlap(other_obj_BVHtree)
            # If any intersecting parts are found save object to array so it can be deleted 
            if overlap != []:
                return True
            else:
                return False
   

#------------------------------------------------------------------
#           Main Function
#------------------------------------------------------------------


if __name__ == "__main__":
    reset_scene() 
    
    generate_creature(SEED)          