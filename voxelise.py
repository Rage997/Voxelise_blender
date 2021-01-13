import bpy
import mathutils
from mathutils import Vector
import os
import numpy as np
import matplotlib.pyplot as plt
import math

'''
References:
    https://matplotlib.org/3.1.0/gallery/mplot3d/voxels.html
    https://stackoverflow.com/questions/42611342/representing-voxels-with-matplotlib
'''


def random_outside_interval(interval):
  '''
  Generates random numbers outside a given interval
  '''
  int1 = np.random.uniform(high=interval[0])
  int2 = np.random.uniform(low=interval[1])

  return np.random.choice([int1, int2])



def approximate(arr):
    '''
    Ceil a number to the closest integer in absolute value
    i.e. 1.2 ~ 2
         4.7 ~ 5
         -3.2 ~ -4
    '''
    return np.sign(arr) * np.ceil(np.abs(arr))
        

# NOTE please, recompute origin, set it to the bounding box center
# and then use the mesgrid with the bounding box dimension domain
# i m testing with suzanne (the blender monkey) and its origin is already "good"
obj = bpy.context.active_object

#Cube works perfectly
# Cylinder has false positives near the sharp edges
# Suzanne has many false positives

class voxelGrid:
    '''A class representing a voxel grid. This class is capable of creating a voxel from a blender's object'''

    def __init__(self, size):
        #TODO the voxel grid domain should be the size of the bounding box
        #self.mg = global_matrix
        self.size = size
        #TODO replace this with a bytearray
        data = (size, size, size)
        self.voxels = np.zeros(data, dtype=bool)
#        self.voxels = []
        self.init = False
        
    def voxelise(self, obj):
        '''
        Computes the voxel grid of an object. It stores its global matrix and 
        uses the bounding box as the voxel grid domain
        '''
        
        self.mg = obj.matrix_world
        bbox = [Vector(v) for v in obj.bound_box]
        #Domain = [bottom_left, right_top]
        minimums = np.amin(bbox, axis=0)
        maximums = np.amax(bbox, axis=0)

        self.xs = np.linspace(minimums[0], maximums[0], self.size)
        self.ys = np.linspace(minimums[1], maximums[1], self.size)
        self.zs = np.linspace(minimums[2], maximums[2], self.size)
#        np.meshgrid(xs, ys, zs)
        # Check for all voxel grid point if it is inside or outside mesh
        for i in range(self.size):
            for j in range(self.size):
                for k in range(self.size):
                    x, y, z = self.xs[i], self.ys[j], self.zs[k]
                    point = Vector((x, y, z))
#                    print('Testing point:', point)
#                    if self.inside_or_out_dot_product(obj, point):
                    if self.inside_or_out_raycasting(obj, point):
#                        print('voxel created')
                        self.voxels[i, j, k] = True
                    else:
                        self.voxels[i, j, k] = False
   
        self.init = True
        
    def is_voxel_init(self):
        '''Tells you if the voxels have been computed or not'''
        if not self.init:
            raise Exception('The voxel grid has not been computed!')
                
                        
    def visualise(self):
        '''Visualise the voxels grid'''
        self.is_voxel_init()
                
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.voxels(self.voxels, edgecolor='k')
        plt.show()

    def inside_or_out_dot_product(self, obj, point, tollerance=0.05):
        '''The function tells you if a given point is inside or outside
            a given objects by employing the dot product between the given
            point and the closest point on the mesh. Finding the closest
            point on the mesh is an expensive operation. Moreover, due to
            roundoff approximations when the angle between the point and the
            normal of the closest point on the object is close to 90 degree
            the point is detected as inside even if it's not. This has been tweaked by
            introducing some tolerance into the algorithm
            
            params:
                obj: the blender object
                point (mathutils.Vector): a point in world space
        '''
        
        # Search for the closest point and the normal on this point
        # this operation is really expensive...
        _, point_on_mesh, normal, _ = obj.closest_point_on_mesh(point, distance=1.84467e+19)
        # compute the vector between the given point and the closest point on the mesh
        direction = point_on_mesh - point
        # recall that v.dot(w) = norm(v) norm(w) cos(theta) where theta is the angle
        # between v and w. Therefore if the point is inside the mesh cos(theta) is greater
        # than zero. 
        dot = direction.dot(normal)
        # Get the angle between the point and the normal on the closest point on the mesh
        return dot >= 0.0
    
    def inside_or_out_raycasting(self, obj, point, sample=20):
        '''The function tells you if a given point is inside or outside
            a given objects by casting a ray from the point towards a random point
            outside the objects' bounding box. A single ray is sufficient to determine
            wheter a point is inside or outisde an object because by definition (or sorta)
            all the objects are closed paths. If we had an open path, another approach (more expensive)
            would have been to compute the winding number
            
            params:
                obj: the blender object
                point (mathutils.Vector): a point in world space
        '''
        
        #sample a number of random points around the bounding box of the object
        bbox = [obj.matrix_world @ Vector(v) for v in obj.bound_box]
        min_corner, max_corner = min(bbox), max(bbox)
        tmp = 0
        for i in range(sample):
            rnd = [random_outside_interval([low, high]) * 100 for low, high in zip(min_corner, max_corner)]
            
            intersections = self.intersect_ray_object(obj, point, Vector(rnd))
            tmp += intersections % 2
             
        return round(tmp/sample)
    
    def intersect_ray_object(self, obj, origin: Vector, destination: Vector):
        '''
        Cast a ray y(t) = origin + t * (destination - origin) and returns the number of intersections
        with the geoemtry of an object
        '''    
        
        # Cast a ray from the origin toward destination and get the first hit with the geometry
        result, hit_location, normal, face_idx = obj.ray_cast(origin, destination)
        
        if not result: #the ray did not hit any geometry
                return False
        else: # the ray hitted geometry
             # Cast n more rays to count how many times the ray hits the geoemtry
             # the new ray starts from hitted location + a little step toward the destination
        
            max_intersections = 1000
            direction = destination - hit_location
            amount = 0.0001 / direction.length
            i = 1
            while i < max_intersections:
                #TODO put this into a single function
                # Interpolate point = point + alpha * direction
                origin = hit_location.lerp(direction, amount)
                # cast a new ray from new origin
                result, hit_location, normal, face_idx = obj.ray_cast(origin, destination)
                
                if not result:  
                    break #there are no more faces between the current ray origin and mesh
                i += 1
#            return i % 2
            return i #return the number of times a ray intersects an object        
    
    def save(self, fp):
        self.is_voxel_init()
        with open(fp, 'wb+') as f:
            np.save(arr=self.voxels, file=f)
        
#TODO compute the ray intersections in model space

vg = voxelGrid(20)
vg.voxelise(obj)
vg.visualise()
vg.save('/Users/rage/Desktop/test.np')