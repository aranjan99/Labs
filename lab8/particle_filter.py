from grid import *
from particle import Particle
from utils import *
from setting import *

import numpy as np

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    #add gaussian noise to expected particle cordinates after applying odometry to move
    for i in particles:
        motion_particles.append(Particle(
        add_gaussian_noise(i.x + rotate_point(odom[0], odom[1], i.h)[0], 0.02), 
        add_gaussian_noise(i.y + rotate_point(odom[0], odom[1], i.h)[1], 0.02),
        add_gaussian_noise(i.h + odom[2], 2)))
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    finalParticleList, finalWeightsList, measurementUpdate, particleScoreMap = [], [], [], []
    
    if len(measured_marker_list) > 0:        
        for particle in particles:            
            #ignore if particle beyond grid length
            if particle.x < 0 or particle.x > grid.width or particle.y < 0 or particle.y > grid.height:
                particleScoreMap.append((0, particle))
                continue
            
            nearestMarkersMap = []
            markers = particle.read_markers(grid)
            countDifference = len(measured_marker_list)-len(markers)

            #find closest marker match in measured_marker_list for markers seen by particle p
            for measuredMarker in measured_marker_list:
                new_marker = add_marker_measurement_noise(measuredMarker, 0.5, 5)
                minimum_marker_particle = None    
                minimum_marker_distance = 1000000
                for marker in markers:
                    gridDistance = grid_distance(marker[0], marker[1], new_marker[0], new_marker[1])
                    minimum_marker_particle = marker if gridDistance <= minimum_marker_distance else minimum_marker_particle
                    minimum_marker_distance = gridDistance if gridDistance <= minimum_marker_distance else minimum_marker_distance
                if minimum_marker_particle != None:
                    markers.remove(minimum_marker_particle)
                    nearestMarkersMap.append((minimum_marker_particle, measuredMarker))

            weight = 1
            maxExpDis = [0]
            
            #get particle weight based on above matches
            for marker1, marker2 in nearestMarkersMap:
                nearestMarkerDistance = grid_distance(marker1[0], marker1[1], marker2[0], marker2[1])
                maxExpDis.append(nearestMarkerDistance * nearestMarkerDistance * 2)                
                nearestMarkerAngle = diff_heading_deg(marker1[2], marker2[2])
                weight = weight * np.exp(-(nearestMarkerDistance * nearestMarkerDistance * 2)-(nearestMarkerAngle * nearestMarkerAngle * 0.02))
            for i in range(int(countDifference)):
                weight = weight * np.exp(-max(maxExpDis)-40.5)
            particleScoreMap.append((weight, particle))

        #normalize weights of all particles
        normalizationFactor = 0
        particleScoreMap.sort(key=lambda x: x[0])
        particleScoreMap = particleScoreMap[100:]
        count = 0
        for weight, particle in particleScoreMap:
            count = count + 1 if weight == 0 else 0
            normalizationFactor = normalizationFactor + weight

        for weight, particle in particleScoreMap[count:]:
            finalWeightsList.append(weight/normalizationFactor)
            finalParticleList.append(Particle(particle.x, particle.y, particle.h))
        measurementUpdate = Particle.create_random(count + 100, grid)[:]
         
    #if no markers are detected simply assign equal weights to all particles
    if len(measured_marker_list) == 0:
        finalParticleList = particles
        totalCount = len(finalParticleList)
        for i in particles:
            finalWeightsList.append(1/totalCount)
        measurementUpdate = Particle.create_random(100, grid)[:]
    #resample
    for p in np.random.choice(finalParticleList, size=len(finalParticleList), replace = True, p=finalWeightsList):
        measurementUpdate.append(Particle(add_gaussian_noise(p.x, 0.02), add_gaussian_noise(p.y, 0.02), add_gaussian_noise(p.h, 2)))        
    return measurementUpdate