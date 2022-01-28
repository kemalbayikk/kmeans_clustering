# -*- coding: utf-8 -*-
from __future__ import division
import math
import random


class KMeansClusterClassifier:
    
    def __init__(self,X):
        self.X=X
        pass
        
    def fit(self,X):
        
        points = [
            self.setPoints(X,i) for i in range(len(X))
        ]
        return points
    
    def predict(self,points):
        best_clusters, elbow_errors = self.kmeans_iterative(points,100,8)

        for i, c in enumerate(best_clusters):
            for p in c.points:
                print (" Cluster: ", i, "\t Point :", p)
        return best_clusters, elbow_errors
    
    def setPoints(self,X,k):
        p = Point([X[k][i] for i in range(len(X[0]))])
        return p
    
    def kmeans_iterative(self,points, iteration_count, max_k):
        candidate_clusters = []
        best_cluster_array = []
        errors = []
        elbow_errors = []
        for i in range(1,max_k):
            for _ in range(iteration_count):
                clusters = self.kmeansAlg(points, i, iteration_count,  epoch = _)
                error = self.calculateError(clusters)
                candidate_clusters.append(clusters)
                errors.append(error)
            
            elbow_errors.append(self.elbowError(clusters))
            highest_error = max(errors)
            lowest_error = min(errors)
            ind_of_lowest_error = errors.index(lowest_error)
            best_clusters = candidate_clusters[ind_of_lowest_error]
            best_cluster_array.append(best_clusters)
        
            
        lowest_elbow_error = min(elbow_errors)
        ind_of_lowest_elbow_error = elbow_errors.index(lowest_elbow_error)
        best_clusters_last = best_cluster_array[ind_of_lowest_elbow_error-2]
        print(" K : ",ind_of_lowest_elbow_error - 1)
        return best_clusters_last, elbow_errors

    def kmeansAlg(self, points, k, total_epoch, epoch):
        print("epoch and k",epoch,k)
        initial_centroids = random.sample(points, k)
        clusters = [Cluster([p]) for p in initial_centroids]

        loopCounter = 0
        while True:

            lists = [[] for _ in clusters]
            clusterCount = len(clusters)
    

            loopCounter += 1

            for p in points:
               
                smallest_distance = self.euclidDistance(p, clusters[0].centroid)
                clusterIndex = 0
                for i in range(1, clusterCount):
                    distance = self.euclidDistance(p, clusters[i].centroid)

                    if distance < smallest_distance:
                        smallest_distance = distance
                        clusterIndex = i

                lists[clusterIndex].append(p)
    

            biggest_shift = 0.0

            for i in range(clusterCount):
                shift = clusters[i].update(lists[i])
                biggest_shift = max(biggest_shift, shift)
    
            clusters = [c for c in clusters if len(c.points) != 0]
            epoch+=1
            if total_epoch == epoch:
                break
        return clusters
    
    def euclidDistance(self,a, b):
    
        accumulatedDifference = 0.0
        for i in range(a.n):
            squareDifference = pow((a.coords[i]-b.coords[i]), 2)
            accumulatedDifference += squareDifference
    
        return accumulatedDifference
    
    def calculateError(self,clusters):

        accumulatedDistances = 0
        num_points = 0
        for cluster in clusters:
            num_points += len(cluster.points)
            accumulatedDistances += cluster.getTotalDistance()
    
        error = accumulatedDistances / num_points
        return error
    
    def elbowError(self,clusters):

        accumulatedDistances = 0
        num_points = 0
        for cluster in clusters:
            num_points += len(cluster.points)
            accumulatedDistances += cluster.getTotalDistance() * cluster.getTotalDistance()    
        return accumulatedDistances

class Point(object):

    def __init__(self, coords):

        self.coords = coords
        self.n = len(coords)

    def __repr__(self):
        return str(self.coords)
    
class Cluster(object):

    def __init__(self, points):

      
        self.points = points
        self.n = points[0].n

        self.centroid = self.calculateCentroid()

    def __repr__(self):
        return str(self.points)

    def update(self, points):
        old_centroid = self.centroid
        self.points = points
        if len(self.points) == 0:
            return 0

        self.centroid = self.calculateCentroid()
        shift = self.euclidDistance(old_centroid, self.centroid)
        return shift

    def calculateCentroid(self):
        numPoints = len(self.points)
        coords = [p.coords for p in self.points]
        unzipped = zip(*coords)
        centroid_coords = [math.fsum(dList)/numPoints for dList in unzipped]

        return Point(centroid_coords)

    def getTotalDistance(self):
        sumOfDistances = 0.0
        for p in self.points:
            sumOfDistances += self.euclidDistance(p, self.centroid)

        return sumOfDistances
    def euclidDistance(self,a, b):
    
        accumulatedDifference = 0.0
        for i in range(a.n):
            squareDifference = pow((a.coords[i]-b.coords[i]), 2)
            accumulatedDifference += squareDifference
    
        return accumulatedDifference

    
