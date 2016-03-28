import cv2
import numpy as np

def recognize_object(frame, depthframe, known_objects):
    sift = cv2.SIFT()
    # extract sift features:
    kp, des = sift.detectAndCompute(frame, None)
    # setup FLANN parameters (Fast Library for Approximate Nearest Neighbors)
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    scores = []
    scores2 = []
    for name, kp2, des2 in known_objects:
        #print "%s %s" % (name, len(des2))
        matches = flann.knnMatch(des, des2, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)
        if len(good) > .00*len(des):
            if len(good) > len(des) or len(good) > len(des2):
                print "something went wrong? algorithm error?"
                print len(good)
                print len(des)
                print len(des2)
                print len(matches)
                print name
                print "end error"
            x_coord = np.mean([kp[x.queryIdx].pt[0] for x in good])
            y_coord = np.mean([kp[x.queryIdx].pt[1] for x in good])
            z_coord = np.mean([depthframe[int(kp[x.queryIdx].pt[1]), int(kp[x.queryIdx].pt[0])] for x in good])
            scores2.append((name, len(good), len(des), len(des2)))
            #scores.append((len(good)*1.0/len(des2), name, (x_coord,y_coord, z_coord), vectors_best_matches(good, kp, depthframe, 20), len(good), len(des), len(des2)))
            calc = (2*float((len(good)))/len(des2)*(float(len(good))/len(des))) / ((float(len(good))/len(des2))+(float(len(good)/len(des))))
            scores.append((calc, name, (x_coord,y_coord, z_coord), vectors_best_matches(good, kp, depthframe, 20), len(good), len(des), len(des2)))
    temp_print = ""
    for i in scores2:
        temp_print += "%s,%s,%s,%s:" % (i[0],i[1],i[2],i[3])
    #print temp_print
    #print calc
    return (kp, des, scores)
                
def vectors_best_matches(matches, kp, depthframe, n):
    matches = sorted(matches, key = lambda x:x.distance)
    vectors = np.array([])
    for match in matches:
        if vectors.size < n*3:
            x = int(kp[match.queryIdx].pt[0])
            y = int(kp[match.queryIdx].pt[1])
            z = depthframe[y,x]
            if z > 500 and z < 1000:
                if vectors.size > 0:
                    vectors = np.append(vectors, [[x,y,z]], axis=0)
                else:
                    vectors = np.array([[x,y,z]])
    return vectors

