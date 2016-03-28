#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import codecs
import collections
import random
from sklearn import svm, feature_extraction
import recognize
import pickle
import time
import matplotlib.pyplot as plt

class Recognizer():
    def __init__(self):
        self.start_time = time.time()
        self.demo_on = False
        self.evaluation_mode = False
        self.return_average = False
        self.evaluation_frames = []
        self.node_name = "kille_recognizer"

        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow("RGB Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("RGB Image", 25, 75)

        # And one for the depth image
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)
        self.known_objects = []
        self.sift = cv2.SIFT()

        self.taken_actions = []

        self.current_depth = None
        self.current_stage = None
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("DMsubscriber", String)
        self.rate = rospy.Rate(10)  # 10hz
        self.last_recognized = None
        self.linclassifier = svm.LinearSVC()
        self.learned_relations = []
        self.region_of_interest = (1, 1000)
        self.recognitions = []
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("DMpublisher", String, self.receive, queue_size=1)

        try:
            self.learned_relations = pickle.load(open("saved_locations", "rb"))
            self.train_locations() # learned_relations is the list of relations in memory before training the classifier
        except:
            print "failed loading saved relations"

        try:
            # the features are not serializable, hence this work-around to load them from a file
            pickled_objects = pickle.load(open("saved_objects", "rb"))
            self.known_objects = []
            for obj in pickled_objects:
                kps = []
                for point in obj[1]:
                    temp_kp = cv2.KeyPoint(x=point[0][0], y=point[0][1], _size=point[1], _angle=point[2],
                                           _response=point[3], _octave=point[4], _class_id=point[5])
                    kps.append(temp_kp)
                self.known_objects.append((obj[0], kps, obj[2]))
        except:
            print "couldn't load learned objects file"
        rospy.loginfo("Waiting for image topics...")


    # this function is called for every rgb frame
    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        depth_frame = self.current_depth
        try:
            frame = np.array(self.bridge.imgmsg_to_cv2(ros_image, "bgr8"), dtype=np.uint8)
        except CvBridgeError, e:
            print e
        display_image = self.process_image(frame, depth_frame)
        cv2.imshow("RGB Image", display_image)
        cv.WaitKey(5) # HighGui (of which imshow() is a function of) needs waitkey so it has time to process its event loop

        # see what the next thing to do is, e.g. if a(n external) order has come in
        # this is inefficient and creates a small lag between depth frame processing and rgb frame processing
        # however, I have not found a reliable way to circumvent this, due to the necessity to catch the rgb and d frames
        # through the 'image_callback()' function.
        if self.current_stage is not None:
            self.taken_actions.append((self.current_stage, time.time() - self.start_time))
            method = self.current_stage[0]
            if method == self.send_scores:
                if self.evaluation_mode:
                    if len(self.evaluation_frames) == 10:
                        arguments = [display_image] + [depth_frame] + self.current_stage[1:]
                        method(*arguments)
                        self.current_stage = None
                        self.evaluation_frames = []
                    else:
                        self.evaluation_frames.append((display_image, depth_frame))
                else:
                    arguments = [display_image] + [depth_frame] + self.current_stage[1:]
                    method(*arguments)
                    self.current_stage = None
            else:
                arguments = [display_image] + self.current_stage[1:]
                method(*arguments)
                self.current_stage = None

    # this one is called for every depth-frame (which is in gray-scale)
    def depth_callback(self, ros_image):
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "16UC1")
        except CvBridgeError, e:
            print e

        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array = np.roll(depth_array, -15)
        self.current_depth = np.copy(depth_array)

        max_depth = self.region_of_interest[1]
        depth_array[depth_array < self.region_of_interest[0]] = max_depth
        depth_array[depth_array > self.region_of_interest[1]] = max_depth
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array)

        # Display the result
        cv2.imshow("Depth Image", depth_display_image)

    def process_depth_image(self, frame):
        return frame
        # return self.good_Features(frame)

    def process_image(self, frame, depth_frame):
        # set every pixel that is outside of the RoI to white (255,255,255).
        frame[np.tile(depth_frame > self.region_of_interest[1], (1, 1, 3))] = 255
        frame[np.tile(depth_frame < self.region_of_interest[0], (1, 1, 3))] = 255

        # show SIFT features
        if self.demo_on:
            kp, des = self.sift.detectAndCompute(frame, None)
            #frame = cv2.drawKeypoints(frame, kp, color=(0, 255, 0), flags=0)
            frame = cv2.drawKeypoints(frame,kp,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return frame

    def learn_new_object(self, frame, name):
        kp, des = self.sift.detectAndCompute(frame, None)
        self.known_objects.append((name, kp, des))
        self.send("learned:%s" % name)

    def learn_object_location(self, frame, lst):
        print "learning relation"
        name, name2, location = lst
        recognized = recognize.recognize_object(frame, self.current_depth, self.known_objects)[2]
        if recognized:
            recognized = self.get_unique_recognized(recognized)
            first_loc = None
            second_loc = None
            for rec_item in recognized:
                if rec_item[1] == name:
                    first_loc = rec_item[2]
                elif rec_item[1] == name2:
                    second_loc = rec_item[2]
            if first_loc and second_loc:
                self.learn_relation(location, first_loc, second_loc)
            else:
                self.send("failed to recognize objects")
        else:
            self.send("failed to recognize objects")

    @staticmethod
    def get_unique_recognized(self, recognized):
        unique = []
        unique_names_only = []
        for rec_object in recognized:
            if rec_object[1] not in unique_names_only:
                unique_names_only.append(rec_object[1])
                unique.append(rec_object)
        return unique

    def learn_relation(self, location, coord1, coord2):
        print coord1
        print coord2
        relative_coord = {'x': (coord1[0] - coord2[0]), 'y': (coord1[1] - coord2[1]), 'z': coord1[2] - coord2[2]}
        print relative_coord
        self.learned_relations.append((relative_coord, location))
        self.send(self.train_locations())

    def train_locations(self):
        x1 = [x[0] for x in self.learned_relations]
        y1 = [x[1] for x in self.learned_relations]
        if len(set(y1)) > 1:
            x1e = feature_extraction.DictVectorizer().fit_transform(x1)
            self.linclassifier.fit(x1e, y1)
            return "relation:%s" % x[-1:]
        else:
            return "relation:%s" % x[-1:]

    def get_relation(self, coord1, coord2):
        if len(set([x[1] for x in self.learned_relations])) < 2:
            return "unknown"
        relative_coord = {'x': (coord1[0] - coord2[0]), 'y': (coord1[1] - coord2[1]), 'z': coord1[2] - coord2[2]}
        print relative_coord
        print len(self.learned_relations)

        return self.linclassifier.predict(feature_extraction.DictVectorizer().fit_transform([relative_coord]))[0]

    def reinforce_object(self, frame):
        kp, des = self.sift.detectAndCompute(frame, None)
        self.known_objects.append((self.last_recognized[0], kp, des))
        self.send("reinforced:%s" % self.last_recognized[0])

    def unlearn_object(self, frame):
        name = self.known_objects[-1][0]
        self.known_objects = self.known_objects[:-1]
        self.send("unlearned:%s" % name)

    def unlearn_specific(self, frame, name):
        # diff_len is not used currently
        old_len = len(self.known_objects)
        self.known_objects = [x for x in self.known_objects if x[0] != name]
        diff_len = len(self.known_objects) - old_len
        self.send("removed:%s" % diff_len)

    def correct_last_object(self, frame, name):
        if len(self.known_objects) > 0:
            old_name = self.known_objects[-1][0]
            self.known_objects[-1] = (name, self.known_objects[-1][1], self.known_objects[-1][2])

            self.send("changed:%s:%s" % (old_name, name))
        else:
            self.send("I do not know any objects")

    # this method sends the scores back to ROS, in my implementation to ROSJava
    def send_scores(self, frame, depth_frame, plurality):
        if self.evaluation_mode:
            list_of_relations = []
            list_of_scores = []
            for frm, depth_frm in self.evaluation_frames:
                kp, des, scores = recognize.recognize_object(frm, depth_frm, self.known_objects)
                if scores:
                    relation = ""
                    if len(scores) > 1 and plurality == "plural":
                        if not any([np.isnan(x) for x in scores[0][2]]) and not any(
                                [np.isnan(y) for y in scores[1][2]]):
                            relation += self.get_relation(scores[0][2], scores[1][2])
                            list_of_relations.append(relation)
                    list_of_scores.append(scores)
            if not list_of_relations:
                self.send("detected none")
                return "detected none"
            relation = collections.Counter(list_of_relations).most_common(1)[0][0]
            combined_scores = self.combine_trials(list_of_scores)
            self.send(self.send_combined_entries(combined_scores, relation, plurality), plurality)
            if combined_scores:
                self.last_recognized = (combined_scores[0][1], kp, des)
            print [(x[0][1], x[0][0]) for x in list_of_scores]
            return combined_scores
        else:
            kp, des, scores = recognize.recognize_object(frame, depth_frame, self.known_objects)
            if scores:
                relation = ""
                combined_scores = self.combine_entries(scores)
                if len(combined_scores) > 1 and plurality == "plural":
                    relation += self.get_relation(combined_scores[0][2], combined_scores[1][2])
                if plurality == "plural":
                    self.send(self.send_combined_entries(combined_scores, relation, plurality))
                else:
                    self.send(self.send_combined_entries([combined_scores[0]], relation, plurality))
                self.last_recognized = (combined_scores[0][1], kp, des)
                return combined_scores
            else:
                self.send("detected none")
                return "detected none"

    # when different frames had to be tested, this method combines the results
    def combine_trials(self, lst):
        unique_items = []
        for scores in lst:
            for x in scores:
                if x[1] not in unique_items:
                    unique_items.append(x[1])
        scored_list = []
        scored_list2 = []
        for y in unique_items:
            total = []
            total2 = []
            total_coord = []
            lendes = []
            lendes2 = []
            for trial in lst:
                for x in trial:
                    if x[1] == y:
                        total.append(x[0])
                        total2.append(x[4])
                        lendes.append(x[5])
                        lendes2.append(x[6])
                        total_coord.append(x[2])
            average_coord = (int(sum([z[0] for z in total_coord]) / len(total_coord)),
                             int(sum([z[1] for z in total_coord]) / len(total_coord)),
                             int(sum([z[2] for z in total_coord]) / len(total_coord)))
            if self.return_average:
                scored_list.append(((sum(total) / len(total)), y, average_coord))
            else:
                scored_list.append((max(total), y, average_coord))
                scored_list2.append((y, max(total2), lendes[total2.index(max(total2))], lendes2[total2.index(max(total2))], average_coord))
        to_print = ""
        to_location = []
        for i in sorted(scored_list2, key=lambda x: x[1], reverse=True):
            print "%s %s" % (i[0],i[1])
            to_location.append((i[0],i[4]))
            to_print += ",%s %s %s" % (i[1],i[2],i[3])
        #print to_print
        print "%s" % [x[0] for x in to_location]
        print "%s, %s, %s, %s"% (to_location[0][0], to_location[1][0], to_location[0][1], to_location[1][1])
        print "%s, %s, %s" % (to_location[0][0], to_location[1][0], self.get_relation(to_location[0][1],to_location[1][1]))
        return sorted(scored_list2)
        #return sorted(scored_list, reverse=True)

    def combine_entries(self, lst):
        unique_items = []
        for x in lst:
            if x[1] not in unique_items:
                unique_items.append(x[1])
        scored_list = []
        for y in unique_items:
            total = []
            total_coord = []
            for x in lst:
                if x[1] == y:
                    total.append(x[0])
                    total_coord.append(x[2])
            average_coord = (int(sum([z[0] for z in total_coord]) / len(total_coord)),
                             int(sum([z[1] for z in total_coord]) / len(total_coord)),
                             int(sum([z[2] for z in total_coord]) / len(total_coord)))
            if self.return_average:
                scored_list.append(((sum(total) / len(total)), y, average_coord))
            else:
                scored_list.append((max(total), y, average_coord))
        return sorted(scored_list, reverse=True)

    @staticmethod
    def send_combined_entries(self, lst, relation, plurality):
        if plurality == "plural":
            to_send = "detectedloc:%s:%s:" % (len(lst), relation)
        else:
            to_send = "detected:%s:%s:" % (len(lst), relation)
        for entry in lst:
            to_send += '%s,%s:' % (entry[1], entry[0])
        return to_send

    # called at shutdown. Saves learned stuff to files and gracefully exits.
    def cleanup(self):
        pickle.dump(self.learned_relations, open("saved_locations", "wb"))
        pickleable_known_objects = []
        for object in self.known_objects:
            name, kp, des = object
            kp_list = []
            for point in kp:
                temp_kp = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
                kp_list.append(temp_kp)
            pickleable_known_objects.append((name, kp_list, des))
        pickle.dump(pickleable_known_objects, open("saved_objects", "wb"))
        if self.taken_actions:
            with open("taken_actions", "a") as write_taken_actions:
                for action in self.taken_actions:
                    write_taken_actions.write("%s : %s : %s\n" % (action[0][0].__name__, action[0][1:], action[1]))
        print "Shut down."
        cv2.destroyAllWindows()

    def send(self, data):
        extra = " : "
        if self.has_enough_knowledge():
            extra = self.has_enough_knowledge()
        elif len(set([x[1] for x in self.learned_relations])) < 2 and len(set([x[0] for x in self.known_objects] > 1)):
            extra = "relation:more"
        print "sending: %s~%s" % (data, extra)
        self.pub.publish(data + "~" + extra)
        self.rate.sleep()

    # tests wether the system considers itself to know enough about objects
    def has_enough_knowledge(self):
        objects_to_query = [x[0] for x in self.known_objects]
        if self.current_stage[0] == self.learn_new_object:
            objects_to_query = [y for y in objects_to_query if y != self.current_stage[1]]
        objects_freq = collections.Counter(objects_to_query)
        for x in objects_freq:
            if objects_freq[x] < 3:
                if random.randrange(2) == 1:
                    return 'object:%s' % x
        return " "

    def reinforce_relation(self, data):
        # TODO ? needs names of the objects ideally, which are not stored anywhere at the moment
        name = self.learned_relations[-1][1]

    def unlearn_relation(self, data):
        name = self.learned_relations[-1][1]
        self.learned_relations = self.learned_relations[:-1]
        self.train_locations()
        self.send("unlearned-last-relation:%s" % name)

    def unlearn_specific_relation(self, frame, name):
        # diff_len is not used currently
        old_len = len(self.learned_relations)
        self.known_objects = [x for x in self.learned_relations if x[0] != name]
        self.train_locations()
        diff_len = len(self.known_objects) - old_len
        self.send("removed-relation:%s" % diff_len)

    # handles input from rosjava (or substituting dialogue interface
    def receive(self, msg):
        data = codecs.decode(msg.data, 'utf-8').split(":")
        print "received:", data

        if data[0] == "learn":
            print 'trying to learn ' + data[1]
            self.current_stage = [self.learn_new_object, data[1]]
        elif data[0] == "what is this?":
            self.current_stage = [self.send_scores, "singular"]
        elif data[0] == "what are these?":
            self.current_stage = [self.send_scores, "plural"]
        elif data[0] == "last-object":
            self.current_stage = [self.reinforce_object]
        elif data[0] == "unlearn-last-object":
            self.current_stage = [self.unlearn_object]
        elif data[0] == "unlearn-object":
            self.current_stage = [self.unlearn_specific, data[1]]
        elif data[0] == "last-relation":
            self.current_stage = [self.reinforce_relation]
        elif data[0] == "unlearn-last-relation":
            self.current_stage = [self.unlearn_relation]
        elif data[0] == "unlearn-relation":
            self.current_stage = [self.unlearn_specific_relation, data[1]]
        elif data[0] == "relearn":
            self.current_stage = [self.correct_last_object, data[1]]
        elif data[0] == "relation":
            self.current_stage = [self.learn_object_location, (data[1], data[2], data[3])]
        else:
            print u"I don't understand", data


def main(args):
    print("kille och rosdial")
    try:
        Recognizer()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shut down."
        cv.DestroyAllWindows()


def reduce_method(m):
    return (getattr, (m.__self__, m.__func__.__name__))


if __name__ == '__main__':
    main(sys.argv)
