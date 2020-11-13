#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_acquire_stream.srv import Say
from pepper_acquire_stream.srv import SayResponse

class AnimatedSay(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self,'animated_speech')
        self.connectNaoQi()
        pass
        
    def say(self,data):
        rospy.loginfo("START: %s %s", data.labels, data.position)
        phrase = "I saw "
        label_occ = {}
        if len(data.labels) == 0:
            phrase += "nothing "
        else:
            for label in data.labels:
                if label not in label_occ.keys():
                    label_occ[label] = 1
                else:
                    label_occ[label] += 1
            for label, occ in label_occ.items():
                if occ == 1:
                    phrase += "a " + label + ", "
                else:
                    phrase += str(occ) +" " + label + ", "
            phrase = phrase[:-2]
            if ',' in phrase:
                head, _sep, tail = phrase.rpartition(',')
                phrase = head + " and" + tail
        phrase += " on " + data.position
        #self.speech.say(data.message)
        rospy.loginfo("END: %s", phrase)
        return SayResponse(True)

    def connectNaoQi(self):
        #self.speech=self.get_proxy("ALAnimatedSpeech")
        self.s = rospy.Service('animated_say', Say, self.say)

if __name__=="__main__":
    pub = AnimatedSay()
    rospy.spin()