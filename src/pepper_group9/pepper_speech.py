#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_group9.srv import Say, SayResponse

# Class AnimatedSay, extends NaoqiNode
# Phrase composition of what Pepper have to say after detection.
# Class Attributes:
# speech - ALTextToSpeech proxy
# s - variable of 'animated_say' Service
class AnimatedSay(NaoqiNode):

    # Initializes the NaoQi node with name "animated_speech"
    def __init__(self):
        NaoqiNode.__init__(self,'animated_speech')
        self.connectNaoQi()
        pass
    
    # Composes the phrase that pepper says after object detection.
    # 
    # param: data - message of type DetectorMessage.msg
    # returns: SayResponse - Bool, response of Say.srv
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
        if data.position == "center":
            phrase += " in front of me"
        else:
            phrase += " on the " + data.position
        self.speech.say(phrase, "English")
        rospy.loginfo("END: %s", phrase)
        return SayResponse(True)
    
    # ALTextToSpeech module allows the robot to speak.
    # It sends commands to a text-to-speech engine, and authorizes also 
    # voice customization. The result of the synthesis is sent to the 
    # robot’s loudspeakers.
    #
    # Creation of "animated_say" Service
    def connectNaoQi(self):
        self.speech=self.get_proxy("ALTextToSpeech")
        self.s = rospy.Service('animated_say', Say, self.say)

if __name__=="__main__":
    say = AnimatedSay()
    rospy.loginfo("Animated Speech node successfully started")
    rospy.spin()
