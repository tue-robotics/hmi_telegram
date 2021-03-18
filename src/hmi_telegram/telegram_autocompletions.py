import random

import rospy
from hmi import AbstractHMIServer, HMIResult
from hmi.common import random_sentence, result_from_ros, parse_sentence
from grammar_parser.cfgparser import CFGParser
from threading import Event

from std_msgs.msg import String
from telegram_ros.msg import Options

wait_time = 5


class TelegramAutocompletions(AbstractHMIServer):
    def __init__(self, *args, **kwargs):
        super(TelegramAutocompletions, self).__init__(*args, **kwargs)

        self._messsage_sub = rospy.Subscriber('message_to_ros', String, self._message_cb)
        self._options_pub = rospy.Publisher('options_from_ros', Options, queue_size=1)
        self._message_pub = rospy.Publisher('message_from_ros', String, queue_size=1)

        self._grammar_parser = None  # type: CFGParser
        self._target = ''
        self._intermediate_answer = []
        self._answer_ready = Event()

        self._timeout = 300

    @property
    def intermediate_sentence(self):
        return ' '.join(self._intermediate_answer)

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        if is_preempt_requested and self._target:
            rospy.logwarn("Preempt requested while a query was being answered")
            self._answer_ready.set()
        else:
            rospy.logwarn("Preempt requested while NO query was being answered")

        rospy.loginfo('Sending to telegram')
        self._intermediate_answer = []
        self._target = target
        self._grammar_parser = CFGParser.fromstring(grammar)
        self._grammar_parser.verify()

        self._options_pub.publish(Options(question=description,
                                          options=sorted(set(
                                              self._grammar_parser.next_word(self._target,self._intermediate_answer)))))

        start = rospy.Time.now()
        while not rospy.is_shutdown() and \
                        rospy.Time.now() < start + rospy.Duration(self._timeout) and \
                not self._answer_ready.isSet():
            self._answer_ready.wait(1)

        if self._answer_ready.is_set():
            rospy.loginfo("Answer is ready")
            self._answer_ready.clear()

            try:
                sentence = self.intermediate_sentence
                rospy.loginfo("Gathered sentence: %s", sentence)
                if sentence:
                    semantics = parse_sentence(sentence, grammar, target)
                    rospy.loginfo("Parsed semantics: %s", semantics)
                    self._message_pub.publish("Completed sentence: '{}'. Thnx!".format(sentence))

                    self._intermediate_answer = []
                    self._target = ''
                    return HMIResult(sentence, semantics)
            except Exception:
                raise
        else:
            self._message_pub.publish("Sorry, that took too long")
            rospy.loginfo("Telegram took to look to complete the query")

    def _message_cb(self, msg):
        # type: (String) -> None
        rospy.loginfo("Got another bit: {}".format(msg.data))

        if msg.data == '.' or msg.data.endswith('.'):
            rospy.loginfo("Done: '{}' signifies end of sentence".format(msg.data))
            self._answer_ready.set()
        else:
            self._intermediate_answer += [msg.data]
            rospy.loginfo("Intermediate sentence is '{}'".format(self.intermediate_sentence))

            # Below, the sentence is split right after we join it.
            # This allows the user to manuallly type (not pick from button) several words at once
            completions = sorted(set(
                self._grammar_parser.next_word(self._target, self.intermediate_sentence.split(' '))))

            if completions:
                rospy.loginfo("There are {} completions available, proposing..".format(len(completions)))
                self._options_pub.publish(Options(question=self.intermediate_sentence + '...', options=completions))
            else:
                rospy.loginfo("Done: there are no more completions available after '{}'".format(self.intermediate_sentence))
                self._answer_ready.set()
