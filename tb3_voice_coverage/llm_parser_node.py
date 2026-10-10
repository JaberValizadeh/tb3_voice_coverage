#!/usr/bin/env python3
"""
llm_parser_node - free-form speech in, validated cleaning command out.

Sits in front of voice_coverage_node rather than inside it, exactly as
tb3_llm_control sits beside tb3_voice_control: the robot layer stays free of
any dependency on a model or a network.

    /voice_text  --(LLM)-->  /coverage_request  -->  voice_coverage_node
                             /voice_commands

Degrading rather than failing is the point. If the config file is missing, the
API is down, or the reply is unparseable, the raw utterance is forwarded to
/voice_commands untouched, where the keyword parser handles it. Worst case you
are back to the fixed vocabulary, never to a robot that ignores you.
"""

import json

import rospy
from std_msgs.msg import String

from tb3_voice_coverage.llm_parser import LLMParser, to_command_string


class LLMParserNode:
    def __init__(self):
        rospy.init_node('llm_parser')

        config_file = rospy.get_param('~config_file', '')
        model = rospy.get_param('~model', 'gpt-4o')
        in_topic = rospy.get_param('~input_topic', '/voice_text')
        timeout = rospy.get_param('~timeout', 20.0)

        self._req = rospy.Publisher('/coverage_request', String, queue_size=10)
        self._cmd = rospy.Publisher('/voice_commands', String, queue_size=10)
        self._say = rospy.Publisher('/coverage_status', String,
                                    queue_size=10, latch=True)

        self._parser = None
        try:
            self._parser = LLMParser(config_file, model, timeout=timeout)
            rospy.loginfo('LLM understanding enabled (model %s)', model)
        except Exception as exc:                       # noqa: BLE001
            rospy.logwarn('LLM disabled (%s: %s) - falling back to the '
                          'keyword parser', type(exc).__name__, exc)

        rospy.Subscriber(in_topic, String, self._on_text)
        rospy.loginfo('Listening for free-form requests on %s', in_topic)

    def _on_text(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return
        rospy.loginfo('request: "%s"', text)

        if self._parser is None:
            self._fallback(text)
            return

        try:
            cmd = self._parser.interpret(text)
        except Exception as exc:                       # noqa: BLE001
            rospy.logwarn('LLM call failed (%s: %s) - using keyword parser',
                          type(exc).__name__, exc)
            self._fallback(text)
            return

        if cmd['reply']:
            self._say.publish(String(data=cmd['reply']))
            rospy.loginfo('reply: %s', cmd['reply'])

        if cmd['action'] == 'clarify':
            # Nothing actionable; the reply above already asked the question.
            return

        if cmd['action'] == 'clean':
            # Structured, so the order the person asked for survives.
            self._req.publish(String(data=json.dumps({'rooms': cmd['rooms']})))
            rospy.loginfo('-> clean %s', ', '.join(cmd['rooms']))
        else:
            self._cmd.publish(String(data=to_command_string(cmd)))
            rospy.loginfo('-> %s', cmd['action'])

    def _fallback(self, text):
        rospy.loginfo('-> keyword parser: "%s"', text)
        self._cmd.publish(String(data=text))


def main():
    LLMParserNode()
    rospy.spin()


if __name__ == '__main__':
    main()
