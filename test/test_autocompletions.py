#! /usr/bin/env python

import actionlib
from hmi_msgs.msg import QueryAction
from hmi import Client
import os

grammar = """
O[P] -> ORDER[P] | can i have a ORDER[P] | i would like ORDER[P] | can i get ORDER[P] | could i have ORDER[P] | may i get ORDER[P] | bring me ORDER[P]
ORDER[<A1>] -> ITEMS[A1] | DET ITEMS[A1]
ORDER[<A1, A2>] -> ITEMS[A1] ITEMS[A2] | DET ITEMS[A1] DET ITEMS[A2] | DET ITEMS[A1] and DET ITEMS[A2] | ITEMS[A1] and ITEMS[A2]
ORDER[<A1, A2, A3>] -> ITEMS[A1] ITEMS[A2] ITEMS[A3] | ITEMS[A1] ITEMS[A2] and ITEMS[A3] |DET ITEMS[A1] DET ITEMS[A2] DET ITEMS[A3] | DET ITEMS[A1] and DET ITEMS[A2] and DET ITEMS[A3] | DET ITEMS[A1] DET ITEMS[A2] and DET ITEMS[A3]
DET -> a | an
"""

candies = ["biscuit", "frosty_fruits", "snakes"]
drinks = ["beer", "chocolate_milk", "coke", "juice", "lemonade", "water"]
food = ["carrot", "cereals", "noodles", "onion", "vegemite"]
fruits = ["apple", "kiwi", "lemon", "orange", "pear"]
snacks = ["cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties"]


for item in candies + drinks + food + fruits + snacks:
    grammar += "\nITEMS['{}'] -> {}[B]".format(item, item.replace('_', ' '))

target = 'O'

def read_valid_file(p, arg):
    if not os.path.exists(arg):
        p.error("The file %s does not exist!" % arg)
    else:
        return open(arg, 'r').read()

if __name__ == "__main__":
    import rospy
    rospy.init_node('test_autocompletions')

    target = rospy.get_param('~target', target)
    grammar_file = rospy.get_param('~grammar_file', None)

    # Verify the specified grammar
    if grammar_file and target:
        with open(grammar_file) as gf:
            grammar = gf.read()

    sac = actionlib.SimpleActionClient('/hmi', QueryAction)
    sac.wait_for_server()
    client = Client(simple_action_client=sac)

    rospy.sleep(rospy.Duration(5))

    rospy.logwarn("Press enter to continue after you have started a conversation with the bot")
    raw_input("Press enter to continue after you have started a conversation with the bot")

    result = client.query("What is your order?",
                          grammar, target,
                          timeout=300)

    print(result)
