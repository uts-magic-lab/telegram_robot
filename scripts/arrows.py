#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Basic example for a bot that uses inline keyboards.
# This program is dedicated to the public domain under the CC0 license.

import rospy
from geometry_msgs.msg import Twist
import logging
from telegram import KeyboardButton, ReplyKeyboardMarkup
from telegram.ext import Updater, CommandHandler, CallbackQueryHandler, MessageHandler, Filters
import time

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)


class BaseKeyboard(object):
    def __init__(self):
        self.base_pub = rospy.Publisher("/base_controller/command", Twist,
                                        queue_size=1)
        token = rospy.get_param('/telegram/token', None)

        # Create the Updater and pass it your bot's token.
        updater = Updater(token)

        # Add command and error handlers
        updater.dispatcher.add_handler(CommandHandler('start', self.start))
        updater.dispatcher.add_handler(CommandHandler('help', self.help))
        updater.dispatcher.add_handler(MessageHandler(Filters.text, self.echo))
        updater.dispatcher.add_error_handler(self.error)

        # Start the Bot
        updater.start_polling()

    def incrementalGripperKeyboard(self):
        min = 0
        max = 1000.0
        num_incr = 5.0
        incr = max / num_incr
        slider_range = map(lambda x: str(x), map(lambda x: float(
            x) / 1000, range(0, int(max + incr), int(incr))))

        gripperKeyboard = [
            slider_range
        ]
        return gripperKeyboard

    def createKeyboard(self, strKeys):
        keyboard = []
        for row in strKeys:
            newRow = map(KeyboardButton, row)
            keyboard.append(newRow)
        return keyboard

    def start(self, bot, update):
        keyboardStr = [
            ["-", "↑", "+"],
            ["←", " ", "→"],
            ["↻", "↓", "↺"],
            ["STOP"],
        ]

        headKeyboard = [
            [" ", "↑", " "],
            ["↻", " ", "↺"],
            [" ", "↓", " "],
            ["Paparazzi"],
        ]

        gripperKeyboard = [
            ["Open Left", "Open Right"],
            ["Close Half Left", "Close Half Right"],
            ["Close Left", "Close Right"]
        ]

        keyboard = self.createKeyboard(keyboardStr)

        reply_markup = ReplyKeyboardMarkup(keyboard)

        update.message.reply_text(
            'Tap the arrows to move the PR2:', reply_markup=reply_markup)

    def echo(self, bot, update):
        #rospy.loginfo("Echoing: " + str(update))
        update.message.reply_text(update.message.text)
        # TODO: transform this in an edit of the last message.
        text = update.message.text
        t = Twist()
        adv = 0.2
        rot = 0.2
        if text == 'STOP':
            # Keep the message at 0.0 to stop
            pass
        elif text == u'↑':
            t.linear.x = adv
        elif text == u'↓':
            t.linear.x = -adv / 2.0
        elif text == u'←':
            t.linear.y = adv
        elif text == u'→':
            t.linear.y = -adv
        elif text == u'↻':
            t.angular.z = -rot
        elif text == u'↺':
            t.angular.z = rot
        else:
            print("Nothing Interesting")
        # TODO: add dealing with + -

        ini_t = time.time()
        while time.time() - ini_t < 1.0:
            self.base_pub.publish(t)

    def help(self, bot, update):
        update.message.reply_text("Use /start to control this bot.")

    def error(self, bot, update, error):
        logging.warning('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':

    # Fetch the token from token.yaml
    rospy.init_node('telegram_example')
    bk = BaseKeyboard()
    rospy.spin()
