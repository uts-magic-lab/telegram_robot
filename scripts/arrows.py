#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Basic example for a bot that uses inline keyboards.
# This program is dedicated to the public domain under the CC0 license.

import rospy
import logging
from telegram import KeyboardButton, ReplyKeyboardMarkup
from telegram.ext import Updater, CommandHandler, CallbackQueryHandler

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)


def createKeyboard(strKeys):
    keyboard = []
    for row in strKeys:
        newRow = map(KeyboardButton, row)
        keyboard.append(newRow)
    return keyboard

def start(bot, update):
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

    keyboard = createKeyboard(headKeyboard)

    reply_markup = ReplyKeyboardMarkup(keyboard)

    update.message.reply_text('Tap the arrows to move the PR2:', reply_markup=reply_markup)


def help(bot, update):
    update.message.reply_text("Use /start to control this bot.")


def error(bot, update, error):
    logging.warning('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':

    # Fetch the token from token.yaml
    rospy.init_node('telegram_example')
    token = rospy.get_param('/telegram/token', None)

    # Create the Updater and pass it your bot's token.
    updater = Updater(token)

    # Add command and error handlers
    updater.dispatcher.add_handler(CommandHandler('start', start))
    updater.dispatcher.add_handler(CommandHandler('help', help))
    updater.dispatcher.add_error_handler(error)

    # Start the Bot
    updater.start_polling()

    # Run the bot until the user presses Ctrl-C or the process receives SIGINT,
    # SIGTERM or SIGABRT
    updater.idle()