#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Basic example for a bot that uses inline keyboards.
# This program is dedicated to the public domain under the CC0 license.
"""
Testing making a few buttons on a keyboard.

TODO: Make the robot say the sentences.

Author: Jordan Lewis <Jordan.Lewis at uts.edu.au>
"""
import rospy
import logging
from telegram import KeyboardButton, ReplyKeyboardMarkup
from telegram.ext import Updater, CommandHandler, CallbackQueryHandler

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

def start(bot, update):
    hiKey = KeyboardButton("Hi", callback_data="Bonjour")
    howRU = KeyboardButton("How are you?")
    bye = KeyboardButton("Bye")
    introduce = KeyboardButton("I'm Gutsy")

    keyboard = [
    [hiKey, howRU],
    [bye, introduce]
    ]

    reply_markup = ReplyKeyboardMarkup(keyboard)

    update.message.reply_text('Please choose:', reply_markup=reply_markup)


def button(bot, update):
    query = update.callback_query

    bot.editMessageText(text="Selected option: %s" % query.data,
                        chat_id=query.message.chat_id,
                        message_id=query.message.message_id)


def help(bot, update):
    update.message.reply_text("Use /start to test this bot.")


def error(bot, update, error):
    logging.warning('Update "%s" caused error "%s"' % (update, error))


rospy.init_node('telegram_example')
token = rospy.get_param('/telegram/token', None)

# Create the Updater and pass it your bot's token.
updater = Updater(token)

updater.dispatcher.add_handler(CommandHandler('start', start))
updater.dispatcher.add_handler(CallbackQueryHandler(button))
updater.dispatcher.add_handler(CommandHandler('help', help))
updater.dispatcher.add_error_handler(error)

# Start the Bot
updater.start_polling()

# Run the bot until the user presses Ctrl-C or the process receives SIGINT,
# SIGTERM or SIGABRT
updater.idle()