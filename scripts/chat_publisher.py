#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Publish the received messages on a std_msgs/String
topic for other nodes to be able to do stuff with it.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
import logging
import rospy
from std_msgs.msg import String

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


class ChatPublisher(object):
    def __init__(self):
        token = rospy.get_param('/telegram/token', None)
        if token is None:
            rospy.logerr("No token found in /telegram/token param server.")
            exit(0)
        else:
            rospy.loginfo("Got telegram bot token from param server.")

        self.str_pub = rospy.Publisher("~telegram_chat", String, queue_size=1)
        rospy.loginfo("Publishing received messages in topic: " +
                      str(self.str_pub.resolved_name))

        # Create the EventHandler and pass it your bot's token.
        updater = Updater(token)

        # Get the dispatcher to register handlers
        dp = updater.dispatcher

        # on noncommand i.e message - echo the message on Telegram
        dp.add_handler(MessageHandler(Filters.text, self.pub_received))

        # log all errors
        dp.add_error_handler(self.error)

        # Start the Bot
        updater.start_polling()

    # Define a few command handlers
    def pub_received(self, bot, update):
        rospy.loginfo("Received: " + str(update))
        self.str_pub.publish(update.message.text)
        update.message.reply_text("✔")

    def error(self, bot, update, error):
        logger.warn('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_chat_pub')
    cp = ChatPublisher()
    rospy.spin()
