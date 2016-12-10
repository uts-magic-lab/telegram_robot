#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Publish the received messages on a std_msgs/String
topic for other nodes to be able to do stuff with it.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, CallbackQueryHandler
from telegram import InlineKeyboardButton, InlineKeyboardMarkup
import logging
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)

logger = logging.getLogger(__name__)


class ImageReply(object):
    def __init__(self):
        token = rospy.get_param('/telegram/token', None)
        if token is None:
            rospy.logerr("No token found in /telegram/token param server.")
            exit(0)
        else:
            rospy.loginfo("Got telegram bot token from param server.")

        # Set CvBridge
        self.bridge = CvBridge()

        # Create the EventHandler and pass it your bot's token.
        updater = Updater(token)

        # Get the dispatcher to register handlers
        dp = updater.dispatcher

        # on message...
        dp.add_handler(MessageHandler(Filters.text, self.pub_received))
        dp.add_handler(CallbackQueryHandler(self.button))

        # log all errors
        dp.add_error_handler(self.error)

        # Start the Bot
        updater.start_polling()

    def get_image(self, image_topic=None):
        rospy.loginfo("Getting image...")
        if image_topic is None:
            image_topic = "/wide_stereo/left/image_raw"
        image_msg = rospy.wait_for_message(image_topic, Image)
        rospy.loginfo("Got image!")

        cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_file_path = "/tmp/telegram_last_image.jpg"
        cv2.imwrite(img_file_path, cv2_img)
        rospy.loginfo("Saved to: " + img_file_path)
        return img_file_path

    # For some reason the image is grayscale...
    def get_image_compressed(self):
        rospy.loginfo("Getting image...")
        image_msg = rospy.wait_for_message(
            "/wide_stereo/left/image_raw/compressed",
            CompressedImage)
        rospy.loginfo("Got image!")

        # Image to numpy array
        np_arr = np.fromstring(image_msg.data, np.uint8)
        # Decode to cv2 image and store
        cv2_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_file_path = "/tmp/telegram_last_image.png"
        cv2.imwrite(img_file_path, cv2_img)
        rospy.loginfo("Saved to: " + img_file_path)
        return img_file_path

    # Define a few command handlers
    def pub_received(self, bot, update):
        rospy.loginfo("Received: " + str(update))
        valid_necessary_words = ['what do you see',
                                 'picture',
                                 'camera']
        found_word = False
        for v in valid_necessary_words:
            if v in update.message.text.lower():
                self.do_image_stuff(update)
                found_word = True
                break
        # In case the user said anything else
        if not found_word:
            update.message.reply_text("Try any of: " +
                                      str(valid_necessary_words))

    def do_image_stuff(self, update):
        # Get topics of type Image
        topics_and_types = rospy.get_published_topics()
        image_topics = []
        for top, typ in topics_and_types:
            if typ == 'sensor_msgs/Image':
                image_topics.append(top)

        keyboard = []
        for topicname in image_topics:
            keyboard.append([InlineKeyboardButton(
                topicname, callback_data=topicname)])

        reply_markup = InlineKeyboardMarkup(keyboard)

        update.message.reply_text('Choose image topic:',
                                  reply_markup=reply_markup)

    def button(self, bot, update):
        query = update.callback_query

        bot.editMessageText(text="Capturing image of topic: %s" % query.data,
                            chat_id=query.message.chat_id,
                            message_id=query.message.message_id)

        img_file_path = self.get_image(image_topic=query.data)
        bot.editMessageText(text="Uploading captured image of topic: %s" % query.data,
                            chat_id=query.message.chat_id,
                            message_id=query.message.message_id)
        bot.send_photo(chat_id=query.message.chat_id,
                       photo=open(img_file_path, 'rb'),
                       caption="This is what I see on topic " +
                       query.data)
        # update.message.reply_photo(photo=open(img_file_path, 'rb'),
        #                            caption="This is what I see on topic " +
        #                            query.data)

    def error(self, bot, update, error):
        logger.warn('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_chat_pub')
    cp = ImageReply()
    rospy.spin()
