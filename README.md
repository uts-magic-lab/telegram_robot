# Telegram bot for your ROS enabled robot

Given that the realtime chatting app `Telegram` [enables the use of bots](https://core.telegram.org/bots) why not connect your robot via that service?

## Requisites
You need `python-telegram-bot`.

```bash
sudo pip install python-telegram-bot --upgrade
```

You need to get a TOKEN talking to the @botfather bot as [described here](https://core.telegram.org/bots#6-botfather). Remember you can use the [Telegram web client](https://web.telegram.org/), although you still need a phone with telegram to allow connecting to it.s

## Configure
Copy from [config/example_token.yaml](config/example_token.yaml) to `token.yaml` and add your token.

## Launch
You can test if your token works with:
```bash
roslaunch telegram_robot echobot_example.launch
```

The bot should echo what you write to it.

Launch the actual node:

```bash
roslaunch telegram_robot telegram_robot.launch
```

## What can you do


## How we got here
Telegram recommends the Python libraries:

* `Telepot`. Python framework for Telegram Bot API.
[https://github.com/nickoala/telepot](https://github.com/nickoala/telepot)

* `twx.botapi`. Library and client + documentation with Python examples.
[https://github.com/datamachine/twx.botapi](https://github.com/datamachine/twx.botapi)

And googling you also find:

* `python-telegram-bot`. Pure Python interface for the Telegram Bot API. It's compatible with Python versions 2.7, 3.3+ and PyPy. It also works with Google App Engine.
[https://github.com/python-telegram-bot/python-telegram-bot](https://github.com/python-telegram-bot/python-telegram-bot)

So we choose `python-telegram-bot` after checking which had the biggest and most active community of users and also had good documentation.