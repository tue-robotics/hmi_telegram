# hmi_telegram

[![CI](https://github.com/tue-robotics/hmi_telegram/workflows/CI/badge.svg)](https://github.com/tue-robotics/hmi_telegram/actions)

Server for [HMI](https://github.com/tue-robotics/hmi) to iteratively suggestion options to be presented as a custom keyboard for Telegram, through [telegram_ros](https://github.com/tue-robotics/telegram_ros)

![Screenshot that builds the sentence 'I would like a ...'](https://github.com/tue-robotics/hmi_telegram/raw/master/screenshot.jpg "I would like a...")

## Running

There is an example launch file:

```bash
roslaunch hmi_telegram test_autocompletions.launch token:=5693...
```

Tokens for telegram can be obtained via BotFather on Telegram
