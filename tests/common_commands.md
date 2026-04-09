# Common Commands for Testing

## How to test custom API commands for Chassis (base) in the command line with Python REPL
```
# start the REPL
python3 -m asyncio

# imports and bot initialization
from hex_device.chassis_client import ChassisClient; bot = ChassisClient("ws://192.168.1.230:8439"); await bot.connect(); await bot.initialize()

# velocity commands (x, y, theta)
bot.set_velocity(0.2, 0.0, 0.0)
bot.brake()
bot.stop()
bot.release_brake()

# de-initialize bot
await bot.deinitialize(); await bot.close()
```
