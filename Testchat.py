import asyncio
from telegram import Bot

# Thay 'YOUR_TOKEN' bằng token của bot mà bạn đã nhận được từ BotFather
TOKEN = '7377344059:AAGfmth-0UmRiyGicVb0BcyY_uEihWYTQgk'
# Thay 'YOUR_CHAT_ID' bằng chat ID của người nhận
CHAT_ID = '6888591504'

bot = Bot(token=TOKEN)

async def send_message_periodically():
    while True:
        await bot.send_message(chat_id=CHAT_ID, text="Nhà bạn có cháy!!!")  # Sử dụng await ở đây
        await asyncio.sleep(2)  # Đợi 2 giây trước khi gửi lại

async def main():
    await send_message_periodically()

if __name__ == '__main__':
    asyncio.run(main())
