from twilio.rest import Client

# Thông tin từ Twilio
account_sid = 'AC0bcc2d0c60c6be5b655f6793a522c0ff'  # Thay bằng Account SID của bạn
auth_token = 'd5a5efde68edc75dd2ebe6ce0c49dd89'    # Thay bằng Auth Token của bạn
twilio_phone_number = '+17278004258'  # Số điện thoại Twilio của bạn
recipient_phone_number = '0963286361'  # Số điện thoại nhận tin nhắn

# Khởi tạo Twilio Client
client = Client(account_sid, auth_token)

# Hàm gửi tin nhắn
def send_sms_alert():
    try:
        message = client.messages.create(
            body="Cảnh báo! Đây là tin nhắn thử nghiệm từ Raspberry Pi.",
            from_=twilio_phone_number,
            to=recipient_phone_number
        )
        print(f"Tin nhắn đã được gửi: {message.sid}")
    except Exception as e:
        print(f"Đã xảy ra lỗi: {e}")

# Thực hiện gửi tin nhắn
send_sms_alert()

from twilio.rest import Client

account_sid = 'AC0bcc2d0c60c6be5b655f6793a522c0ff'
auth_token = '9ddc84eca43e2b3324f872a490a0d66a'
client = Client(account_sid, auth_token)

message = client.messages.create(
  from_='+17278004258',
  to='+840963286361'
)

print(message.sid)
