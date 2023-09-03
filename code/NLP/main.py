import logging
import jieba


from function import *


if __name__ == '__main__':
    # 将jieba的日志级别设置为WARNING
    jieba.setLogLevel(logging.WARNING)

    openai.api_key = "sk-zXLNlyG8y74Tid7GMxytSsjJsZ6lC6sjN58Wt9aPzw8zaJRr"
    openai.api_base = "https://api.chatanywhere.com.cn"

    cnt_error = 0
    while cnt_error < 3:
        request = audio_recognize()

        # 检查返回的结果是否是 "error"
        if request == "error":
            cnt_error += 1
        else:
            # 重置错误计数器
            cnt_error = 0
            gpt_message = gpt_stream(request)
            items = get_items(gpt_message)
            break

    # 如果错误计数达到3次或者成功获取了信息，则使用input获取用户需求
    if cnt_error >= 3:
        request = input("请告诉我你的需求，输入在文本框中：")
        gpt_message = gpt_stream(request)
        items = get_items(gpt_message)
