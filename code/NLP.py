from googletrans import Translator
import openai
import jieba
import jieba.posseg as pseg
import logging


def gpt_stream(request):
    messages = [{'role': 'user', 'content': request}]

    response = openai.ChatCompletion.create(
        model='gpt-3.5-turbo',
        messages=messages,
        stream=True,
    )

    completion = {'role': '', 'content': ''}

    for event in response:
        if event['choices'][0]['finish_reason'] == 'stop':
            return completion['content']

        for delta_k, delta_v in event['choices'][0]['delta'].items():
            completion[delta_k] += delta_v


def translate(text):
    # 创建翻译器对象
    translator = Translator()
    # 翻译中文文本为英文
    translated_text = translator.translate(text, src='zh-CN', dest='en').text

    return translated_text


def get_items(gpt_message):
    # 使用冒号分割句子，并取分割后的第二部分（冒号后面的内容）
    parts = gpt_message.split("：")  # 这里使用全角冒号，如果是半角冒号需要改为":"

    # 判断是否成功分割，然后获取第二部分
    if len(parts) > 1:
        result = parts[1]
    else:
        # 如果没有冒号，保留整个句子
        result = gpt_message

    # 使用jieba分词，并标注词性
    words = pseg.cut(result)

    # 提取名词
    nouns = []
    for word, flag in words:
        # 名词词性以'n'开头
        if flag.startswith('n'):
            word = translate(word)
            nouns.append(word)

    return nouns


if __name__ == '__main__':
    # 将jieba的日志级别设置为WARNING
    jieba.setLogLevel(logging.WARNING)

    openai.api_key = "sk-zXLNlyG8y74Tid7GMxytSsjJsZ6lC6sjN58Wt9aPzw8zaJRr"
    openai.api_base = "https://api.chatanywhere.com.cn"

    request = input("请告诉我你的需求：")
     # 我喜欢冰箱和苹果，我讨厌山和香蕉，他超爱梨和鸭子。

    request += '请根据以上信息，逐个列出大家喜欢的物品。请注意：只要给出对应的物品就行，不需要分成每个人，也不要提及大家讨厌的物品。'

    gpt_message = gpt_stream(request)
    print(gpt_message)

    items = get_items(gpt_message)
    # 打印提取到的名词
    print(items)

