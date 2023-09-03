from googletrans import Translator
import openai
import jieba.posseg as pseg
import speech_recognition as sr


def audio_recognize():
    # 创建一个Recognizer对象
    r = sr.Recognizer()

    # 使用麦克风录音
    with sr.Microphone() as source:
        print("请告诉我你的需求：")
        audio = r.listen(source)

    # 将语音转换为文本
    try:
        text = r.recognize_google(audio, language='zh-CN')
        print(text)
        return text
    except sr.UnknownValueError as e:
        print(str(e))
        return 'error'
    except sr.RequestError as e:
        print(str(e))
        return 'error'


def gpt_stream(request):
    request += '请根据以上信息，逐个列出大家喜欢的物品。请注意：只要给出对应的物品就行，不需要分成每个人，也不要提及大家讨厌的物品。'
    messages = [{'role': 'user', 'content': request}]

    response = openai.ChatCompletion.create(
        model='gpt-3.5-turbo',
        messages=messages,
        stream=True,
    )

    completion = {'role': '', 'content': ''}

    for event in response:
        if event['choices'][0]['finish_reason'] == 'stop':
            gpt_message = completion['content']
            print(gpt_message)
            return gpt_message

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
    parts = gpt_message.split("：")

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
