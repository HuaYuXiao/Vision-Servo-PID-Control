import nltk
from nltk.tokenize import word_tokenize

def extract_items(sentence):
    # 分词
    tokens = word_tokenize(sentence)

    # 提取关键词
    keywords = []
    for token in tokens:
        if token.lower() in ['need', 'want', 'like']:
            # 找到关键词后的物品名
            idx = tokens.index(token)
            keywords.extend(tokens[idx+1:])

    # 去除误导项
    misleading_words = ['hate', 'dislike']
    items = [item for item in keywords if item.lower() not in misleading_words]

    return items

# 示例句子
sentence = "we need an apple. we hate banana"

items = extract_items(sentence)
print("target:", items)
