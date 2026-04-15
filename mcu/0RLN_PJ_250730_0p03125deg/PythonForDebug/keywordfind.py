# テキストを行単位で読みリストに格納する
def open_text_line(filename):
	with open(filename, 'rt') as f:
		text = f.readlines()
	return text

# 指定したキーワードが何行目にあるか調べる関数
def search_keyword(text, keyword):
	count = []
	for i in range(len(text)):
		line = text[i]
		# 検索キーワードとリストの文字列がある場合append(.findが-1なら無い。ある場合は位置が返ってくる)
		if (keyword in line):
			OneWord = line.split()
			for word in OneWord:
				if ( keyword == word):
					count.append(i)
					return count
				elif( '@' in word ):            #localの変数名のアドレスを探す
					localVariab = word.split('@')
					for localName in localVariab:
						if( keyword == localName):
							count.append(i)
							return count
	if ( count == [] ):
		print("no matching!")

# ファイル読み込み
def find_ValueAaddress(mapfilename,keyword):
	text_line = open_text_line(mapfilename)
	keyword = "_" + keyword

	# 「NODE」という文が何行目にあるか検索する
	lines = search_keyword(text_line, keyword)
	print('Lines that match the keyword:',keyword,lines)
	if( lines != None ):
		test=[]
		test = str(text_line[lines[0]+1])
		test = test.split()
		print(test[0])
		return format(int(test[0],16),'#x')