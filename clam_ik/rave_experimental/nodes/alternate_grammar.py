		self.robot_grammar = nltk.parse_cfg("""
		S -> COMMAND | COMMAND COMMAND | COMMAND CONJ COMMAND | COMMAND COMMAND COMMAND
		CONJ -> 'and'
		COMMAND -> V | V P NP | V NP
		V -> GRAB | POINT | TRAVEL | REPLACE | DROP
		SR -> P NP
		NP -> DET N | DET N SR
		GRAB -> 'grab' | 'pick up' | 'take' | 'get' | 'pick'
		DROP -> PUT N FLOOR | 'drop' NP | 'release' NP
		PUT -> 'put' | 'place'
		REPLACE -> PUT NP SR
		POINT -> POINTER P
		POINTER -> 'point' |  'point' | 'look'
		P -> 'to' | 'at' | 'up' | 'down' | 'here' | 'on' | 'in' | 'beside' | 'next'
		NP -> DET N | DET ADJ N | DET ADJ ADJ N | N | ADJ N | ADJ ADJ N | NP SR
		DET -> 'the' | 'a' | 'this' | 'that' | 'an' | 'my'
		ADJ -> COLOR | SIZE
		COLOR -> 'black' | 'blue' | 'gray' | 'green' | 'red' | 'yellow' | 'white'
		SIZE -> 'small' | 'large'
		N ->  BLOCK | FLOOR | HELD
		HELD -> 'it'
		BLOCK -> 'box' | 'cube' | 'block'
		FLOOR -> 'floor' | 'ground' | 'down'
		TRAVEL -> 'stop' | 'halt' | 'go' | 'come'
		""")

	def interpret_tree(self, tree):
		print tree
		for command in tree:
			flatCommand = str(command)
			color = re.search("\(COLOR (\w+)\)", flatCommand)
			color = "" if color == None else color.group(1)
			noun = re.search("\(N \((\w+)", flatCommand)
			noun = "" if noun == None else noun.group(1)
			print "noun = "+noun
			print "item color = "+color
			if 'GRAB' in flatCommand:
				print "Action: GRAB"
			elif 'DROP' in flatCommand:
				print "Action: DROP"
			elif 'REPLACE' in flatCommand:
				print "Action: REPLACE"
				flatRelation = str(command[0][0][2])  # this should be the spatial relation, can get location info and color
				#print flatRelation
				noun2 = re.search("\(N \((\w+)", flatRelation)
				noun2 = "" if noun2 == None else noun2.group(1)
				color2 = re.search("\(COLOR (\w+)\)", flatRelation)
				color2 = "" if color2 == None else color2.group(1)
				print "landmark = "+noun2
				print "landmark color = "+color2
				#If we only want one candidate... #break

