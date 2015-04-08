class LTL(object):
	"""docstring for LTL"""	

	def __init__(self):
		print('The LTL class is not meant to be instantiated!')

	@staticmethod
	def conj(terms):
		return " & ".join(terms)

	@staticmethod
	def disj(terms):
		return " | ".join(terms)

	@staticmethod
	def neg(term):
		return "! " + term

	@staticmethod
	def next(proposition):
		# return proposition + "'"
		return "next(" + proposition + ")"

	@staticmethod
	def implication(left_hand_side, right_hand_side):
		return left_hand_side + " -> " + right_hand_side 	#TODO: add parentheses to right side ?

	@staticmethod
	def iff(left_hand_side, right_hand_side):
		return left_hand_side + " <-> " + right_hand_side #TODO: add parentheses to right side ?

	@staticmethod
	def paren(term):
		return "(" + term + ")"