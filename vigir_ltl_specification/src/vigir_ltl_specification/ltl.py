class LTL(object):
	"""docstring for LTL"""	

	def __init__(self):
		print('The LTL class is not meant to be instantiated!') #pragma: no cover

	@staticmethod
	def conj(terms):
		if len(terms) > 1:
			return LTL.paren(" & ".join(terms))
		else:
			return terms[0]

	@staticmethod
	def disj(terms):
		if len(terms) > 1:
			return LTL.paren(" | ".join(terms))
		else:
			return terms[0]

	@staticmethod
	def neg(term):
		return "! " + term

	@staticmethod
	def next(proposition):
		# return proposition + "'"
		return "next(" + proposition + ")"

	@staticmethod
	def implication(left_hand_side, right_hand_side):
		return left_hand_side + " -> " + right_hand_side

	@staticmethod
	def iff(left_hand_side, right_hand_side):
		return left_hand_side + " <-> " + right_hand_side

	@staticmethod
	def paren(term):
		return "(" + term + ")"